// Stage 1: left/right WooDrive dual-axis control foundation for the
// 2-wheel AMR. NOT yet a drivable robot -- no /cmd_vel, no differential-
// drive kinematics, no odometry/TF. See README.md's "Not implemented yet"
// section for the full list and what Stage 2 needs.
//
// Lives in its own package (amr_2wheel_robot, under ros2/projects/),
// separate from woodrive_ros2 (the generic, robot-agnostic WooDrive SDK
// package) -- this file is robot-specific (hardcodes the "left/right
// wheel" concept), so it doesn't belong inside the SDK's package. It links
// directly against core/ (the same SDK woodrive_ros2 uses), not against
// the woodrive_ros2 package itself -- this node's comm/control loop is
// designed for a 1-10ms cadence (see below), which going through
// woodrive_ros2's topics/services would add ROS2/DDS serialization
// overhead against; core/'s plain C++ calls don't have that cost. No
// WooDrive protocol logic is reimplemented here -- every register access
// goes through core/WooDriveSdk.h's existing high-level API
// (setMotorMotionAllSigned(), setMotorEnable(), setMotorBrake(),
// getFault(), getMotorStatusAll(), getId()).
//
// Architecture (carried over from the earlier comm/control-thread
// prototype this file grew out of, verified on real hardware at ~12-13ms
// per left+right poll cycle with zero extra OS scheduling tuning): a
// dedicated comm thread owns the RS-485 bus exclusively -- it is the only
// code in this file that ever calls into `drive_`. It polls both wheel
// IDs back-to-back into a mutex-protected per-wheel cache (WheelState) and
// drains a one-slot pending-command queue per wheel between polls. Every
// ROS2 callback (the target_rpm subscriptions, the enable/stop services,
// the watchdog timer) only reads that cache or queues a command -- none of
// them ever block on a serial round-trip.
//
// Run (directly, without a launch file):
//   ros2 run amr_2wheel_robot dual_motor_node --ros-args --params-file config/amr_2wheel.yaml
// or see launch/amr_2wheel.launch.py / README.md for the normal way to run this.

#include "WooDriveSdk.h"
#include "amr_2wheel_robot/motor_command_utils.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <stdexcept>
#include <string>
#include <thread>

using namespace std::chrono_literals;

namespace amr_2wheel_robot {

// One wheel's worth of cached state + pending command slots. Two of these
// (left, right) make up the full shared state comm/control talk through.
// Every field access must hold `mutex`.
struct WheelState
{
    std::mutex mutex;

    // Written by the comm thread, read by ROS callbacks (status topics,
    // the fault-triggered safety stop).
    uint8_t fault = 0xFF;
    WooDrive::MotorStatus status{};
    bool haveData = false;
    bool enabled = false;

    // Written by ROS callbacks, drained by the comm thread.
    std::optional<float> pendingSignedRpm;
    std::optional<bool> pendingEnable;

    // Written by whichever callback last issued a speed command; read by
    // the watchdog timer to decide whether this wheel has gone silent.
    rclcpp::Time lastCommandTime;
};

class DualMotorNode : public rclcpp::Node
{
public:
    DualMotorNode()
    : Node("dual_motor_node"),
      port_(declare_parameter<std::string>("port", "/dev/ttyUSB0")),
      baudrate_(declare_parameter<int>("baudrate", 1000000)),
      leftId_(static_cast<uint8_t>(declare_parameter<int>("left_id", 1))),
      rightId_(static_cast<uint8_t>(declare_parameter<int>("right_id", 2))),
      leftInvert_(declare_parameter<bool>("left_invert", false)),
      rightInvert_(declare_parameter<bool>("right_invert", false)),
      // 116 (0x74) = Vel Curr Abs: current-limited absolute velocity target
      // -- the same default speed_node.cpp / woodrive_node.cpp use. Not in
      // the task's minimum config list, but setMotorMotionAllSigned()
      // structurally requires a mode; this is the sane default for "spin
      // this wheel at N rpm".
      motionMode_(static_cast<uint8_t>(declare_parameter<int>("motion_mode", 116))),
      // Current limit (A) for that mode -- also structurally required,
      // not in the task's list. Conservative default; raise via parameter
      // once real load characteristics are known.
      subTarget_(static_cast<float>(declare_parameter<double>("sub_target", 5.0))),
      accelMs_(static_cast<uint16_t>(declare_parameter<int>("accel_ms", 500))),
      decelMs_(static_cast<uint16_t>(declare_parameter<int>("decel_ms", 500))),
      maxRpm_(static_cast<float>(declare_parameter<double>("max_rpm", 100.0))),
      testRpm_(static_cast<float>(declare_parameter<double>("test_rpm", 20.0))),
      commandTimeoutSec_(declare_parameter<int>("command_timeout_ms", 500) / 1000.0),
      serial_(port_.c_str(), baudrate_),
      drive_(serial_, clock_)
    {
        if (!serial_.isOpen()) {
            throw std::runtime_error("Failed to open serial port: " + port_);
        }
        drive_.setTimeout(200);

        // Startup reachability check (per-device, so a failure names
        // exactly which controller didn't answer).
        const bool leftOk = checkReachable(leftId_, "LEFT");
        const bool rightOk = checkReachable(rightId_, "RIGHT");
        if (!leftOk || !rightOk) {
            throw std::runtime_error(
                "One or both WooDrive controllers did not respond -- see LEFT/RIGHT log lines above");
        }

        left_.lastCommandTime = now();
        right_.lastCommandTime = now();

        leftVelocityPub_ = create_publisher<std_msgs::msg::Float32>("~/left/velocity_rpm", 10);
        rightVelocityPub_ = create_publisher<std_msgs::msg::Float32>("~/right/velocity_rpm", 10);
        leftFaultPub_ = create_publisher<std_msgs::msg::UInt8>("~/left/fault", 10);
        rightFaultPub_ = create_publisher<std_msgs::msg::UInt8>("~/right/fault", 10);

        // Test interface (task 4D): plain std_msgs/std_srvs, no custom
        // msg/srv -- sufficient for "let an operator type in a left/right
        // rpm and enable/disable each side", which is all Stage 1 needs.
        leftTargetSub_ = create_subscription<std_msgs::msg::Float32>(
            "~/left/target_rpm", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg) { onTargetRpm(left_, leftInvert_, msg->data); });
        rightTargetSub_ = create_subscription<std_msgs::msg::Float32>(
            "~/right/target_rpm", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg) { onTargetRpm(right_, rightInvert_, msg->data); });

        leftEnableSrv_ = create_service<std_srvs::srv::SetBool>(
            "~/left/enable",
            [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                   std::shared_ptr<std_srvs::srv::SetBool::Response> res) { onEnable(left_, "LEFT", req, res); });
        rightEnableSrv_ = create_service<std_srvs::srv::SetBool>(
            "~/right/enable",
            [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                   std::shared_ptr<std_srvs::srv::SetBool::Response> res) { onEnable(right_, "RIGHT", req, res); });
        stopSrv_ = create_service<std_srvs::srv::Trigger>(
            "~/stop",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> res) { onStop(res); });

        running_ = true;
        commThread_ = std::thread(&DualMotorNode::commLoop, this);

        watchdogTimer_ = create_wall_timer(100ms, [this] { watchdogTick(); });

        RCLCPP_INFO(get_logger(),
                    "dual_motor_node ready -- left id=%u (invert=%s) right id=%u (invert=%s) "
                    "port=%s baud=%d mode=%u current_limit=%.1fA max_rpm=%.1f timeout=%.0fms",
                    leftId_, leftInvert_ ? "true" : "false", rightId_, rightInvert_ ? "true" : "false",
                    port_.c_str(), baudrate_, motionMode_, subTarget_, maxRpm_, commandTimeoutSec_ * 1000.0);
    }

    ~DualMotorNode() override
    {
        running_ = false;
        if (commThread_.joinable()) commThread_.join();
        // Comm thread has stopped -- no one else touches drive_, so it's
        // safe to call it directly here without the mutex. Task 5: 0 rpm
        // first, then disable, then brake.
        finalStop(leftId_);
        finalStop(rightId_);
        RCLCPP_INFO(get_logger(), "dual_motor_node shut down -- both wheels stopped and disabled");
    }

private:
    bool checkReachable(uint8_t id, const char* label)
    {
        uint8_t idRead = 0;
        if (!drive_.getId(id, idRead)) {
            RCLCPP_ERROR(get_logger(), "%s WooDrive (id=%u) did not respond -- check wiring/ID/baud rate",
                        label, id);
            return false;
        }
        RCLCPP_INFO(get_logger(), "%s WooDrive (id=%u) OK", label, id);
        return true;
    }

    void onTargetRpm(WheelState& wheel, bool invert, float requestedRpm)
    {
        const float rpm = applyInvertAndClamp(requestedRpm, invert, maxRpm_);
        std::lock_guard<std::mutex> lock(wheel.mutex);
        wheel.pendingSignedRpm = rpm;
        wheel.lastCommandTime = now();
    }

    void onEnable(WheelState& wheel, const char* label,
                 const std::shared_ptr<std_srvs::srv::SetBool::Request>& req,
                 const std::shared_ptr<std_srvs::srv::SetBool::Response>& res)
    {
        {
            std::lock_guard<std::mutex> lock(wheel.mutex);
            wheel.pendingEnable = req->data;
            if (!req->data) wheel.pendingSignedRpm = 0.0f;  // disabling also zeroes any pending target
            wheel.lastCommandTime = now();
        }
        res->success = true;
        res->message = std::string(label) + (req->data ? ": enable requested" : ": disable requested");
    }

    void onStop(const std::shared_ptr<std_srvs::srv::Trigger::Response>& res)
    {
        queueStop(left_);
        queueStop(right_);
        res->success = true;
        res->message = "stop requested for both wheels";
    }

    void queueStop(WheelState& wheel)
    {
        std::lock_guard<std::mutex> lock(wheel.mutex);
        wheel.pendingSignedRpm = 0.0f;
        wheel.pendingEnable = false;
    }

    // ===== Comm thread: the ONLY code in this file that touches the bus =====
    void commLoop()
    {
        auto pollAndCommand = [this](uint8_t id, WheelState& wheel) {
            WooDrive::MotorStatus status{};
            uint8_t fault = 0;
            if (drive_.getFault(id, fault) && drive_.getMotorStatusAll(id, status)) {
                std::lock_guard<std::mutex> lock(wheel.mutex);
                wheel.fault = fault;
                wheel.status = status;
                wheel.haveData = true;
            }

            std::optional<bool> enableCmd;
            std::optional<float> rpmCmd;
            {
                std::lock_guard<std::mutex> lock(wheel.mutex);
                enableCmd = wheel.pendingEnable;
                wheel.pendingEnable.reset();
                rpmCmd = wheel.pendingSignedRpm;
                wheel.pendingSignedRpm.reset();
            }

            if (enableCmd.has_value()) {
                bool ok;
                if (*enableCmd) {
                    ok = drive_.setMotorBrake(id, 1) && drive_.setMotorEnable(id, 1);
                } else {
                    ok = drive_.setMotorEnable(id, 0) && drive_.setMotorBrake(id, 1);
                }
                if (ok) {
                    std::lock_guard<std::mutex> lock(wheel.mutex);
                    wheel.enabled = *enableCmd;
                }
            }
            // setMotorMotionAllSigned() sets accel/decel/mode/current-limit
            // and the signed target in one call -- unlike the bare
            // setMotorMainTargetSigned() this file used to call, which only
            // ever set the target/direction and silently relied on
            // whatever motion mode happened to already be configured on
            // the controller from an earlier session.
            if (rpmCmd.has_value()) {
                drive_.setMotorMotionAllSigned(id, accelMs_, decelMs_, motionMode_, subTarget_, *rpmCmd);
            }
        };

        while (running_) {
            pollAndCommand(leftId_, left_);
            pollAndCommand(rightId_, right_);
            std::this_thread::sleep_for(5ms);  // bus-limited cadence -- always sleeps, never busy-waits
        }
    }

    // Runs on the ROS2 executor (wall timer callback) -- never touches the
    // bus directly, only reads the cache / queues commands like any other
    // callback.
    void watchdogTick()
    {
        checkTimeout(left_);
        checkTimeout(right_);
        publishStatus(left_, leftVelocityPub_, leftFaultPub_);
        publishStatus(right_, rightVelocityPub_, rightFaultPub_);

        uint8_t leftFault = 0, rightFault = 0;
        bool haveLeft = false, haveRight = false;
        {
            std::lock_guard<std::mutex> lock(left_.mutex);
            leftFault = left_.fault;
            haveLeft = left_.haveData;
        }
        {
            std::lock_guard<std::mutex> lock(right_.mutex);
            rightFault = right_.fault;
            haveRight = right_.haveData;
        }

        // If either wheel reports a fault, stop BOTH. A 2-wheel robot
        // isn't safely controllable on one working wheel alone, so there's
        // no reason to keep the healthy side driving.
        const bool faulted = (haveLeft && leftFault != 0) || (haveRight && rightFault != 0);
        if (faulted) {
            if (!faultStopLatched_) {
                RCLCPP_ERROR(get_logger(), "fault detected (left=0x%02X right=0x%02X) -- stopping both wheels",
                            leftFault, rightFault);
                faultStopLatched_ = true;
            }
            queueStop(left_);
            queueStop(right_);
        } else {
            faultStopLatched_ = false;
        }
    }

    void checkTimeout(WheelState& wheel)
    {
        std::lock_guard<std::mutex> lock(wheel.mutex);
        if (wheel.enabled && (now() - wheel.lastCommandTime).seconds() > commandTimeoutSec_) {
            // Re-queued every tick until a fresh command arrives -- cheap
            // (setMotorMotionAllSigned to 0 rpm is idempotent) and keeps
            // the wheel held at 0 rpm rather than firing once and going
            // silent again.
            wheel.pendingSignedRpm = 0.0f;
        }
    }

    void publishStatus(WheelState& wheel,
                       const rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr& velocityPub,
                       const rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr& faultPub)
    {
        float velocity = 0.0f;
        uint8_t fault = 0;
        bool haveData;
        {
            std::lock_guard<std::mutex> lock(wheel.mutex);
            velocity = wheel.status.velocity;
            fault = wheel.fault;
            haveData = wheel.haveData;
        }
        if (!haveData) return;
        std_msgs::msg::Float32 v;
        v.data = velocity;
        velocityPub->publish(v);
        std_msgs::msg::UInt8 f;
        f.data = fault;
        faultPub->publish(f);
    }

    // Only safe to call once commThread_ has been joined (destructor).
    void finalStop(uint8_t id)
    {
        drive_.setMotorMotionAllSigned(id, accelMs_, decelMs_, motionMode_, subTarget_, 0.0f);
        drive_.setMotorEnable(id, 0);
        drive_.setMotorBrake(id, 1);
    }

    // Parameters
    std::string port_;
    int baudrate_;
    uint8_t leftId_;
    uint8_t rightId_;
    bool leftInvert_;
    bool rightInvert_;
    uint8_t motionMode_;
    float subTarget_;
    uint16_t accelMs_;
    uint16_t decelMs_;
    float maxRpm_;
    float testRpm_;
    double commandTimeoutSec_;

    PosixSerialTransport serial_;
    StdClock clock_;
    WooDrive drive_;

    WheelState left_;
    WheelState right_;
    bool faultStopLatched_ = false;

    std::atomic_bool running_{false};
    std::thread commThread_;
    rclcpp::TimerBase::SharedPtr watchdogTimer_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr leftVelocityPub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rightVelocityPub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr leftFaultPub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr rightFaultPub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr leftTargetSub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rightTargetSub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr leftEnableSrv_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr rightEnableSrv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stopSrv_;
};

}  // namespace amr_2wheel_robot

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        // Passed as a temporary (not bound to a local variable) so it is
        // destroyed at the end of this statement -- strictly before
        // rclcpp::shutdown() below. A node destroyed after shutdown() tears
        // down against an already-finalized rmw/DDS context and segfaults.
        rclcpp::spin(std::make_shared<amr_2wheel_robot::DualMotorNode>());
    } catch (const std::exception& error) {
        RCLCPP_FATAL(rclcpp::get_logger("dual_motor_node"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
