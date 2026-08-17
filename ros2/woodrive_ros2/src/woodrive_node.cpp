#include "woodrive_ros2/command_utils.hpp"
#include "WooDriveSdk.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "woodrive_ros2/action/auto_motor_setup.hpp"
#include "woodrive_ros2/action/move_position.hpp"
#include "woodrive_ros2/msg/woo_drive_status.hpp"
#include "woodrive_ros2/srv/basic_check.hpp"
#include "woodrive_ros2/srv/motion_command.hpp"

using namespace std::chrono_literals;

class WooDriveNode final : public rclcpp::Node
{
public:
    using MovePosition = woodrive_ros2::action::MovePosition;
    using MoveGoalHandle = rclcpp_action::ServerGoalHandle<MovePosition>;
    using AutoSetup = woodrive_ros2::action::AutoMotorSetup;
    using AutoGoalHandle = rclcpp_action::ServerGoalHandle<AutoSetup>;

    WooDriveNode()
        : Node("woodrive"),
          port_(declare_parameter<std::string>("port", "/dev/ttyUSB0")),
          baudrate_(declare_parameter<int>("baudrate", 9600)),
          targetId_(static_cast<uint8_t>(declare_parameter<int>("target_id", 1))),
          maxRpm_(static_cast<float>(declare_parameter<double>("max_rpm", 100.0))),
          accelMs_(static_cast<uint16_t>(declare_parameter<int>("accel_ms", 1000))),
          decelMs_(static_cast<uint16_t>(declare_parameter<int>("decel_ms", 1000))),
          speedMotionMode_(static_cast<uint8_t>(declare_parameter<int>("motion_mode", 116))),
          subTarget_(static_cast<float>(declare_parameter<double>("sub_target", 10.0))),
          commandTimeout_(declare_parameter<double>("command_timeout_sec", 0.5)),
          defaultPolePairs_(static_cast<uint8_t>(declare_parameter<int>("auto_setup_pole_pairs", 4))),
          serial_(port_.c_str(), baudrate_),
          drive_(serial_, clock_)
    {
        if (!serial_.isOpen()) throw std::runtime_error("Failed to open serial port: " + port_);
        drive_.setTimeout(100);

        uint8_t actualId = 0;
        uint8_t fault = 0;
        if (!drive_.getId(targetId_, actualId) || !drive_.getFault(targetId_, fault)) {
            throw std::runtime_error("WooDrive did not respond");
        }

        statusPub_ = create_publisher<woodrive_ros2::msg::WooDriveStatus>("~/status", 10);
        velocityPub_ = create_publisher<std_msgs::msg::Float32>("~/velocity_rpm", 10);
        busVoltagePub_ = create_publisher<std_msgs::msg::Float32>("~/bus_voltage", 10);
        temperaturePub_ = create_publisher<std_msgs::msg::Float32>("~/temperature", 10);
        faultPub_ = create_publisher<std_msgs::msg::UInt8>("~/fault", 10);

        targetSub_ = create_subscription<std_msgs::msg::Float32>(
            "~/target_rpm", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg) { setTargetRpm(msg->data); });

        checkService_ = create_service<woodrive_ros2::srv::BasicCheck>(
            "~/check",
            [this](const std::shared_ptr<woodrive_ros2::srv::BasicCheck::Request>,
                   std::shared_ptr<woodrive_ros2::srv::BasicCheck::Response> response) {
                basicCheck(response);
            });
        stopService_ = create_service<std_srvs::srv::Trigger>(
            "~/stop",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                std::lock_guard<std::mutex> lock(serialMutex_);
                response->success = stopAndDisableLocked();
                response->message = response->success
                    ? "WooDrive stopped and disabled"
                    : "WooDrive did not acknowledge stop; motor may still be enabled, retrying automatically";
            });
        enableService_ = create_service<std_srvs::srv::SetBool>(
            "~/enable",
            [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                   std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
                std::lock_guard<std::mutex> lock(serialMutex_);
                if (request->data) {
                    const bool ok = drive_.setMotorEnable(targetId_, 1);
                    if (ok) enabled_ = true;
                    response->success = ok;
                    response->message = ok ? "enable state updated" : "WooDrive did not acknowledge";
                } else {
                    response->success = stopAndDisableLocked();
                    response->message = response->success
                        ? "enable state updated"
                        : "WooDrive did not acknowledge disable; motor may still be enabled, retrying automatically";
                }
            });
        brakeService_ = create_service<std_srvs::srv::SetBool>(
            "~/brake",
            [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                   std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
                std::lock_guard<std::mutex> lock(serialMutex_);
                const bool ok = drive_.setMotorBrake(targetId_, request->data ? 1 : 0);
                response->success = ok;
                response->message = ok ? "brake state updated" : "WooDrive did not acknowledge";
            });
        motionCommandService_ = create_service<woodrive_ros2::srv::MotionCommand>(
            "~/motion_command",
            [this](const std::shared_ptr<woodrive_ros2::srv::MotionCommand::Request> request,
                   std::shared_ptr<woodrive_ros2::srv::MotionCommand::Response> response) {
                handleMotionCommand(request, response);
            });

        relativeAction_ = makeMoveServer("~/move_relative", true);
        absoluteAction_ = makeMoveServer("~/move_absolute", false);
        autoSetupAction_ = rclcpp_action::create_server<AutoSetup>(
            this, "~/auto_setup",
            [this](const auto&, const auto&) {
                return operationActive_ ? rclcpp_action::GoalResponse::REJECT
                                        : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            [](const auto) { return rclcpp_action::CancelResponse::ACCEPT; },
            [this](const std::shared_ptr<AutoGoalHandle> handle) {
                addWorker([this, handle] { executeAutoSetup(handle); });
            });

        lastCommand_ = now();
        monitorTimer_ = create_wall_timer(200ms, [this] { monitor(); });
        RCLCPP_INFO(get_logger(),
                    "Connected to WooDrive ID %u on %s at %d baud (startup fault 0x%02X)",
                    static_cast<unsigned>(actualId), port_.c_str(), baudrate_, fault);
    }

    ~WooDriveNode() override
    {
        shuttingDown_ = true;
        {
            std::lock_guard<std::mutex> lock(workerMutex_);
            for (auto& worker : workers_) if (worker.joinable()) worker.join();
        }
        std::lock_guard<std::mutex> lock(serialMutex_);
        stopAndDisableLocked();
    }

private:
    rclcpp_action::Server<MovePosition>::SharedPtr makeMoveServer(const std::string& name,
                                                                  bool relative)
    {
        return rclcpp_action::create_server<MovePosition>(
            this, name,
            [this](const auto&, const std::shared_ptr<const MovePosition::Goal> goal) {
                if (operationActive_ || !std::isfinite(goal->target_deg) ||
                    !std::isfinite(goal->max_rpm)) {
                    return rclcpp_action::GoalResponse::REJECT;
                }
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            [](const auto) { return rclcpp_action::CancelResponse::ACCEPT; },
            [this, relative](const std::shared_ptr<MoveGoalHandle> handle) {
                addWorker([this, handle, relative] { executeMove(handle, relative); });
            });
    }

    template<typename Function>
    void addWorker(Function&& function)
    {
        std::lock_guard<std::mutex> lock(workerMutex_);
        workers_.emplace_back(std::forward<Function>(function));
    }

    bool interruptibleSleep(double seconds, const std::function<bool()>& canceled)
    {
        const int steps = std::max(1, static_cast<int>(seconds * 10.0));
        for (int i = 0; i < steps; ++i) {
            if (shuttingDown_ || canceled()) return false;
            std::this_thread::sleep_for(100ms);
        }
        return true;
    }

    void basicCheck(const std::shared_ptr<woodrive_ros2::srv::BasicCheck::Response>& response)
    {
        std::lock_guard<std::mutex> lock(serialMutex_);
        uint8_t actualId = 0;
        uint8_t fault = 0;
        response->success = drive_.getId(targetId_, actualId) && drive_.getFault(targetId_, fault);
        response->id = actualId;
        response->fault = fault;
        response->message = response->success ? "WooDrive communication OK"
                                              : "WooDrive communication failed";
    }

    void setTargetRpm(float signedRpm)
    {
        if (operationActive_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Ignoring RPM command while an action is active");
            return;
        }
        std::lock_guard<std::mutex> lock(serialMutex_);
        lastCommand_ = now();
        const auto command = woodrive_ros2::makeSpeedCommand(signedRpm, maxRpm_);
        if (command.direction == 0) {
            stopAndDisableLocked();
            return;
        }
        if (!enableLocked() ||
            !drive_.setMotorMotionAll(targetId_, accelMs_, decelMs_, speedMotionMode_,
                                      subTarget_, command.rpm, command.direction)) {
            RCLCPP_ERROR(get_logger(), "Failed to send RPM command");
            stopAndDisableLocked();
        }
    }

    // Generic passthrough to setMotorMotionAll() for the ~/motion_command
    // service -- see srv/MotionCommand.srv for the field docs and the
    // "live command, dead-man's-switch applies" contract this shares with
    // setTargetRpm(). Rejects anything operationActive_ (a move/auto_setup
    // action is running) or outside isAllowedMotionMode()'s Velocity/
    // Position family allowlist.
    void handleMotionCommand(const std::shared_ptr<woodrive_ros2::srv::MotionCommand::Request>& request,
                             const std::shared_ptr<woodrive_ros2::srv::MotionCommand::Response>& response)
    {
        if (operationActive_) {
            response->success = false;
            response->message = "an action (move/auto_setup) is active";
            return;
        }
        if (request->direction > 2) {
            response->success = false;
            response->message = "direction must be 0 (DIR_ZERO), 1 (CCW/+), or 2 (CW/-)";
            return;
        }
        if (request->direction != 0 && !woodrive_ros2::isAllowedMotionMode(request->motion_mode)) {
            response->success = false;
            response->message = "motion_mode not in the allowed Velocity/Position family set "
                                "(Voltage/Current direct-drive modes are rejected)";
            return;
        }

        std::lock_guard<std::mutex> lock(serialMutex_);
        lastCommand_ = now();

        if (request->direction == 0) {
            response->success = stopAndDisableLocked();
            response->message = response->success ? "DIR_ZERO sent (decelerate/home)"
                                                   : "WooDrive did not acknowledge";
            return;
        }

        if (!enableLocked() ||
            !drive_.setMotorMotionAll(targetId_, request->accel_ms, request->decel_ms,
                                      request->motion_mode, request->sub_target,
                                      request->main_target, request->direction)) {
            response->success = false;
            response->message = "WooDrive did not acknowledge motion command";
            stopAndDisableLocked();
            return;
        }
        response->success = true;
        response->message = "motion command sent; will auto-stop if not repeated within command_timeout_sec";
    }

    bool enableLocked()
    {
        if (enabled_) return true;
        if (!drive_.setMotorBrake(targetId_, 1) || !drive_.setMotorEnable(targetId_, 1)) {
            return false;
        }
        enabled_ = true;
        return true;
    }

    // Returns whether the drive actually acknowledged both commands. On
    // failure enabled_ is left untouched (not forced to false) so monitor()'s
    // timeout/fault checks keep retrying the stop every cycle instead of
    // wrongly assuming the motor is already safe.
    bool stopAndDisableLocked()
    {
        const bool enableOk = drive_.setMotorEnable(targetId_, 0);
        const bool brakeOk = drive_.setMotorBrake(targetId_, 1);
        if (enableOk && brakeOk) {
            enabled_ = false;
        }
        return enableOk && brakeOk;
    }

    void monitor()
    {
        if (operationActive_) return;
        std::lock_guard<std::mutex> lock(serialMutex_);
        if (enabled_ && (now() - lastCommand_).seconds() > commandTimeout_) {
            RCLCPP_WARN(get_logger(), "Command timeout; stopping motor");
            stopAndDisableLocked();
        }

        uint8_t fault = 0;
        WooDrive::MotorStatus status{};
        if (!drive_.getFault(targetId_, fault) || !drive_.getMotorStatusAll(targetId_, status)) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "WooDrive status read failed");
            if (enabled_) stopAndDisableLocked();
            return;
        }
        publishStatus(status, fault);
        if (fault != 0 && enabled_) {
            RCLCPP_ERROR(get_logger(), "WooDrive fault 0x%02X; stopping motor", fault);
            stopAndDisableLocked();
        }
    }

    void publishStatus(const WooDrive::MotorStatus& status, uint8_t fault)
    {
        woodrive_ros2::msg::WooDriveStatus message;
        message.header.stamp = now();
        message.id = targetId_;
        message.fault = fault;
        message.enabled = enabled_;
        message.total_time_ms = status.totalTime;
        message.elapsed_time_ms = status.elapsedTime;
        message.remain_time_ms = status.remainTime;
        message.position_deg = status.position;
        message.velocity_rpm = status.velocity;
        message.iq_current_a = status.iqCurrent;
        message.id_current_a = status.idCurrent;
        message.bus_current_a = status.busCurrent;
        message.vq_voltage_v = status.vqVoltage;
        message.vd_voltage_v = status.vdVoltage;
        message.bus_voltage_v = status.busVoltage;
        message.temperature_c = status.temperature;
        message.pulse_count = status.pulseCount;
        message.u_phase_current_a = status.uPhaseCurrent;
        message.w_phase_current_a = status.wPhaseCurrent;
        statusPub_->publish(message);

        std_msgs::msg::Float32 value;
        value.data = status.velocity;
        velocityPub_->publish(value);
        value.data = status.busVoltage;
        busVoltagePub_->publish(value);
        value.data = status.temperature;
        temperaturePub_->publish(value);
        std_msgs::msg::UInt8 faultMessage;
        faultMessage.data = fault;
        faultPub_->publish(faultMessage);
    }

    void executeMove(const std::shared_ptr<MoveGoalHandle>& handle, bool relative)
    {
        bool expected = false;
        if (!operationActive_.compare_exchange_strong(expected, true)) {
            auto result = std::make_shared<MovePosition::Result>();
            result->message = "another operation is active";
            handle->abort(result);
            return;
        }
        struct ClearFlag { std::atomic_bool& flag; ~ClearFlag() { flag = false; } } clear{operationActive_};

        const auto goal = handle->get_goal();
        const uint8_t mode = relative ? 245 : 244;
        const float magnitude = std::abs(goal->target_deg);
        const uint8_t direction = goal->target_deg >= 0.0f ? 1 : 2;
        const float rpm = std::clamp(goal->max_rpm > 0.0f ? goal->max_rpm : 20.0f,
                                     1.0f, maxRpm_);
        const uint16_t accel = goal->accel_ms > 0 ? goal->accel_ms : accelMs_;
        const uint16_t decel = goal->decel_ms > 0 ? goal->decel_ms : decelMs_;
        const double timeout = goal->timeout_sec > 0.0f ? goal->timeout_sec : 30.0;
        auto result = std::make_shared<MovePosition::Result>();

        auto cancelRequested = [&] { return handle->is_canceling(); };

        // Force a clean, stopped state before starting a new position move.
        // NOTE: this used to send setMotorMotionAll(..., dir=0) here as a
        // "harmless" priming step, but per the WooDrive protocol (address
        // 0x9B) direction value 0x00 is DIR_ZERO -- a real command that
        // drives the motor to absolute position 0, not a no-op. Use the
        // enable/brake registers directly instead; they cannot command motion.
        {
            std::lock_guard<std::mutex> lock(serialMutex_);
            stopAndDisableLocked();
        }
        if (!interruptibleSleep(1.0, cancelRequested)) return cancelMove(handle, result);

        WooDrive::MotorStatus start{};
        {
            std::lock_guard<std::mutex> lock(serialMutex_);
            if (!enableLocked() || !drive_.getMotorStatusAll(targetId_, start) ||
                !drive_.setMotorMotionAll(targetId_, accel, decel, mode, rpm, magnitude, direction)) {
                result->message = "failed to start position move";
                stopAndDisableLocked();
                handle->abort(result);
                return;
            }
        }

        const auto begin = std::chrono::steady_clock::now();
        int settledSamples = 0;
        bool success = false;
        WooDrive::MotorStatus current{};
        uint8_t fault = 0;
        while (!shuttingDown_) {
            if (handle->is_canceling()) return cancelMove(handle, result);
            {
                std::lock_guard<std::mutex> lock(serialMutex_);
                if (!drive_.getFault(targetId_, fault) ||
                    !drive_.getMotorStatusAll(targetId_, current)) break;
            }
            const float moved = current.position - start.position;
            auto feedback = std::make_shared<MovePosition::Feedback>();
            feedback->current_position_deg = current.position;
            feedback->moved_deg = moved;
            feedback->velocity_rpm = current.velocity;
            feedback->fault = fault;
            handle->publish_feedback(feedback);

            if (fault != 0) break;
            const bool enoughTravel = !relative || magnitude < 1.0f ||
                                      std::abs(moved) >= magnitude * 0.8f;
            if (enoughTravel && std::abs(current.velocity) < 1.0f) {
                if (++settledSamples >= 5) { success = true; break; }
            } else {
                settledSamples = 0;
            }
            if (relative && magnitude > 0.0f && std::abs(moved) > magnitude * 1.5f + 10.0f) break;
            if (std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count() > timeout) break;
            std::this_thread::sleep_for(100ms);
        }

        {
            std::lock_guard<std::mutex> lock(serialMutex_);
            stopAndDisableLocked();
        }
        result->success = success;
        result->final_position_deg = current.position;
        result->moved_deg = current.position - start.position;
        result->fault = fault;
        result->message = success ? (relative ? "relative move complete" : "absolute move complete")
                                  : "position move failed, faulted, or timed out";
        if (success) handle->succeed(result); else handle->abort(result);
    }

    void cancelMove(const std::shared_ptr<MoveGoalHandle>& handle,
                    const std::shared_ptr<MovePosition::Result>& result)
    {
        std::lock_guard<std::mutex> lock(serialMutex_);
        stopAndDisableLocked();
        result->message = "position move canceled";
        handle->canceled(result);
    }

    void executeAutoSetup(const std::shared_ptr<AutoGoalHandle>& handle)
    {
        bool expected = false;
        if (!operationActive_.compare_exchange_strong(expected, true)) {
            auto result = std::make_shared<AutoSetup::Result>();
            result->message = "another operation is active";
            handle->abort(result);
            return;
        }
        struct ClearFlag { std::atomic_bool& flag; ~ClearFlag() { flag = false; } } clear{operationActive_};
        const auto goal = handle->get_goal();
        const double timeout = goal->timeout_sec > 0.0f ? goal->timeout_sec : 240.0;
        const uint8_t polePairs = goal->pole_pairs > 0 ? goal->pole_pairs : defaultPolePairs_;
        auto result = std::make_shared<AutoSetup::Result>();

        {
            std::lock_guard<std::mutex> lock(serialMutex_);
            stopAndDisableLocked();
            if (!drive_.setPolePairs(targetId_, polePairs) ||
                !drive_.setAutoMotorSetup(targetId_, 0)) {
                result->message = "failed to start auto motor setup";
                handle->abort(result);
                return;
            }
        }

        const auto begin = std::chrono::steady_clock::now();
        uint8_t setupState = 0xFF;
        bool responded = false;
        while (!shuttingDown_) {
            if (handle->is_canceling()) {
                result->message = "auto setup canceled; controller setup may still be running";
                handle->canceled(result);
                return;
            }
            const double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - begin).count();
            if (elapsed > timeout) break;
            {
                std::lock_guard<std::mutex> lock(serialMutex_);
                responded = drive_.getAutoMotorSetup(targetId_, setupState);
            }
            auto feedback = std::make_shared<AutoSetup::Feedback>();
            feedback->elapsed_sec = static_cast<float>(elapsed);
            feedback->state = responded ? "controller responded" : "controller busy";
            handle->publish_feedback(feedback);
            if (responded) break;
            if (!interruptibleSleep(5.0, [&] { return handle->is_canceling(); })) continue;
        }

        uint8_t fault = 0xFF;
        {
            std::lock_guard<std::mutex> lock(serialMutex_);
            if (responded) drive_.getFault(targetId_, fault);
        }
        result->success = responded && fault == 0;
        result->setup_state = setupState;
        result->fault = fault;
        result->message = result->success ? "auto motor setup complete"
                                          : "auto motor setup failed or timed out";
        if (result->success) handle->succeed(result); else handle->abort(result);
    }

    std::string port_;
    int baudrate_;
    uint8_t targetId_;
    float maxRpm_;
    uint16_t accelMs_;
    uint16_t decelMs_;
    uint8_t speedMotionMode_;
    float subTarget_;
    double commandTimeout_;
    uint8_t defaultPolePairs_;
    PosixSerialTransport serial_;
    StdClock clock_;
    WooDrive drive_;
    std::mutex serialMutex_;
    bool enabled_{false};
    rclcpp::Time lastCommand_;
    std::atomic_bool operationActive_{false};
    std::atomic_bool shuttingDown_{false};
    std::mutex workerMutex_;
    std::vector<std::thread> workers_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr targetSub_;
    rclcpp::Publisher<woodrive_ros2::msg::WooDriveStatus>::SharedPtr statusPub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr velocityPub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr busVoltagePub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr temperaturePub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr faultPub_;
    rclcpp::Service<woodrive_ros2::srv::BasicCheck>::SharedPtr checkService_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stopService_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enableService_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr brakeService_;
    rclcpp::Service<woodrive_ros2::srv::MotionCommand>::SharedPtr motionCommandService_;
    rclcpp_action::Server<MovePosition>::SharedPtr relativeAction_;
    rclcpp_action::Server<MovePosition>::SharedPtr absoluteAction_;
    rclcpp_action::Server<AutoSetup>::SharedPtr autoSetupAction_;
    rclcpp::TimerBase::SharedPtr monitorTimer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<WooDriveNode>());
    } catch (const std::exception& error) {
        RCLCPP_FATAL(rclcpp::get_logger("woodrive"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
