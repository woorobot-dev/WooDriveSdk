// Growing playground node -- starting point for the 2-wheel diff-drive
// robot. Not part of the raspberrypi/arduino Example01-05 parity set (see
// examples/README.md for those); this one is expected to keep growing:
// dual-motor comm/control now, /cmd_vel -> per-wheel kinematics and /odom
// later, eventually becoming (or feeding) a dedicated woodrive_diffdrive
// package. See dev/README.md for the overall plan.
//
// Architecture (see the session's comm_control_demo.cpp prototype this is
// based on): a dedicated comm thread owns the RS-485 bus exclusively and
// polls both wheel IDs back-to-back into a mutex-protected shared cache;
// SET commands queue up and get drained by that same thread between polls.
// The "control" side (here: main's own loop, standing in for what will
// later be a timer callback / cmd_vel subscription) never touches the bus
// directly -- it only reads the cache and pushes commands onto the queue.
// This is what makes a 1-10ms control cadence possible: the control side
// is never blocked waiting on a serial round-trip.
//
// Defaults to 1,000,000 bps -- the whole reason to split into two IDs on
// one bus is this tight loop, which needs the faster bus (see dev/README.md
// for why 9600 makes a 1-10ms cycle physically impossible).
//
// Run:
//   ros2 run woodrive_ros2 dual_motor_node --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=1000000 -p left_id:=1 -p right_id:=2

#include "WooDriveSdk.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <thread>

using namespace std::chrono_literals;

namespace {

rclcpp::Node::SharedPtr g_node;
std::atomic<bool> g_running{true};

// One wheel's worth of cached state + a pending command slot. Two of these
// (left, right) make up the full shared state comm/control talk through.
struct WheelState {
    std::mutex mutex;
    uint8_t fault = 0xFF;
    WooDrive::MotorStatus status{};
    bool haveData = false;
    int pollCount = 0;

    // Set by control, drained by comm. std::optional as a simple one-slot
    // queue (latest command wins) -- fine for a playground; a real queue
    // would matter if every intermediate command needed to be honored.
    std::optional<float> pendingSignedRpm;
};

// ===== Comm thread: the ONLY code in this file that touches the bus =====
void commThreadFunc(WooDrive& drive, uint8_t leftId, uint8_t rightId,
                    WheelState& left, WheelState& right)
{
    auto pollAndCommand = [&drive](uint8_t id, WheelState& wheel) {
        WooDrive::MotorStatus status{};
        uint8_t fault = 0;
        if (drive.getFault(id, fault) && drive.getMotorStatusAll(id, status)) {
            std::lock_guard<std::mutex> lock(wheel.mutex);
            wheel.fault = fault;
            wheel.status = status;
            wheel.haveData = true;
            wheel.pollCount++;
        }

        std::optional<float> rpm;
        {
            std::lock_guard<std::mutex> lock(wheel.mutex);
            rpm = wheel.pendingSignedRpm;
            wheel.pendingSignedRpm.reset();
        }
        if (rpm.has_value()) {
            drive.setMotorMainTargetSigned(id, *rpm);
        }
    };

    while (g_running) {
        pollAndCommand(leftId, left);
        pollAndCommand(rightId, right);
        std::this_thread::sleep_for(5ms);  // bus-limited cadence, see dev/README.md
    }
    RCLCPP_INFO(g_node->get_logger(), "[comm] thread exiting");
}

void printWheel(const char* label, const WheelState& wheelConst)
{
    // lock_guard needs a non-const mutex reference; wheelConst is only
    // logically read-only from this function's point of view.
    auto& wheel = const_cast<WheelState&>(wheelConst);
    std::lock_guard<std::mutex> lock(wheel.mutex);
    if (!wheel.haveData) {
        RCLCPP_INFO(g_node->get_logger(), "[%s] no data yet", label);
        return;
    }
    RCLCPP_INFO(g_node->get_logger(),
                "[%s] fault=0x%02X pos=%.2fdeg vel=%.2frpm iq=%.3fA temp=%.1fC (comm polled it %d times)",
                label, wheel.fault, wheel.status.position, wheel.status.velocity,
                wheel.status.iqCurrent, wheel.status.temperature, wheel.pollCount);
}

// All node-owning work lives here so g_node is reset (and the underlying
// rclcpp::Node destroyed) strictly before rclcpp::shutdown() runs in main()
// -- see the other examples for why (node-destroyed-after-shutdown segfault).
int run()
{
    const std::string port = g_node->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    const int baudrate = g_node->declare_parameter<int>("baudrate", 1000000);
    const uint8_t leftId = static_cast<uint8_t>(g_node->declare_parameter<int>("left_id", 1));
    const uint8_t rightId = static_cast<uint8_t>(g_node->declare_parameter<int>("right_id", 2));

    PosixSerialTransport serial(port.c_str(), baudrate);
    if (!serial.isOpen()) {
        RCLCPP_ERROR(g_node->get_logger(), "serial open failed: %s", port.c_str());
        return 1;
    }

    StdClock wooClock;
    WooDrive drive(serial, wooClock);
    drive.setTimeout(200);

    RCLCPP_INFO(g_node->get_logger(), "=== WooDrive dual_motor_node (left id=%u, right id=%u, %d bps) ===",
                leftId, rightId, baudrate);

    for (auto id : {leftId, rightId}) {
        uint8_t idRead = 0;
        if (!drive.getId(id, idRead)) {
            RCLCPP_ERROR(g_node->get_logger(), "id=%u did not respond -- check wiring/ID", id);
            return 1;
        }
    }

    WheelState left, right;
    RCLCPP_INFO(g_node->get_logger(), "[main] starting comm thread -- control loop below never touches the bus");
    std::thread commThread(commThreadFunc, std::ref(drive), leftId, rightId, std::ref(left), std::ref(right));

    // ===== "Control" side: reads the cache only, decides, queues commands =====
    // Stands in for what a /cmd_vel subscription callback will do later.
    for (int i = 0; i < 40 && rclcpp::ok(); ++i) {
        printWheel("LEFT ", left);
        printWheel("RIGHT", right);
        std::this_thread::sleep_for(100ms);
    }

    g_running = false;
    commThread.join();
    RCLCPP_INFO(g_node->get_logger(), "=== DONE ===");
    return 0;
}

}  // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("woodrive_dual_motor");
    const int rc = run();
    g_node.reset();  // destroy the node before shutdown() -- see run()'s comment
    rclcpp::shutdown();
    return rc;
}
