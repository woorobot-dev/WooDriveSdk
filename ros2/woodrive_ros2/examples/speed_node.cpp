// ROS 2 equivalent of raspberrypi/Example04_Speed.cpp.
// A simple, one-shot node: run the motor at +speed, stop, -speed, stop,
// then demonstrate DIR_ZERO (velocity mode: "decelerate to zero speed"),
// exit. No topics/services/actions -- see src/woodrive_node.cpp's
// ~/target_rpm topic for continuous velocity control.
//
// Run:
//   ros2 run woodrive_ros2 speed_node --ros-args -p port:=/dev/ttyUSB0

#include "WooDriveSdk.h"

#include <chrono>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <thread>

using namespace std::chrono_literals;

namespace {

constexpr uint16_t kAccelMs = 1000;
constexpr uint16_t kDecelMs = 1000;
// 116 (0x74) = "Vel Curr Abs": absolute velocity target, current-limited by
// kSubTarget. See core/WooDriveSdk.h / the protocol doc's motion mode table.
constexpr uint8_t kMotionMode = 116;
constexpr float kSubTarget = 10.0f;    // max current (A)
constexpr float kMainTargetRpm = 100.0f;
constexpr auto kRunMonitorDuration = 5s;

// Protocol V1.1.0 address 0x9B (motor direction): 0 = DIR_ZERO (drive the
// motion-mode's main target to zero -- for a velocity-based mode like this
// one, that means "decelerate to a stop"), 1 = CCW/+, 2 = CW/-.
constexpr uint8_t kDirZero = 0;
constexpr uint8_t kDirPositive = 1;
constexpr uint8_t kDirNegative = 2;

rclcpp::Node::SharedPtr g_node;
WooDrive* g_drive = nullptr;
uint8_t g_targetId = 1;

void step(const char* name, bool ok)
{
    RCLCPP_INFO(g_node->get_logger(), "%s : %s", name, ok ? "OK" : "FAIL");
}

bool setRunReady()
{
    bool ok = g_drive->setMotorBrake(g_targetId, 1);
    step("setMotorBrake(1)", ok);
    if (!ok) return false;
    std::this_thread::sleep_for(100ms);

    ok = g_drive->setMotorEnable(g_targetId, 1);
    step("setMotorEnable(1)", ok);
    if (!ok) return false;
    std::this_thread::sleep_for(100ms);
    return true;
}

bool stopCoast()
{
    bool ok = g_drive->setMotorEnable(g_targetId, 0);
    step("setMotorEnable(0)", ok);
    std::this_thread::sleep_for(100ms);

    const bool brakeOk = g_drive->setMotorBrake(g_targetId, 1);
    step("setMotorBrake(1)", brakeOk);
    std::this_thread::sleep_for(100ms);
    return ok && brakeOk;
}

bool monitorStatus(std::chrono::steady_clock::duration duration)
{
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
        WooDrive::MotorStatus status{};
        uint8_t fault = 0;
        const bool ok1 = g_drive->getFault(g_targetId, fault);
        const bool ok2 = g_drive->getMotorStatusAll(g_targetId, status);
        if (ok1 && ok2) {
            RCLCPP_INFO(g_node->get_logger(), "F:%u | P:%.2f | V:%.2f | Iq:%.3f | BusV:%.2f | Temp:%.2f",
                        fault, status.position, status.velocity, status.iqCurrent, status.busVoltage,
                        status.temperature);
            if (fault != 0) {
                RCLCPP_ERROR(g_node->get_logger(), "Fault detected during motor run.");
                return false;
            }
        } else {
            RCLCPP_WARN(g_node->get_logger(), "Status read fail");
        }
        std::this_thread::sleep_for(200ms);
    }
    return true;
}

bool runSpeed(float speedTarget, uint8_t direction)
{
    const bool ok = g_drive->setMotorMotionAll(g_targetId, kAccelMs, kDecelMs, kMotionMode, kSubTarget,
                                                speedTarget, direction);
    step(direction == kDirZero ? "setMotorMotionAll(zero)"
                                : direction == kDirPositive ? "setMotorMotionAll(+dir)"
                                                             : "setMotorMotionAll(-dir)",
         ok);
    if (!ok) return false;
    return monitorStatus(kRunMonitorDuration);
}

bool checkFaultZero()
{
    uint8_t fault = 0;
    const bool ok = g_drive->getFault(g_targetId, fault);
    step("getFault", ok);
    if (!ok) return false;
    RCLCPP_INFO(g_node->get_logger(), "Fault : 0x%02X", fault);
    return fault == 0x00;
}

// All node-owning work lives here so g_node is reset (and the underlying
// rclcpp::Node destroyed) strictly before rclcpp::shutdown() runs in main().
// A node destroyed after shutdown() tears down against an already-finalized
// rmw/DDS context and segfaults on exit.
int run()
{
    const std::string port = g_node->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    const int baudrate = g_node->declare_parameter<int>("baudrate", 9600);
    g_targetId = static_cast<uint8_t>(g_node->declare_parameter<int>("target_id", 1));

    PosixSerialTransport serial(port.c_str(), baudrate);
    if (!serial.isOpen()) {
        RCLCPP_ERROR(g_node->get_logger(), "serial open failed: %s", port.c_str());
        return 1;
    }

    StdClock wooClock;
    WooDrive drive(serial, wooClock);
    g_drive = &drive;
    drive.setTimeout(100);

    RCLCPP_INFO(g_node->get_logger(), "=== WooDrive Example04 : Speed ===");

    uint8_t idRead = 0;
    bool ok = drive.getId(g_targetId, idRead);
    step("getId", ok);
    if (!ok) return 0;
    RCLCPP_INFO(g_node->get_logger(), "ID : %u", idRead);

    if (!checkFaultZero()) {
        RCLCPP_ERROR(g_node->get_logger(), "WooDrive fault detected before run.");
        return 0;
    }

    if (!setRunReady()) {
        RCLCPP_ERROR(g_node->get_logger(), "Run ready setup failed.");
        return 0;
    }

    RCLCPP_INFO(g_node->get_logger(), "=== RUN 1 : + SPEED (%.2f rpm) ===", kMainTargetRpm);
    if (!runSpeed(kMainTargetRpm, kDirPositive)) { stopCoast(); return 0; }
    if (!stopCoast()) return 0;
    std::this_thread::sleep_for(1s);

    RCLCPP_INFO(g_node->get_logger(), "=== RUN 2 : - SPEED (%.2f rpm) ===", kMainTargetRpm);
    if (!setRunReady()) return 0;
    if (!runSpeed(kMainTargetRpm, kDirNegative)) { stopCoast(); return 0; }
    if (!stopCoast()) return 0;

    RCLCPP_INFO(g_node->get_logger(), "=== RUN 3 : ZERO (decelerate to a stop) ===");
    if (!setRunReady()) return 0;
    if (!runSpeed(kMainTargetRpm, kDirZero)) { stopCoast(); return 0; }
    if (!stopCoast()) return 0;

    RCLCPP_INFO(g_node->get_logger(), "=== DONE ===");
    return 0;
}

}  // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("woodrive_example_speed");
    const int rc = run();
    g_node.reset();  // destroy the node before shutdown() -- see run()'s comment
    rclcpp::shutdown();
    return rc;
}
