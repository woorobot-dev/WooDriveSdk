// Growing playground node -- starting point for the 2-wheel diff-drive
// robot. Not part of the raspberrypi/arduino Example01-05 parity set (see
// examples/README.md for those); this one is expected to keep growing:
// dual-motor read/control now, /cmd_vel -> per-wheel kinematics and /odom
// later, eventually becoming (or feeding) a dedicated woodrive_diffdrive
// package.
//
// Right now: connects to two WooDrive controllers on the same RS-485 bus
// (left_id, right_id -- different protocol IDs, same physical wire) and
// prints both motors' status side by side. Defaults to 1,000,000 bps since
// the whole point of splitting into two IDs is the tight comm/control loop
// discussed for the diff-drive work, which needs the faster bus.
//
// Run:
//   ros2 run woodrive_ros2 dual_motor_node --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=1000000 -p left_id:=1 -p right_id:=2

#include "WooDriveSdk.h"

#include <chrono>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <thread>

using namespace std::chrono_literals;

namespace {

rclcpp::Node::SharedPtr g_node;

void printWheel(const char* label, uint8_t id, bool faultOk, uint8_t fault,
                bool statusOk, const WooDrive::MotorStatus& s)
{
    if (!faultOk || !statusOk) {
        RCLCPP_ERROR(g_node->get_logger(), "[%s id=%u] read FAIL", label, id);
        return;
    }
    RCLCPP_INFO(g_node->get_logger(),
                "[%s id=%u] fault=0x%02X pos=%.2fdeg vel=%.2frpm iq=%.3fA temp=%.1fC",
                label, id, fault, s.position, s.velocity, s.iqCurrent, s.temperature);
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

    // Quick reachability check for both IDs before looping.
    for (auto id : {leftId, rightId}) {
        uint8_t idRead = 0;
        if (!drive.getId(id, idRead)) {
            RCLCPP_ERROR(g_node->get_logger(), "id=%u did not respond -- check wiring/ID", id);
            return 1;
        }
    }

    for (int i = 0; i < 20 && rclcpp::ok(); ++i) {
        uint8_t leftFault = 0, rightFault = 0;
        WooDrive::MotorStatus leftStatus{}, rightStatus{};

        const bool leftFaultOk = drive.getFault(leftId, leftFault);
        const bool leftStatusOk = drive.getMotorStatusAll(leftId, leftStatus);
        printWheel("LEFT ", leftId, leftFaultOk, leftFault, leftStatusOk, leftStatus);

        const bool rightFaultOk = drive.getFault(rightId, rightFault);
        const bool rightStatusOk = drive.getMotorStatusAll(rightId, rightStatus);
        printWheel("RIGHT", rightId, rightFaultOk, rightFault, rightStatusOk, rightStatus);

        std::this_thread::sleep_for(200ms);
    }

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
