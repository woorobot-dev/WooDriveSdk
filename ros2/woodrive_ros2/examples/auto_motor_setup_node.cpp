// ROS 2 equivalent of raspberrypi/Example03_AutoMotorSetup.cpp.
// A simple, one-shot node: run the controller's auto motor setup procedure,
// poll for completion, print the result, exit. No topics/services/actions
// -- see src/woodrive_node.cpp's ~/auto_setup action for the persistent,
// cancelable version.
//
// Run:
//   ros2 run woodrive_ros2 auto_motor_setup_node --ros-args -p port:=/dev/ttyUSB0

#include "WooDriveSdk.h"

#include <chrono>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <thread>

using namespace std::chrono_literals;

namespace {

// Auto setup command value (see core/WooDriveSdk.h Address::AUTO_MOTOR_SETUP).
constexpr uint8_t kAutoSetupCmd = 0;
constexpr auto kAutoSetupTimeout = 240s;
constexpr auto kRetryInterval = 5s;

bool waitAutoSetupComplete(const rclcpp::Node::SharedPtr& node, WooDrive& drive, uint8_t targetId,
                            uint8_t& autoSetupState)
{
    const auto deadline = std::chrono::steady_clock::now() + kAutoSetupTimeout;

    RCLCPP_INFO(node->get_logger(), "Motor setup is running...");
    while (std::chrono::steady_clock::now() < deadline) {
        if (drive.getAutoMotorSetup(targetId, autoSetupState)) {
            RCLCPP_INFO(node->get_logger(), "Controller response restored.");
            return true;
        }
        RCLCPP_INFO(node->get_logger(), "Setup running, controller busy... wait 5 sec");
        std::this_thread::sleep_for(kRetryInterval);
    }
    return false;
}

// All node-owning work lives here so the rclcpp::Node::SharedPtr is
// destroyed when this function returns -- strictly before rclcpp::shutdown()
// runs in main(). A node destroyed after shutdown() tears down against an
// already-finalized rmw/DDS context and segfaults on exit.
int run(const rclcpp::Node::SharedPtr& node)
{
    const std::string port = node->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    const int baudrate = node->declare_parameter<int>("baudrate", 9600);
    const uint8_t targetId = static_cast<uint8_t>(node->declare_parameter<int>("target_id", 1));
    const uint8_t polePairs = static_cast<uint8_t>(node->declare_parameter<int>("pole_pairs", 4));

    PosixSerialTransport serial(port.c_str(), baudrate);
    if (!serial.isOpen()) {
        RCLCPP_ERROR(node->get_logger(), "serial open failed: %s", port.c_str());
        return 1;
    }

    StdClock wooClock;
    WooDrive drive(serial, wooClock);
    drive.setTimeout(100);

    RCLCPP_INFO(node->get_logger(), "=== WooDrive Example03 : Auto Motor Setup ===");

    uint8_t idRead = 0;
    uint8_t faultBefore = 0;
    uint8_t autoSetupState = 0xFF;

    bool ok = drive.getId(targetId, idRead);
    RCLCPP_INFO(node->get_logger(), "getId : %s", ok ? "OK" : "FAIL");
    if (!ok) return 0;

    ok = drive.getFault(targetId, faultBefore);
    RCLCPP_INFO(node->get_logger(), "getFault(before) : %s", ok ? "OK" : "FAIL");
    if (!ok) return 0;

    drive.setPolePairs(targetId, polePairs);

    RCLCPP_INFO(node->get_logger(), "ID : %u", idRead);
    RCLCPP_INFO(node->get_logger(), "Fault Before : 0x%02X", faultBefore);

    RCLCPP_INFO(node->get_logger(), "=== START AUTO MOTOR SETUP ===");
    ok = drive.setAutoMotorSetup(targetId, kAutoSetupCmd);
    RCLCPP_INFO(node->get_logger(), "setAutoMotorSetup : %s", ok ? "OK" : "FAIL");
    if (!ok) return 0;

    ok = waitAutoSetupComplete(node, drive, targetId, autoSetupState);
    if (!ok) {
        RCLCPP_ERROR(node->get_logger(), "AUTO SETUP TIMEOUT: controller did not respond within %ld sec.",
                     static_cast<long>(kAutoSetupTimeout.count()));
        return 0;
    }
    RCLCPP_INFO(node->get_logger(), "Auto Setup State : %u", autoSetupState);

    uint8_t faultAfter = 0;
    if ((ok = drive.getFault(targetId, faultAfter))) {
        RCLCPP_INFO(node->get_logger(), "Fault After : 0x%02X", faultAfter);
    }
    RCLCPP_INFO(node->get_logger(), "getFault(after) : %s", ok ? "OK" : "FAIL");

    RCLCPP_INFO(node->get_logger(), "=== DONE ===");
    return 0;
}

}  // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    int rc;
    {
        auto node = rclcpp::Node::make_shared("woodrive_example_auto_motor_setup");
        rc = run(node);
    }  // node destroyed here, before shutdown()
    rclcpp::shutdown();
    return rc;
}
