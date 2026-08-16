// ROS 2 equivalent of raspberrypi/Example01_BasicCheck.cpp.
// A simple, one-shot node: connect, read ID and fault, print the result,
// exit. No topics/services/actions -- see src/woodrive_node.cpp for the
// full, persistent driver.
//
// Run:
//   ros2 run woodrive_ros2 basic_check_node --ros-args -p port:=/dev/ttyUSB0

#include "WooDriveSdk.h"

#include <cstdint>
#include <rclcpp/rclcpp.hpp>

namespace {

// All node-owning work lives here so the rclcpp::Node::SharedPtr is
// destroyed when this function returns -- strictly before rclcpp::shutdown()
// runs in main(). Destroying a node after shutdown() (e.g. via a node that
// outlives main(), or a shutdown() call before the node's own destructor)
// tears down against an already-finalized rmw/DDS context and segfaults.
int run(const rclcpp::Node::SharedPtr& node)
{
    const std::string port = node->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    const int baudrate = node->declare_parameter<int>("baudrate", 9600);
    const uint8_t targetId = static_cast<uint8_t>(node->declare_parameter<int>("target_id", 1));

    PosixSerialTransport serial(port.c_str(), baudrate);
    if (!serial.isOpen()) {
        RCLCPP_ERROR(node->get_logger(), "serial open failed: %s", port.c_str());
        return 1;
    }

    StdClock wooClock;
    WooDrive drive(serial, wooClock);
    drive.setTimeout(100);

    RCLCPP_INFO(node->get_logger(), "=== WooDrive Example01 : Basic Check ===");

    uint8_t id = 0;
    uint8_t fault = 0;

    if (drive.getId(targetId, id)) {
        RCLCPP_INFO(node->get_logger(), "ID OK : %u", static_cast<unsigned>(id));
    } else {
        RCLCPP_ERROR(node->get_logger(), "ID FAIL");
        return 0;
    }

    if (drive.getFault(targetId, fault)) {
        RCLCPP_INFO(node->get_logger(), "FAULT : %u", static_cast<unsigned>(fault));
    } else {
        RCLCPP_ERROR(node->get_logger(), "FAULT READ FAIL");
        return 0;
    }

    RCLCPP_INFO(node->get_logger(), "=== RESULT : PASS ===");
    return 0;
}

}  // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    int rc;
    {
        auto node = rclcpp::Node::make_shared("woodrive_example_basic_check");
        rc = run(node);
    }  // node destroyed here, before shutdown()
    rclcpp::shutdown();
    return rc;
}
