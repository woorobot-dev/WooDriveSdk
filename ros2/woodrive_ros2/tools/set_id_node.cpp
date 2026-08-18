// dev/ tool: check and optionally change a WooDrive controller's protocol
// ID. Needed before dual_motor_node's right-wheel path can be tested for
// real -- a freshly-added second controller ships at the same default ID
// as the first, so it needs to be moved to a distinct ID (2, by this
// project's convention) before both can share one RS-485 bus.
//
// SAFETY: do this with ONLY the target controller on the bus. Two
// controllers sharing the same ID at once can both try to answer the same
// request and corrupt each other's responses -- disconnect the other unit
// first, set this one's ID, then reconnect both.
//
// Run (check only, no new_id given):
//   ros2 run woodrive_ros2 set_id_node --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=1000000
// Run (check current ID, then set to 2):
//   ros2 run woodrive_ros2 set_id_node --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=1000000 -p current_id:=1 -p new_id:=2

#include "WooDriveSdk.h"

#include <cstdint>
#include <rclcpp/rclcpp.hpp>

namespace {

rclcpp::Node::SharedPtr g_node;

int run()
{
    const std::string port = g_node->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    const int baudrate = g_node->declare_parameter<int>("baudrate", 1000000);
    const uint8_t currentId = static_cast<uint8_t>(g_node->declare_parameter<int>("current_id", 1));
    // new_id <= 0 means "check only, don't change anything" -- protocol IDs
    // are always >= 1, so 0 is a safe sentinel for "not provided".
    const int newIdParam = g_node->declare_parameter<int>("new_id", 0);

    PosixSerialTransport serial(port.c_str(), baudrate);
    if (!serial.isOpen()) {
        RCLCPP_ERROR(g_node->get_logger(), "serial open failed: %s", port.c_str());
        return 1;
    }
    StdClock wooClock;
    WooDrive drive(serial, wooClock);
    drive.setTimeout(200);

    uint8_t idRead = 0;
    if (!drive.getId(currentId, idRead)) {
        RCLCPP_ERROR(g_node->get_logger(),
                     "no response at id=%u -- wrong current_id, or two controllers "
                     "sharing an ID are colliding on the bus", currentId);
        return 1;
    }
    RCLCPP_INFO(g_node->get_logger(), "Controller responding at id=%u (readback confirms id=%u)",
                currentId, idRead);

    if (newIdParam <= 0) {
        RCLCPP_INFO(g_node->get_logger(), "new_id not given -- check-only, nothing changed.");
        return 0;
    }

    const uint8_t newId = static_cast<uint8_t>(newIdParam);
    if (!drive.setId(currentId, newId)) {
        RCLCPP_ERROR(g_node->get_logger(), "setId(%u -> %u) send failed", currentId, newId);
        return 1;
    }
    RCLCPP_INFO(g_node->get_logger(), "setId(%u -> %u) sent. Verifying at the new id...",
                currentId, newId);

    uint8_t verifyRead = 0;
    if (drive.getId(newId, verifyRead) && verifyRead == newId) {
        RCLCPP_INFO(g_node->get_logger(), "OK -- controller now responds at id=%u", newId);
        return 0;
    }
    RCLCPP_ERROR(g_node->get_logger(),
                 "id change did not verify -- controller may still be at id=%u, "
                 "or needs a moment/power-cycle to apply", currentId);
    return 1;
}

}  // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("woodrive_set_id");
    const int rc = run();
    g_node.reset();
    rclcpp::shutdown();
    return rc;
}
