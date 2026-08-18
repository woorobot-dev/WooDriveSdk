// dev/ tool: shows the RAW bytes going over RS-485, not the parsed values
// the other examples print. Wraps ITransport with a logging decorator --
// every write() call is one outgoing frame (TX), every byte read() picks
// up gets buffered and flushed as one line (RX) right before the next TX
// (a new request implies the previous response is done).
//
// Runs the same getId()/getFault() sequence as basic_check_node, just with
// the wire traffic visible. Useful for cross-checking against the
// protocol doc's SET/GET/RSP hex examples by eye.
//
// Run:
//   ros2 run woodrive_ros2 raw_io_check_node --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=1000000

#include "WooDriveSdk.h"

#include <cstdint>
#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace {

rclcpp::Node::SharedPtr g_node;

class LoggingTransport : public ITransport
{
public:
    explicit LoggingTransport(ITransport& inner) : _inner(inner) {}

    size_t write(const uint8_t* data, size_t len) override
    {
        flushRx();  // previous response (if any) is done -- print it before the new request
        std::string line = "TX: ";
        char byteText[4];
        for (size_t i = 0; i < len; ++i) {
            std::snprintf(byteText, sizeof(byteText), "%02X ", data[i]);
            line += byteText;
        }
        RCLCPP_INFO(g_node->get_logger(), "%s", line.c_str());
        return _inner.write(data, len);
    }

    int available() override { return _inner.available(); }

    int read() override
    {
        const int b = _inner.read();
        if (b >= 0) _rxBuffer.push_back(static_cast<uint8_t>(b));
        return b;
    }

    void flush() override { _inner.flush(); }
    void clearRx() override { flushRx(); _inner.clearRx(); }

    // Call once at the very end to print any bytes read but not yet shown.
    void flushRx()
    {
        if (_rxBuffer.empty()) return;
        std::string line = "RX: ";
        char byteText[4];
        for (uint8_t b : _rxBuffer) {
            std::snprintf(byteText, sizeof(byteText), "%02X ", b);
            line += byteText;
        }
        RCLCPP_INFO(g_node->get_logger(), "%s", line.c_str());
        _rxBuffer.clear();
    }

private:
    ITransport& _inner;
    std::vector<uint8_t> _rxBuffer;
};

int run()
{
    const std::string port = g_node->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    const int baudrate = g_node->declare_parameter<int>("baudrate", 1000000);
    const uint8_t targetId = static_cast<uint8_t>(g_node->declare_parameter<int>("target_id", 1));

    PosixSerialTransport serial(port.c_str(), baudrate);
    if (!serial.isOpen()) {
        RCLCPP_ERROR(g_node->get_logger(), "serial open failed: %s", port.c_str());
        return 1;
    }
    LoggingTransport logging(serial);

    StdClock wooClock;
    WooDrive drive(logging, wooClock);
    drive.setTimeout(200);

    RCLCPP_INFO(g_node->get_logger(), "=== raw_io_check: getId + getFault, showing wire bytes ===");

    uint8_t id = 0;
    const bool idOk = drive.getId(targetId, id);
    logging.flushRx();
    RCLCPP_INFO(g_node->get_logger(), "getId : %s (id=%u)", idOk ? "OK" : "FAIL", id);

    uint8_t fault = 0;
    const bool faultOk = drive.getFault(targetId, fault);
    logging.flushRx();
    RCLCPP_INFO(g_node->get_logger(), "getFault : %s (fault=0x%02X)", faultOk ? "OK" : "FAIL", fault);

    RCLCPP_INFO(g_node->get_logger(), "=== DONE ===");
    return (idOk && faultOk) ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("woodrive_raw_io_check");
    const int rc = run();
    g_node.reset();
    rclcpp::shutdown();
    return rc;
}
