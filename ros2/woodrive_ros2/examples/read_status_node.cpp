// ROS 2 equivalent of raspberrypi/Example02_ReadStatus.cpp.
// A simple, one-shot node: connect, dump every config/status block once,
// exit. No topics/services/actions -- see src/woodrive_node.cpp for the
// full, persistent driver (which publishes ~/status continuously).
//
// Run:
//   ros2 run woodrive_ros2 read_status_node --ros-args -p port:=/dev/ttyUSB0

#include "WooDriveSdk.h"

#include <cstdint>
#include <rclcpp/rclcpp.hpp>

namespace {

rclcpp::Node::SharedPtr g_node;

void step(const char* name, bool ok)
{
    RCLCPP_INFO(g_node->get_logger(), "%s : %s", name, ok ? "OK" : "FAIL");
}

void printBoardConfig(const WooDrive::BoardConfig& s)
{
    RCLCPP_INFO(g_node->get_logger(), "Reset : %u", s.reset);
    RCLCPP_INFO(g_node->get_logger(), "ID : %u", s.id);
    RCLCPP_INFO(g_node->get_logger(), "Fault : %u", s.fault);
    RCLCPP_INFO(g_node->get_logger(), "Communication Mode : %u", s.communicationMode);
    RCLCPP_INFO(g_node->get_logger(), "BPS : %u", s.bps);
    RCLCPP_INFO(g_node->get_logger(), "Watchdog : %u", s.watchdog);
}

void printMotorConfig(const WooDrive::MotorConfig& s)
{
    RCLCPP_INFO(g_node->get_logger(), "Motor Type : %u", s.motorType);
    RCLCPP_INFO(g_node->get_logger(), "Feedback Type : %u", s.feedbackType);
    RCLCPP_INFO(g_node->get_logger(), "Startup Feedback Type : %u", s.startupFeedbackType);
    // Protocol V1.1.0: 0x13 is direction+phase (0-3), not a plain invert
    // flag -- see core/WooDriveSdk.h DirectionPhase.
    RCLCPP_INFO(g_node->get_logger(), "Direction/Phase : %u", s.directionPhase);
    RCLCPP_INFO(g_node->get_logger(), "Field Weakening Enable : %u", s.fieldWeakeningEnable);
    RCLCPP_INFO(g_node->get_logger(), "External Brake Present : %u", s.externalBrakePresent);
    RCLCPP_INFO(g_node->get_logger(), "Pole Pairs : %u", s.polePairs);
    RCLCPP_INFO(g_node->get_logger(), "Feedback Dir : %u", s.feedbackDir);
    RCLCPP_INFO(g_node->get_logger(), "Feedback Resolution : %u", s.feedbackResolution);
    RCLCPP_INFO(g_node->get_logger(), "Gear : %.6f", s.gear);
}

void printMotorFocSetting(const WooDrive::MotorFocSetting& s)
{
    RCLCPP_INFO(g_node->get_logger(), "Auto Motor Setup : %u", s.autoMotorSetup);
    RCLCPP_INFO(g_node->get_logger(), "Alignment Mode : %u", s.alignmentMode);
    RCLCPP_INFO(g_node->get_logger(), "Phase Offset : %u", s.phaseOffset);
    RCLCPP_INFO(g_node->get_logger(), "Force Angle Level : %u", s.forceAngleLevel);
    RCLCPP_INFO(g_node->get_logger(), "Hall Sensor Setting : %u", s.hallSensorSetting);
    RCLCPP_INFO(g_node->get_logger(), "Main Port Level Now : %u", s.mainPortLevelNow);
    RCLCPP_INFO(g_node->get_logger(), "Sub Port Level Now : %u", s.subPortLevelNow);
}

void printMotorParam(const WooDrive::MotorParam& s)
{
    RCLCPP_INFO(g_node->get_logger(), "Auto Parameter : %u", s.autoParameter);
    RCLCPP_INFO(g_node->get_logger(), "Rated Speed : %.2f", s.ratedSpeed);
    RCLCPP_INFO(g_node->get_logger(), "Rated Current : %.2f", s.ratedCurrent);
    RCLCPP_INFO(g_node->get_logger(), "Rated Voltage : %.2f", s.ratedVoltage);
    RCLCPP_INFO(g_node->get_logger(), "Resistance : %.3f", s.resistance);
    RCLCPP_INFO(g_node->get_logger(), "Inductance : %.6f", s.inductance);
    RCLCPP_INFO(g_node->get_logger(), "Torque Constant : %.3f", s.torqueConstant);
    RCLCPP_INFO(g_node->get_logger(), "Moment Of Inertia : %.3f", s.momentOfInertia);
}

void printMotorGain(const WooDrive::MotorGain& s)
{
    RCLCPP_INFO(g_node->get_logger(), "Gain Mode : %u", s.gainMode);
    RCLCPP_INFO(g_node->get_logger(), "Position Gain Scale : %u", s.positionGainScale);
    RCLCPP_INFO(g_node->get_logger(), "Velocity Gain Scale : %u", s.velocityGainScale);
    RCLCPP_INFO(g_node->get_logger(), "Current Gain Scale : %u", s.currentGainScale);
    RCLCPP_INFO(g_node->get_logger(), "Position P Gain : %u", s.positionPGain);
    RCLCPP_INFO(g_node->get_logger(), "Velocity P Gain : %u", s.velocityPGain);
    RCLCPP_INFO(g_node->get_logger(), "Velocity I Gain : %u", s.velocityIGain);
    RCLCPP_INFO(g_node->get_logger(), "Current P Gain : %u", s.currentPGain);
    RCLCPP_INFO(g_node->get_logger(), "Current I Gain : %u", s.currentIGain);
}

void printMotorLimit(const WooDrive::MotorLimit& s)
{
    RCLCPP_INFO(g_node->get_logger(), "Position CCW Max : %.2f", s.positionCcwMax);
    RCLCPP_INFO(g_node->get_logger(), "Position CW Max : %.2f", s.positionCwMax);
    RCLCPP_INFO(g_node->get_logger(), "Velocity CCW Max : %.2f", s.velocityCcwMax);
    RCLCPP_INFO(g_node->get_logger(), "Velocity CW Max : %.2f", s.velocityCwMax);
    RCLCPP_INFO(g_node->get_logger(), "Iq Current CCW Max : %.2f", s.iqCurrentCcwMax);
    RCLCPP_INFO(g_node->get_logger(), "Iq Current CW Max : %.2f", s.iqCurrentCwMax);
    RCLCPP_INFO(g_node->get_logger(), "Id Current Max : %.2f", s.idCurrentMax);
    RCLCPP_INFO(g_node->get_logger(), "Iq Current Limit : %.2f", s.iqCurrentLimit);
    RCLCPP_INFO(g_node->get_logger(), "Bus Voltage Max Limit : %.2f", s.busVoltageMaxLimit);
    RCLCPP_INFO(g_node->get_logger(), "Bus Voltage Min Limit : %.2f", s.busVoltageMinLimit);
    RCLCPP_INFO(g_node->get_logger(), "Temperature Max Limit : %.2f", s.temperatureMaxLimit);
}

void printMotorControl(const WooDrive::MotorControl& s)
{
    RCLCPP_INFO(g_node->get_logger(), "Accel Time : %u", s.accelTime);
    RCLCPP_INFO(g_node->get_logger(), "Decel Time : %u", s.decelTime);
    RCLCPP_INFO(g_node->get_logger(), "Motion Mode : %u", s.motionMode);
    RCLCPP_INFO(g_node->get_logger(), "Sub Target : %.2f", s.subTarget);
    RCLCPP_INFO(g_node->get_logger(), "Main Target : %.2f", s.mainTarget);
    RCLCPP_INFO(g_node->get_logger(), "Direction : %u", s.direction);
    RCLCPP_INFO(g_node->get_logger(), "Run Mode : %u", s.runMode);
    RCLCPP_INFO(g_node->get_logger(), "Motor Enable : %u", s.motorEnable);
    RCLCPP_INFO(g_node->get_logger(), "Motor Brake : %u", s.motorBrake);
    RCLCPP_INFO(g_node->get_logger(), "External Brake : %u", s.externalBrake);
}

void printMotorStatus(const WooDrive::MotorStatus& s)
{
    RCLCPP_INFO(g_node->get_logger(), "Total Time : %u", s.totalTime);
    RCLCPP_INFO(g_node->get_logger(), "Elapsed Time : %u", s.elapsedTime);
    RCLCPP_INFO(g_node->get_logger(), "Remain Time : %u", s.remainTime);
    RCLCPP_INFO(g_node->get_logger(), "Position : %.2f", s.position);
    RCLCPP_INFO(g_node->get_logger(), "Velocity : %.2f", s.velocity);
    RCLCPP_INFO(g_node->get_logger(), "Iq Current : %.3f", s.iqCurrent);
    RCLCPP_INFO(g_node->get_logger(), "Id Current : %.3f", s.idCurrent);
    RCLCPP_INFO(g_node->get_logger(), "Bus Current : %.3f", s.busCurrent);
    RCLCPP_INFO(g_node->get_logger(), "Vq Voltage : %.3f", s.vqVoltage);
    RCLCPP_INFO(g_node->get_logger(), "Vd Voltage : %.3f", s.vdVoltage);
    RCLCPP_INFO(g_node->get_logger(), "Bus Voltage : %.2f", s.busVoltage);
    RCLCPP_INFO(g_node->get_logger(), "Temperature : %.2f", s.temperature);
    RCLCPP_INFO(g_node->get_logger(), "Pulse Count : %ld", static_cast<long>(s.pulseCount));
    RCLCPP_INFO(g_node->get_logger(), "U Phase Current : %.3f", s.uPhaseCurrent);
    RCLCPP_INFO(g_node->get_logger(), "W Phase Current : %.3f", s.wPhaseCurrent);
}

// All node-owning work lives here so g_node is reset (and the underlying
// rclcpp::Node destroyed) strictly before rclcpp::shutdown() runs in main().
// A node destroyed after shutdown() tears down against an already-finalized
// rmw/DDS context and segfaults on exit.
int run()
{
    const std::string port = g_node->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    const int baudrate = g_node->declare_parameter<int>("baudrate", 9600);
    const uint8_t targetId = static_cast<uint8_t>(g_node->declare_parameter<int>("target_id", 1));

    PosixSerialTransport serial(port.c_str(), baudrate);
    if (!serial.isOpen()) {
        RCLCPP_ERROR(g_node->get_logger(), "serial open failed: %s", port.c_str());
        return 1;
    }

    StdClock wooClock;
    WooDrive drive(serial, wooClock);
    drive.setTimeout(100);

    RCLCPP_INFO(g_node->get_logger(), "=== WooDrive Example02 : Read Status ===");

    uint8_t idRead = 0;
    uint8_t fault = 0;

    bool ok = drive.getId(targetId, idRead);
    step("getId", ok);
    if (!ok) return 1;

    ok = drive.getFault(targetId, fault);
    step("getFault", ok);
    if (!ok) return 1;

    RCLCPP_INFO(g_node->get_logger(), "ID : %u", idRead);
    RCLCPP_INFO(g_node->get_logger(), "FAULT : %u", fault);

    WooDrive::BoardConfig board{};
    if ((ok = drive.getBoardConfigAll(targetId, board))) printBoardConfig(board);
    step("BoardConfigAll", ok);

    WooDrive::MotorConfig motorConfig{};
    if ((ok = drive.getMotorConfigAll(targetId, motorConfig))) printMotorConfig(motorConfig);
    step("MotorConfigAll", ok);

    WooDrive::MotorFocSetting foc{};
    if ((ok = drive.getMotorFocSettingAll(targetId, foc))) printMotorFocSetting(foc);
    step("MotorFocSettingAll", ok);

    WooDrive::MotorParam param{};
    if ((ok = drive.getMotorParamAll(targetId, param))) printMotorParam(param);
    step("MotorParamAll", ok);

    WooDrive::MotorGain gain{};
    if ((ok = drive.getMotorGainAll(targetId, gain))) printMotorGain(gain);
    step("MotorGainAll", ok);

    WooDrive::MotorLimit limit{};
    if ((ok = drive.getMotorLimitAll(targetId, limit))) printMotorLimit(limit);
    step("MotorLimitAll", ok);

    WooDrive::MotorControl control{};
    if ((ok = drive.getMotorControlAll(targetId, control))) printMotorControl(control);
    step("MotorControlAll", ok);

    WooDrive::MotorStatus status{};
    if ((ok = drive.getMotorStatusAll(targetId, status))) printMotorStatus(status);
    step("MotorStatusAll", ok);

    RCLCPP_INFO(g_node->get_logger(), "=== DONE ===");
    return 0;
}

}  // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("woodrive_example_read_status");
    const int rc = run();
    g_node.reset();  // destroy the node before shutdown() -- see run()'s comment
    rclcpp::shutdown();
    return rc;
}
