#pragma once

// Pure, hardware-independent helpers used by dual_motor_node.cpp. Kept
// separate (mirrors woodrive_ros2's command_utils.hpp) so the logic is
// unit-testable without a live serial connection.

namespace amr_2wheel_robot {

// Applies mechanical direction inversion (a wheel mounted mirrored on one
// side of the chassis) and clamps to +/-maxRpm. maxRpm <= 0 disables
// clamping (treated as "no limit configured").
//
// Invert is applied BEFORE clamping so max_rpm always means "the fastest
// this wheel is allowed to physically spin", regardless of which sign the
// caller's raw request used.
inline float applyInvertAndClamp(float requestedRpm, bool invert, float maxRpm)
{
    float rpm = invert ? -requestedRpm : requestedRpm;
    if (maxRpm > 0.0f) {
        if (rpm > maxRpm) rpm = maxRpm;
        if (rpm < -maxRpm) rpm = -maxRpm;
    }
    return rpm;
}

}  // namespace amr_2wheel_robot
