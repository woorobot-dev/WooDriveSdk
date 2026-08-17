#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace woodrive_ros2 {

struct SpeedCommand
{
    float rpm;
    uint8_t direction;
};

inline SpeedCommand makeSpeedCommand(float signedRpm, float maxRpm)
{
    if (!std::isfinite(signedRpm) || maxRpm <= 0.0f) return {0.0f, 0};
    const float rpm = std::min(std::abs(signedRpm), maxRpm);
    if (rpm < 0.001f) return {0.0f, 0};
    return {rpm, static_cast<uint8_t>(signedRpm > 0.0f ? 1 : 2)};
}

// Motion modes (protocol address 0x94, sent as part of setMotorMotionAll())
// allowed through the generic ~/motion_command service. Velocity family
// (0x70-0x7D) and Position family (0xF0-0xFD) are torque/velocity-limited
// by construction via their sub_target field -- see the protocol doc's
// motion mode table (p.134-136). Voltage-direct-drive (0x10-0x1D) and
// Current-direct-drive (0x30-0x3D) modes have no such limiting and are
// deliberately excluded from this generic interface.
inline bool isAllowedMotionMode(uint8_t mode)
{
    static constexpr uint8_t kVelocityFamily[] = {0x70, 0x71, 0x74, 0x75, 0x78, 0x79, 0x7C, 0x7D};
    static constexpr uint8_t kPositionFamily[] = {0xF0, 0xF1, 0xF4, 0xF5, 0xF8, 0xF9, 0xFC, 0xFD};
    for (uint8_t m : kVelocityFamily) if (m == mode) return true;
    for (uint8_t m : kPositionFamily) if (m == mode) return true;
    return false;
}

}  // namespace woodrive_ros2
