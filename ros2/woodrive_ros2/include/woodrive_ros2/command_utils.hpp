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

}  // namespace woodrive_ros2
