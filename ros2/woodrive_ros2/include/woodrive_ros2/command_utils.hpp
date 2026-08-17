#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>

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
// allowed through the generic ~/motion_command service -- see the protocol
// doc's motion mode table (p.134-136). All 32 defined modes across all four
// families (Voltage 0x10-0x1D, Current 0x30-0x3D, Velocity 0x70-0x7D,
// Position 0xF0-0xFD) are allowed; this is validation against the known
// mode table (rejects typos/garbage values), not a safety allowlist.
//
// Voltage-direct-drive (0x10-0x1D) is open-loop -- it bypasses the current
// loop, so the resulting current is whatever V/R and back-EMF produce
// rather than something the controller regulates. It is exposed here
// deliberately; the caller is responsible for using it with an appropriate
// sub_target/accel profile. Current/Velocity/Position families are all
// closed-loop and additionally bounded by the MOTOR LIMIT registers
// (iq_current_limit, velocity_ccw_max/cw_max, etc. -- see ~/get_gain_limit
// / ~/set_gain_limit_field) regardless of mode.
inline bool isAllowedMotionMode(uint8_t mode)
{
    static constexpr uint8_t kVoltageFamily[]  = {0x10, 0x11, 0x14, 0x15, 0x18, 0x19, 0x1C, 0x1D};
    static constexpr uint8_t kCurrentFamily[]  = {0x30, 0x31, 0x34, 0x35, 0x38, 0x39, 0x3C, 0x3D};
    static constexpr uint8_t kVelocityFamily[] = {0x70, 0x71, 0x74, 0x75, 0x78, 0x79, 0x7C, 0x7D};
    static constexpr uint8_t kPositionFamily[] = {0xF0, 0xF1, 0xF4, 0xF5, 0xF8, 0xF9, 0xFC, 0xFD};
    for (uint8_t m : kVoltageFamily)  if (m == mode) return true;
    for (uint8_t m : kCurrentFamily)  if (m == mode) return true;
    for (uint8_t m : kVelocityFamily) if (m == mode) return true;
    for (uint8_t m : kPositionFamily) if (m == mode) return true;
    return false;
}

// Authoritative list of field names ~/set_gain_limit_field accepts -- each
// one maps 1:1 to a single low-level core::WooDrive setter (setGainMode(),
// setPositionPGain(), setVelocityCcwMax(), ...). Kept here, not just in
// woodrive_node.cpp's dispatcher, so it's unit-testable without a live
// serial connection and stays the single source of truth for the .srv docs.
inline bool isKnownGainLimitField(const std::string& field)
{
    static const std::array<const char*, 20> kFields = {
        // MOTOR GAIN
        "gain_mode", "position_gain_scale", "velocity_gain_scale", "current_gain_scale",
        "position_p_gain", "velocity_p_gain", "velocity_i_gain", "current_p_gain", "current_i_gain",
        // MOTOR LIMIT
        "position_ccw_max", "position_cw_max", "velocity_ccw_max", "velocity_cw_max",
        "iq_current_ccw_max", "iq_current_cw_max", "id_current_max", "iq_current_limit",
        "bus_voltage_max_limit", "bus_voltage_min_limit", "temperature_max_limit",
    };
    for (const char* name : kFields) if (field == name) return true;
    return false;
}

}  // namespace woodrive_ros2
