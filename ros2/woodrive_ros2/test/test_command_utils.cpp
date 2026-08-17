#include "woodrive_ros2/command_utils.hpp"

#include <gtest/gtest.h>
#include <limits>

TEST(CommandUtils, StopsAtZero)
{
    const auto command = woodrive_ros2::makeSpeedCommand(0.0f, 100.0f);
    EXPECT_FLOAT_EQ(command.rpm, 0.0f);
    EXPECT_EQ(command.direction, 0);
}

TEST(CommandUtils, MapsSignedDirection)
{
    EXPECT_EQ(woodrive_ros2::makeSpeedCommand(20.0f, 100.0f).direction, 1);
    EXPECT_EQ(woodrive_ros2::makeSpeedCommand(-20.0f, 100.0f).direction, 2);
}

TEST(CommandUtils, ClampsAndRejectsInvalidInput)
{
    EXPECT_FLOAT_EQ(woodrive_ros2::makeSpeedCommand(200.0f, 100.0f).rpm, 100.0f);
    EXPECT_EQ(woodrive_ros2::makeSpeedCommand(
                  std::numeric_limits<float>::quiet_NaN(), 100.0f).direction, 0);
}

TEST(CommandUtils, AllowsAllFourModeFamilies)
{
    // Voltage (0x10-0x1D)
    EXPECT_TRUE(woodrive_ros2::isAllowedMotionMode(0x10));  // Volt Target Abs
    EXPECT_TRUE(woodrive_ros2::isAllowedMotionMode(0x11));  // Volt Target Inc
    EXPECT_TRUE(woodrive_ros2::isAllowedMotionMode(0x14));  // Volt Snap Abs
    EXPECT_TRUE(woodrive_ros2::isAllowedMotionMode(0x1D));  // Volt Time Left Inc
    // Current (0x30-0x3D)
    EXPECT_TRUE(woodrive_ros2::isAllowedMotionMode(0x30));  // Curr Target Abs
    EXPECT_TRUE(woodrive_ros2::isAllowedMotionMode(0x34));  // Curr Volt Abs
    EXPECT_TRUE(woodrive_ros2::isAllowedMotionMode(0x3D));  // Curr Time Left Inc
    // Velocity (0x70-0x7D) -- the mode the node already uses by default.
    EXPECT_TRUE(woodrive_ros2::isAllowedMotionMode(0x74));  // Vel Curr Abs
    EXPECT_TRUE(woodrive_ros2::isAllowedMotionMode(0x7D));  // Vel Time Left Inc
    // Position (0xF0-0xFD) -- the modes move_absolute/move_relative use.
    EXPECT_TRUE(woodrive_ros2::isAllowedMotionMode(0xF4));  // Pos Vel Abs
    EXPECT_TRUE(woodrive_ros2::isAllowedMotionMode(0xFD));  // Pos Time Left Inc
}

TEST(CommandUtils, RejectsUndefinedModeValues)
{
    // Not a real mode byte from the protocol's motion mode table.
    EXPECT_FALSE(woodrive_ros2::isAllowedMotionMode(0x00));  // None
    EXPECT_FALSE(woodrive_ros2::isAllowedMotionMode(0x99));  // garbage
    EXPECT_FALSE(woodrive_ros2::isAllowedMotionMode(0xFF));  // garbage
}

TEST(CommandUtils, KnowsAllGainAndLimitFields)
{
    EXPECT_TRUE(woodrive_ros2::isKnownGainLimitField("gain_mode"));
    EXPECT_TRUE(woodrive_ros2::isKnownGainLimitField("position_p_gain"));
    EXPECT_TRUE(woodrive_ros2::isKnownGainLimitField("current_i_gain"));
    EXPECT_TRUE(woodrive_ros2::isKnownGainLimitField("position_ccw_max"));
    EXPECT_TRUE(woodrive_ros2::isKnownGainLimitField("temperature_max_limit"));
}

TEST(CommandUtils, RejectsUnknownGainLimitField)
{
    EXPECT_FALSE(woodrive_ros2::isKnownGainLimitField(""));
    EXPECT_FALSE(woodrive_ros2::isKnownGainLimitField("motion_mode"));      // that's a motion_command field, not gain/limit
    EXPECT_FALSE(woodrive_ros2::isKnownGainLimitField("Position_P_Gain"));  // case-sensitive
}
