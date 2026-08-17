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

TEST(CommandUtils, AllowsVelocityAndPositionFamilies)
{
    // Vel Curr Abs/Inc, Pos Vel Abs/Inc -- the modes the node already uses.
    EXPECT_TRUE(woodrive_ros2::isAllowedMotionMode(0x74));
    EXPECT_TRUE(woodrive_ros2::isAllowedMotionMode(0x75));
    EXPECT_TRUE(woodrive_ros2::isAllowedMotionMode(0xF4));
    EXPECT_TRUE(woodrive_ros2::isAllowedMotionMode(0xF5));
    // Time / Time-Left variants in both families are also allowed.
    EXPECT_TRUE(woodrive_ros2::isAllowedMotionMode(0x78));
    EXPECT_TRUE(woodrive_ros2::isAllowedMotionMode(0x7D));
    EXPECT_TRUE(woodrive_ros2::isAllowedMotionMode(0xF8));
    EXPECT_TRUE(woodrive_ros2::isAllowedMotionMode(0xFD));
}

TEST(CommandUtils, RejectsVoltageAndCurrentDirectDrive)
{
    // Voltage-direct-drive (0x10-0x1D) and Current-direct-drive (0x30-0x3D)
    // have no built-in torque/velocity limiting.
    EXPECT_FALSE(woodrive_ros2::isAllowedMotionMode(0x10));
    EXPECT_FALSE(woodrive_ros2::isAllowedMotionMode(0x1D));
    EXPECT_FALSE(woodrive_ros2::isAllowedMotionMode(0x30));
    EXPECT_FALSE(woodrive_ros2::isAllowedMotionMode(0x3D));
    EXPECT_FALSE(woodrive_ros2::isAllowedMotionMode(0x00));  // None
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
