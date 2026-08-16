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
