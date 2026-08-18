#include "amr_2wheel_robot/motor_command_utils.hpp"

#include <gtest/gtest.h>

using amr_2wheel_robot::applyInvertAndClamp;

TEST(MotorCommandUtils, PassesThroughWithinLimit)
{
    EXPECT_FLOAT_EQ(applyInvertAndClamp(20.0f, false, 100.0f), 20.0f);
    EXPECT_FLOAT_EQ(applyInvertAndClamp(-20.0f, false, 100.0f), -20.0f);
}

TEST(MotorCommandUtils, InvertsSign)
{
    EXPECT_FLOAT_EQ(applyInvertAndClamp(20.0f, true, 100.0f), -20.0f);
    EXPECT_FLOAT_EQ(applyInvertAndClamp(-20.0f, true, 100.0f), 20.0f);
}

TEST(MotorCommandUtils, ClampsToMaxRpm)
{
    EXPECT_FLOAT_EQ(applyInvertAndClamp(500.0f, false, 100.0f), 100.0f);
    EXPECT_FLOAT_EQ(applyInvertAndClamp(-500.0f, false, 100.0f), -100.0f);
}

TEST(MotorCommandUtils, InvertThenClampAppliesInCorrectOrder)
{
    // Invert first (20 -> -20), then clamp against the limit -- still well
    // within +/-50, so nothing should be clamped away.
    EXPECT_FLOAT_EQ(applyInvertAndClamp(20.0f, true, 50.0f), -20.0f);
    // Requesting +80 with invert on becomes -80, which does exceed the
    // +/-50 limit and must clamp to -50, not +50.
    EXPECT_FLOAT_EQ(applyInvertAndClamp(80.0f, true, 50.0f), -50.0f);
}

TEST(MotorCommandUtils, ZeroOrNegativeMaxRpmDisablesClamping)
{
    EXPECT_FLOAT_EQ(applyInvertAndClamp(9999.0f, false, 0.0f), 9999.0f);
    EXPECT_FLOAT_EQ(applyInvertAndClamp(9999.0f, false, -1.0f), 9999.0f);
}
