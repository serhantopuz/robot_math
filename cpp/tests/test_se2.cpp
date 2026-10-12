#include "robot_math/se2.hpp"
#include <gtest/gtest.h>
#include <cmath>

const double pi = std::acos(-1.0);
const double tolerance = 1e-12;

TEST(WrapAngle, MapsIntoMinusPiToPi){
    EXPECT_NEAR(robot_math::wrap_angle(6.0), 6.0 - 2 * pi, tolerance);
    EXPECT_NEAR(robot_math::wrap_angle(370 * pi / 180), 10 * pi / 180, tolerance);
    EXPECT_NEAR(robot_math::wrap_angle(-4.0), -4.0 + 2 * pi, tolerance);
    EXPECT_NEAR(robot_math::wrap_angle(0.5), 0.5, tolerance);
}