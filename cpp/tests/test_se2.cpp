#include "robot_math/se2.hpp"
#include <gtest/gtest.h>
#include <cmath>

const double pi = std::acos(-1.0);
const double tolerance = 1e-12;
namespace {

void expect_pose_near(const robot_math::SE2& actual, const robot_math::SE2& expected){
    EXPECT_NEAR(actual.x(), expected.x(), tolerance);
    EXPECT_NEAR(actual.y(), expected.y(), tolerance);
    EXPECT_NEAR(robot_math::wrap_angle(actual.theta() - expected.theta()), 0.0 , tolerance);
}
} //namespace

TEST(WrapAngle, MapsIntoMinusPiToPi){
    EXPECT_NEAR(robot_math::wrap_angle(6.0), 6.0 - 2 * pi, tolerance);
    EXPECT_NEAR(robot_math::wrap_angle(370 * pi / 180), 10 * pi / 180, tolerance);
    EXPECT_NEAR(robot_math::wrap_angle(-4.0), -4.0 + 2 * pi, tolerance);
    EXPECT_NEAR(robot_math::wrap_angle(0.5), 0.5, tolerance);
}

TEST(SE2, ToMatrixWithNoRotation){
    robot_math::SE2 pose(3.0, 4.0, 0.0);
    Eigen::Matrix3d expected;
    expected << 1, 0, 3, 0, 1, 4, 0, 0, 1;
    EXPECT_TRUE(pose.to_matrix().isApprox(expected));
}

TEST(SE2, IdentityIsIdentityMatrix){
    robot_math::SE2 pose = robot_math::SE2::identity();
    EXPECT_TRUE(pose.to_matrix().isApprox(Eigen::Matrix3d::Identity()));
}

TEST(SE2, StoresWrappedTheta){
    robot_math::SE2 pose(0, 0, 6.0);
    double theta = pose.theta();
    EXPECT_NEAR(theta, 6.0 - 2 * pi, tolerance);
}

TEST(SE2, FromMatrixRoundTripWithNegativeAngle){
    robot_math::SE2 original(1.5, -2.0, -2.0);
    robot_math::SE2 recovered = robot_math::SE2::from_matrix(original.to_matrix());
    expect_pose_near(recovered, original);
}