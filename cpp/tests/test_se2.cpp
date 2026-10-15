#include "robot_math/se2.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <stdexcept>
#include <string>


const double pi = std::acos(-1.0);
const double tolerance = 1e-12;

namespace {

void expect_pose_near(const robot_math::SE2& actual, const robot_math::SE2& expected){
    EXPECT_NEAR(actual.x(), expected.x(), tolerance);
    EXPECT_NEAR(actual.y(), expected.y(), tolerance);
    EXPECT_NEAR(robot_math::wrap_angle(actual.theta() - expected.theta()), 0.0 , tolerance);
}
void expect_rejected(const Eigen::Matrix3d& bad, const std::string& phrase){
    try{
        robot_math::SE2::from_matrix(bad);
        FAIL() << "expected std::invalid_argument";
    } catch (const std::invalid_argument& e) {
        EXPECT_NE(std::string(e.what()).find(phrase), std::string::npos) << e.what();
    }
    
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

TEST(SE2, FromMatrixRejectsBadBottomRow){
    Eigen::Matrix3d bad; bad << 1, 0, 0, 0, 1, 0, 1, 1, 1;
    expect_rejected(bad, "bottom row");
}

TEST(SE2, FromMatrixRejectsAreaPreservingStretch){
    Eigen::Matrix3d bad; bad << 2, 0, 0, 0, 0.5, 0, 0, 0, 1;
    expect_rejected(bad, "stretch");
}

TEST(SE2, FromMatrixRejectsOneStretchedColumn){
    Eigen::Matrix3d bad; bad << 2, 0, 0, 0, 1, 0, 0, 0, 1;
    expect_rejected(bad, "stretch");
}

TEST(SE2, FromMatrixRejectsShear){
    Eigen::Matrix3d bad; bad << 1, 0.5, 0, 0, std::sqrt(3.0) / 2.0, 0, 0, 0, 1;
    expect_rejected(bad, "shear");
}

TEST(SE2, FromMatrixRejectsMirror){
    Eigen::Matrix3d bad; bad << 0, 1, 0, 1, 0, 0, 0, 0, 1;
    expect_rejected(bad, "mirror");
}

TEST(SE2, FromMatrixAcceptsFloatNoiseInColumns){
    robot_math::SE2 original(0.3, -1.0, 1.7);
    robot_math::SE2 recovered = robot_math::SE2::from_matrix(original.to_matrix());
    expect_pose_near(recovered, original);
}

TEST(SE2, ComposeAppliesMotionInRobotFrame){
    robot_math::SE2 expected(0, 1, pi);
    robot_math::SE2 a(0, 0, pi/2);
    robot_math::SE2 b(1, 0, pi/2);

    expect_pose_near(a * b, expected);
}

TEST(SE2, ComposeWithIdentityChangesNothing){
    robot_math::SE2 p(2, 1, pi/2);
    robot_math::SE2 identity = robot_math::SE2::identity();

    expect_pose_near(identity * p, p);
    expect_pose_near(p * identity, p);
}

TEST(SE2, ComposeMatchesMatrixMultiplication){
    robot_math::SE2 a(1.5, -2.0, 0.7);
    robot_math::SE2 b(-0.3, 4.0, -2.1);

    EXPECT_TRUE((a * b).to_matrix().isApprox(a.to_matrix()*b.to_matrix()));
}

TEST(SE2, ComposeResultIsWrapped){
    robot_math::SE2 a(0, 0, 3.0);
    robot_math::SE2 b(0, 0, 3.0);
    robot_math::SE2 via_matrices = robot_math::SE2::from_matrix(a.to_matrix()*b.to_matrix());

    EXPECT_NEAR((a*b).theta(), via_matrices.theta(), tolerance);
}

TEST(SE2, InverseRotatesTranslationBack){
    robot_math::SE2 expected(0, 2, -pi/2);
    robot_math::SE2 p(2, 0, pi/2);

    expect_pose_near(p.inverse(), expected);
}

TEST(SE2, InverseComposesToIdentity){
    robot_math::SE2 p(1.5, -2.0, 0.7);

    expect_pose_near(p*p.inverse(), robot_math::SE2::identity());
    expect_pose_near(p.inverse()*p, robot_math::SE2::identity());
}

TEST(SE2, InverseMatchesMatrixInverse){
    robot_math::SE2 p(1.5, -2.0, 0.7);

    EXPECT_TRUE(p.inverse().to_matrix().isApprox(p.to_matrix().inverse()));
}
