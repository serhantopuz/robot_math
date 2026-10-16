#include "robot_math/diff_drive.hpp"

#include <gtest/gtest.h>
#include <stdexcept>
#include <cmath>

const double tolerance = 1e-12;
const double pi = std::acos(-1.0);

namespace{
    void expect_pose_near(const robot_math::SE2& actual, const robot_math::SE2& expected, double tol = tolerance){
        EXPECT_NEAR(actual.x(), expected.x(), tol);
        EXPECT_NEAR(actual.y(), expected.y(), tol);
        EXPECT_NEAR(robot_math::wrap_angle(actual.theta() - expected.theta()), 0.0 , tol);
    }
}

TEST(DiffDrive, BodyVelocityEqualWheelsDrivesStraight){
    robot_math::DiffDrive drive(0.3);
    robot_math::BodyVelocity result = drive.body_velocity(0.5, 0.5);

    EXPECT_NEAR(result.v, 0.5, tolerance);
    EXPECT_NEAR(result.omega, 0.0, tolerance);
}

TEST(DiffDrive, BodyVelocityOppositeWheelsSpinsInPlace){
    robot_math::DiffDrive drive(0.3);
    robot_math::BodyVelocity result = drive.body_velocity(-0.5, 0.5);

    EXPECT_NEAR(result.v, 0.0, tolerance);
    EXPECT_NEAR(result.omega, 1.0 / 0.3, tolerance);
}

TEST(DiffDrive, BodyVelocityCurvesLeft){
    robot_math::DiffDrive drive(0.3);
    robot_math::BodyVelocity result = drive.body_velocity(0.2, 0.4);

    EXPECT_NEAR(result.v, 0.3, tolerance);
    EXPECT_NEAR(result.omega, 2.0 / 3, tolerance);
}

TEST(DiffDrive, WheelSpeedsTurningLeftSpeedsUpRightWheel){
    robot_math::DiffDrive drive(0.3);
    robot_math::WheelSpeeds result = drive.wheel_speeds(0.2, 0.5);

    EXPECT_NEAR(result.left, 0.125, tolerance);
    EXPECT_NEAR(result.right, 0.275, tolerance);
}

TEST(DiffDrive, BodyVelocityUndoesWheelSpeeds){
    robot_math::DiffDrive drive(0.287);
    robot_math::WheelSpeeds wheels = drive.wheel_speeds(0.43, -1.7);
    robot_math::BodyVelocity result = drive.body_velocity(wheels.left, wheels.right);

    EXPECT_NEAR(result.v, 0.43, tolerance);
    EXPECT_NEAR(result.omega, -1.7, tolerance);
}

TEST(DiffDrive, RejectsNonPositiveWheelBase){
    EXPECT_THROW(robot_math::DiffDrive(0.0), std::invalid_argument);
    EXPECT_THROW(robot_math::DiffDrive(-0.3), std::invalid_argument);
}

TEST(DiffDrive, MotionStepZeroOmegaDrivesStraight){
    robot_math::SE2 actual = robot_math::motion_step(0.5, 0.0, 2.0);

    expect_pose_near(actual, robot_math::SE2(1, 0, 0));
}

TEST(DiffDrive, MotionStepTurningLeftFollowsQuarterCircle){
    robot_math::SE2 actual = robot_math::motion_step(1.0, 1.0, pi/2);

    expect_pose_near(actual, robot_math::SE2(1, 1, pi/2));
}

TEST(DiffDrive, MotionStepTurningRightFollowsQuarterCircle){
    robot_math::SE2 actual = robot_math::motion_step(1.0, -1.0, pi/2);

    expect_pose_near(actual, robot_math::SE2(1, -1, -pi/2));
}

TEST(DiffDrive, MotionStepTwoHalfStepsEqualOneFullStep){
    const double v = 0.43;
    const double omega = -1.7;
    const double dt = 0.37;

    robot_math::SE2 full = robot_math::motion_step(v, omega, dt);
    robot_math::SE2 half = robot_math::motion_step(v, omega, dt / 2);
    expect_pose_near(half * half, full);
}

TEST(DiffDrive, MotionStepBranchesAgreeAtTolerance){
    robot_math::SE2 just_below = robot_math::motion_step(1.0, 0.999*robot_math::kStraightLineTolerance, 0.02);
    robot_math::SE2 just_above = robot_math::motion_step(1.0, 1.001*robot_math::kStraightLineTolerance, 0.02);

    // The branches really differ by about v * dt * dtheta / 2 = 2.2e-10 here: the
    // straight-line step drops the arc's tiny sideways drift. That is below any
    // physical meaning but above 1e-12, so this test needs a looser tolerance.
    // 1e-9 still catches real bugs, e.g. a straight branch that drops omega * dt
    // (a 2e-8 heading gap).
    expect_pose_near(just_below, just_above, 1e-9); 
}

TEST(DiffDrive, MotionStepRejectsNegativeDt){
    EXPECT_THROW(robot_math::motion_step(1.0, 0.5, -0.1), std::invalid_argument);
}
