#include "robot_math/diff_drive.hpp"

#include <gtest/gtest.h>
#include <stdexcept>

const double tolerance = 1e-12;

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