#pragma once
#include "robot_math/se2.hpp"

namespace robot_math{
inline constexpr double kStraightLineTolerance = 1e-6;  // rad/s

/// Pose change after driving at constant v and omega for dt seconds, in the robot's
/// frame at the start of the step: new_pose = pose * motion_step(v, omega, dt).
/// Below kStraightLineTolerance the straight-line step replaces the arc.
/// Throws std::invalid_argument if dt < 0.
SE2 motion_step(double v, double omega, double dt);

/// Forward speed (m/s) and turn rate (rad/s, counter-clockwise positive).
struct BodyVelocity{
    double v;
    double omega;
};
/// Wheel speeds in m/s.
struct WheelSpeeds{
    double left;
    double right;
};

///A two-wheeled differential-drive robot. Wheel base in meters.
class DiffDrive{
public: 
    ///Throws std::invalid_argument if wheel_base <= 0.
    explicit DiffDrive(double wheel_base);
    double wheel_base() const {return wheel_base_;}

    ///Wheel speeds -> body motion (odometry).
    BodyVelocity body_velocity(double v_left, double v_right) const;
    ///Body motion -> wheel speeds (control).
    WheelSpeeds wheel_speeds(double v, double omega) const;
private:
    double wheel_base_;
};
} // namespace robot_math