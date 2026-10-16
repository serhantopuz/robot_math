#pragma once


namespace robot_math{
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