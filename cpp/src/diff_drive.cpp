#include "robot_math/diff_drive.hpp"

#include <stdexcept>
#include <string>
#include <cmath>

namespace robot_math{
DiffDrive::DiffDrive(double wheel_base) : wheel_base_(wheel_base){
    if(wheel_base_ <= 0){
        throw std::invalid_argument("Expected positive wheel base, got " + std::to_string(wheel_base_));
    }
}

BodyVelocity DiffDrive::body_velocity(double v_left, double v_right) const{
    const double v = (v_left + v_right) / 2;
    const double omega = (v_right - v_left) / wheel_base_;

    return {v, omega};
}

WheelSpeeds DiffDrive::wheel_speeds(double v, double omega) const{
    const double v_left = v - omega * wheel_base_ / 2;
    const double v_right = v + omega * wheel_base_ / 2;

    return {v_left, v_right};
}

SE2 motion_step(double v, double omega, double dt){
    if (dt < 0){
        throw std::invalid_argument("Expected non-negative dt, got " + std::to_string(dt));
    }
    if (std::abs(omega) < kStraightLineTolerance){
        return SE2 (v * dt, 0.0, omega * dt);
    }
    const double dtheta = omega * dt;
    const double radius = v / omega;

    return SE2(radius*std::sin(dtheta), radius*(1.0-std::cos(dtheta)), dtheta);
}

std::vector<SE2> DiffDrive::dead_reckon(const SE2& start, const std::vector<WheelReading>& readings) const{
    std::vector<SE2> path;
    path.reserve(readings.size() + 1);
    path.push_back(start);

    for (const WheelReading& r: readings){
        const BodyVelocity b_velocity = DiffDrive::body_velocity(r.v_left, r.v_right);
        path.push_back(path.back() * motion_step(b_velocity.v, b_velocity.omega, r.dt));
    }

    return path;
}
} // namespace robot_math

