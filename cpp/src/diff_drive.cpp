#include "robot_math/diff_drive.hpp"

#include <stdexcept>
#include <string>

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
} // namespace robot_math
