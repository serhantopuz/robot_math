#include "robot_math/se2.hpp"

#include <cmath>

namespace robot_math{

double wrap_angle(double theta){
    return std::atan2(std::sin(theta), std::cos(theta));
}

SE2::SE2(double x, double y, double theta) : x_(x), y_(y), theta_(wrap_angle(theta)){}
SE2 SE2::identity(){
    return SE2(0, 0, 0);
}
Eigen::Matrix3d SE2::to_matrix() const {
    const double cos_theta = std::cos(theta_);
    const double sin_theta = std::sin(theta_);
    
    Eigen::Matrix3d m; 
    m << cos_theta, -sin_theta, x_, sin_theta, cos_theta, y_, 0, 0, 1;
    
    return m;
}
} // namespace robot_math
