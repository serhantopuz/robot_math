#include "robot_math/se2.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace robot_math{
constexpr double kRotationTolerance = 1e-6;

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
SE2 SE2::from_matrix(const Eigen::Matrix3d& m){
    if (!m.row(2).isApprox(Eigen::RowVector3d(0, 0, 1), kRotationTolerance)){
        throw std::invalid_argument("Expected [0, 0, 1] for bottom row, got " + std::to_string(m(2,0)) + ", " + std::to_string(m(2,1)) + ", " + std::to_string(m(2,2)));
    }

    const Eigen::Matrix2d R = m.topLeftCorner<2, 2>();
    const Eigen::Vector2d c1 = R.col(0);
    const Eigen::Vector2d c2 = R.col(1);
    
    if (std::abs(c1.squaredNorm() - 1.0) > kRotationTolerance || std::abs(c2.squaredNorm() - 1.0) > kRotationTolerance){
        throw std::invalid_argument("Expected rotation columns of length 1 (a rotation cannot stretch), got squared lengths " + std::to_string(c1.squaredNorm()) + " and " + std::to_string(c2.squaredNorm()));
    }
    if (std::abs(c1.dot(c2)) > kRotationTolerance){
        throw std::invalid_argument("Expected perpendicular rotation columns (a rotation cannot shear), got dot product " + std::to_string(c1.dot(c2)));
    }
    if (std::abs(R.determinant() - 1.0) > kRotationTolerance){
        throw std::invalid_argument("Expected rotation determinant +1 (a rotation cannot mirror), got " + std::to_string(R.determinant()));
    }
    
    const double x = m(0, 2);
    const double y = m(1, 2);
    const double theta = std::atan2(m(1, 0), m(0,0));

    return SE2(x, y, theta);
}
} // namespace robot_math
