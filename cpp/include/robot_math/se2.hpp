#pragma once
#include <Eigen/Dense>

namespace robot_math {
///Wrap an angle in radians into [-π, π].
double wrap_angle(double theta);

/// A rigid-body pose in the plane: a position and heading.
class SE2 {
    public:
        SE2(double x, double y, double theta);

        static SE2 identity();
        /// Throws std::invalid_argument if m is not a rigid transform.
        static SE2 from_matrix(const Eigen::Matrix3d& m);

        double x() const {return x_;}
        double y() const {return y_;}
        double theta() const {return theta_;}

        Eigen::Matrix3d to_matrix() const;

    private:
        double x_;
        double y_;
        double theta_;
};
}  // namespace robot_math

