#pragma once
#include <Eigen/Dense>

namespace robot_math {
/// Wrap an angle in radians into [-π, π].
double wrap_angle(double theta);

/// A rigid-body pose in the plane: a position and heading.
class SE2 {
 public:
  SE2(double x, double y, double theta);

  static SE2 identity();
  /// Throws std::invalid_argument if m is not a rigid transform.
  static SE2 from_matrix(const Eigen::Matrix3d& m);
  /// Compose: T_A_B*T_B_C = T_A_C
  SE2 operator*(const SE2& other) const;
  /// Inverse: T_A_B -> T_B_A
  SE2 inverse() const;

  double x() const { return x_; }
  double y() const { return y_; }
  double theta() const { return theta_; }

  Eigen::Matrix3d to_matrix() const;
  /// Map one point from this pose's child frame into its parent frame.
  Eigen::Vector2d transform_point(const Eigen::Vector2d& p) const;
  /// Map points (one per column, 2 x N) from the child frame into the parent frame.
  Eigen::Matrix2Xd transform_points(const Eigen::Matrix2Xd& points) const;

 private:
  double x_;
  double y_;
  double theta_;
};
}  // namespace robot_math
