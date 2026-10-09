#include "robot_math/se2.hpp"

#include <cmath>

namespace robot_math{

double wrap_angle(double theta){
    return std::atan2(std::sin(theta), std::cos(theta));
}

} // namespace robot_math
