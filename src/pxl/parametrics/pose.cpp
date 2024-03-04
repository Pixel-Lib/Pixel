#include "pxl/parametrics/pose.hpp"

#include <math.h>

#include "pxl/util.hpp"

namespace pxl {
Pose::Pose(float x, float y, float theta) : Coord(x, y), theta(theta) {}

Pose::Pose(const Coord &other) : Coord(other), theta() {}

float getCurvature(Pose pose, Pose other) {
    // calculate whether the pose is on the left or right side of the circle
    float side = pxl::sgn(std::sin(pose.theta) * (other.x - pose.x) -
                          std::cos(pose.theta) * (other.y - pose.y));
    // calculate center point and radius
    float a = -std::tan(pose.theta);
    float c = std::tan(pose.theta) * pose.x - pose.y;
    float x = std::fabs(a * other.x + other.y + c) / std::sqrt((a * a) + 1);
    float d = std::hypot(other.x - pose.x, other.y - pose.y);

    // return curvature
    return side * ((2 * x) / (d * d));
}
}  // namespace pxl