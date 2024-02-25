#include "pxl/parametrics/pose.hpp"
#include "pxl/util.hpp"
#include <math.h>

namespace pxl {
Pose::Pose(float x, float y, float theta)
    : Coord(x, y),
      theta(theta) {}

Pose::Pose(const Coord& other)
    : Coord(other),
      theta() {}


} // namespace pxl
