#pragma once

#include "pxl/parametrics/coord.hpp"

namespace pxl {
class Pose : public pxl::Coord {
    public:
    float theta;

    Pose(float x, float y, float theta);

    // Constructor to convert pxl::Coord to Pose
    Pose(const pxl::Coord &other);

    float getCurvature(Pose &pose, Pose &other);
};
}  // namespace pxl