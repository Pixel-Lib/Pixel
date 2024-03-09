#pragma once

#include "pxl/parametrics/coord.hpp"

namespace pxl {
/**
 * @brief The `Pose` class contains classes and functions related to (x, y, theta) coordinates.
 */
class Pose : public pxl::Coord {
    public:
        float theta;

        /**
         * @brief Constructs a `Pose` object with the specified coordinates and theta.
         * @param x The x-coordinate.
         * @param y The y-coordinate.
         * @param theta The theta value.
         */
        Pose(float x, float y, float theta);

        /**
         * @brief Constructs a `Pose` object from a `Coord` object.
         * @param other The `Coord` object to convert.
         */
        Pose(const pxl::Coord &other);

        /**
         * @brief Calculates the curvature between two `Pose` objects.
         * @param pose The first `Pose` object.
         * @param other The second `Pose` object.
         * @return The curvature between the two `Pose` objects.
         */
        float getCurvature(Pose &pose, Pose &other);
};
}  // namespace pxl