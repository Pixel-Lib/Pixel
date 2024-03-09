#pragma once

#include <cmath>

namespace pxl {

/**
 * @brief Represents a coordinate in 2D space.
 */
class Coord {
    public:
        float x, y;

        /**
         * @brief Constructs a Coord object with the given x and y coordinates.
         * @param x The x coordinate.
         * @param y The y coordinate.
         */
        Coord(float x, float y) : x(x), y(y) {}

        // Overloaded Operators
        Coord operator+(const Coord &other) const;
        Coord operator-(const Coord &other) const;
        float operator*(const Coord &other) const;
        Coord operator*(const float &scalar) const;
        Coord operator/(const float &scalar) const;
        Coord &operator+=(const Coord &other);

        /**
         * @brief Performs linear interpolation between this coordinate and another coordinate.
         * @param other The other coordinate to interpolate towards.
         * @param t The interpolation factor (between 0 and 1).
         * @return The interpolated coordinate.
         */
        Coord lerp(Coord other, float t) const;

        /**
         * @brief Calculates the Euclidean distance between this coordinate and another coordinate.
         * @param other The other coordinate.
         * @return The distance between the two coordinates.
         */
        float distance(const Coord &other) const;

        /**
         * @brief Calculates the angle between this coordinate and another coordinate.
         * @param other The other coordinate.
         * @return The angle (in radians) between the two coordinates.
         */
        float angle(const Coord &other) const;

        /**
         * @brief Rotates this coordinate around the origin by the specified angle.
         * @param angle The angle (in radians) to rotate by.
         * @return The rotated coordinate.
         */
        Coord rotate(float angle) const;
};

}  // namespace pxl