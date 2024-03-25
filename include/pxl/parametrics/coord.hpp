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

        /**
         * @brief Construct a new empty `Coord` object
         * 
         */
        Coord() = default;

        /**
         * @brief Adds two coordinates together.
         * @param other The other coordinate to add.
         * @return The sum of the two coordinates.
         */
        Coord operator+(const Coord &other) const;

        /**
         * @brief Subtracts one coordinate from another.
         * @param other The coordinate to subtract.
         * @return The difference between the two coordinates.
         */
        Coord operator-(const Coord &other) const;

        /**
         * @brief Calculates the dot product of two coordinates.
         * @param other The other coordinate.
         * @return The dot product of the two coordinates.
         */
        float operator*(const Coord &other) const;

        /**
         * @brief Multiplies a coordinate by a scalar value.
         * @param scalar The scalar value.
         * @return The scaled coordinate.
         */
        Coord operator*(const float &scalar) const;

        /**
         * @brief Divides a coordinate by a scalar value.
         * @param scalar The scalar value.
         * @return The divided coordinate.
         */
        Coord operator/(const float &scalar) const;

        /**
         * @brief Adds another coordinate to this coordinate.
         * @param other The other coordinate to add.
         * @return A reference to this coordinate after the addition.
         */
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