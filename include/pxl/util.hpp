#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <type_traits>
#include <vector>

namespace pxl {

/**
 * @brief Clamps a value to a specified range.
 *
 * This function ensures the input value remains within the specified minimum
 * and maximum values.
 *
 * @param target The desired target value.
 * @param current The current value.
 * @param maxChange The maximum allowed change per update.
 * @return The clamped value.
 */
float slew(float target, float current, float maxChange);

/**
 * @brief Converts radians to degrees.
 *
 * This function converts an angle from radians to degrees.
 *
 * @param rad The angle in radians.
 * @return The angle in degrees.
 */
float radToDeg(float rad);

/**
 * @brief Converts degrees to radians.
 *
 * This function converts an angle from degrees to radians.
 *
 * @param deg The angle in degrees.
 * @return The angle in radians.
 */
float degToRad(float deg);

/**
 * @brief Wraps an angle to the range [-180, 180] degrees.
 *
 * This function wraps an angle to the specified range, ensuring it remains
 * within the bounds.
 *
 * @param deg The angle in degrees.
 * @return The wrapped angle in degrees.
 */
float wrapTo180(float deg);

/**
 * @brief Wraps an angle to the range [0, 360] degrees.
 *
 * This function wraps an angle to the specified range, ensuring it remains
 * within the bounds.
 *
 * @param deg The angle in degrees.
 * @return The wrapped angle in degrees.
 */
float wrapTo360(float deg);

/**
 * @brief Wraps an angle to the range [-Pi, Pi] radians.
 *
 * This function wraps an angle to the specified range, ensuring it remains
 * within the bounds.
 *
 * @param rad The angle in radians.
 * @return The wrapped angle in radians.
 */
float wrapToPi(float rad);

/**
 * @brief Wraps an angle to the range [0, 2*Pi] radians.
 *
 * This function wraps an angle to the specified range, ensuring it remains
 * within the bounds.
 *
 * @param rad The angle in radians.
 * @return The wrapped angle in radians.
 */
float wrapTo2Pi(float rad);

/**
 * @brief Calculates the smallest angular difference between two angles.
 *
 * This function computes the minimum difference between two angles, considering
 * the specified range (degrees or radians).
 *
 * @param angle1 The first angle.
 * @param angle2 The second angle.
 * @param radians True if angles are in radians, false for degrees.
 * @return The smallest angular difference.
 */
float angleError(float angle1, float angle2, bool radians);

/**
 * @brief Returns the sign of a number (-1 for negative, 1 for positive, 0 for
 * zero).
 *
 * This function returns the sign (+1, -1, or 0) of a given number(in any
 * datatype)
 *
 * @tparam x The number.
 * @return The sign of x.
 */
template <typename T> T sgn(T x) {
    return std::is_floating_point_v<T> ? (std::signbit(x) ? T(-1) : T(1)) : ((x < T(0)) ? T(-1) : T(1));
}

/**
 * The function calculates the exponential moving average (EMA) of a current
 * value based on a previous value and a smoothing factor.
 *
 * @param current The `current` parameter represents the current value for which
 * you want to calculate the Exponential Moving Average (EMA).
 * @param previous The "previous" parameter in the `ema` function represents the
 * previous value that you want to use in the exponential moving average
 * calculation. This value is used in the formula to calculate the new
 * exponential moving average based on the current value and the smoothing
 * factor.
 * @param smooth The `smooth` parameter in the `ema` function represents the
 * weight given to the current value `current` when calculating the exponential
 * moving average. A higher `smooth` value will give more weight to the current
 * value, making the EMA more responsive to recent changes. On the other hand, a
 *
 * @return the Exponential Moving Average (EMA) calculated using the formula:
 * (current * smooth) + (previous * (1 - smooth)).
 */
float ema(float current, float previous, float smooth);

/**
 * @brief Calculates the average of a vector of any type.
 *
 * This function computes the arithmetic mean of a vector of elements of any
 * type.
 *
 * @tparam vec The vector of elements.
 * @return The average value (in any datatype).
 */
template <typename T> T avg(const std::vector<T> &vec) {
    T sum = T();  // Initialize sum to default value for T
    for (const T &elem : vec) { sum += elem; }
    return sum / vec.size();
}

std::pair<float,float> normalize(float lateralOut, float angularOut, float maxSpeed);

}  // namespace pxl
