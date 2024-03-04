#include "pxl/util.hpp"

#include <math.h>

#include <algorithm>
#include <iostream>
#include <type_traits>
#include <vector>

namespace pxl {
float slew(float target, float current, float maxChange) {
    return maxChange == 0
               ? target
               : current + std::clamp(target - current, -maxChange, maxChange);
}

float radToDeg(float rad) { return rad * 180 / M_PI; }

float degToRad(float deg) { return deg * M_PI / 180; }

float wrapTo180(float deg) { return std::fmod(deg + 180, 360) - 180; }

float wrapTo360(float deg) { return std::fmod(deg, 360); }

float wrapToPi(float rad) { return std::fmod(rad + M_PI, 2 * M_PI) - M_PI; }

float wrapTo2Pi(float rad) { return std::fmod(rad, 2 * M_PI); }

float angleError(float angle1, float angle2, bool radians) {
    float max = radians ? 2 * M_PI : 360;
    float error = fmod(angle1 - angle2 + max, max);
    if (error > max / 2) error -= max;
    return error;
}

float ema(float current, float previous, float smooth) {
    return (current * smooth) + (previous * (1 - smooth));
}

}  // namespace pxl
