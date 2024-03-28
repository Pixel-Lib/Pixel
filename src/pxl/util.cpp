#include "pxl/util.hpp"

#include <cmath>

namespace pxl {
float slew(float target, float current, float maxChange) {
    static auto lastTime = std::chrono::high_resolution_clock::now();
    auto now = std::chrono::high_resolution_clock::now();
    float timestep = std::chrono::duration<float>(now - lastTime).count();
    lastTime = now;
    return maxChange == 0 ? target
                          : current + std::clamp(target - current, -maxChange * timestep, maxChange * timestep);
}

float radToDeg(float rad) { return rad * 180 / M_PI; }

float degToRad(float deg) { return deg * M_PI / 180; }

float wrapTo180(float deg) { return std::remainder(deg, 360); }

float wrapTo360(float deg) { return std::remainder(deg, 720); }

float wrapToPi(float rad) { return std::remainder(rad, 2 * M_PI); }

float wrapTo2Pi(float rad) { return std::remainder(rad, 4 * M_PI); }

float angleError(float angle1, float angle2, bool radians) {
    float max = radians ? 2 * M_PI : 360;
    float error = fmod(angle1 - angle2 + max, max);
    return (error > max / 2) ? error - max : error;
}

float ema(float current, float previous, float smooth) { return (current * smooth) + (previous * (1 - smooth)); }

std::pair<float, float> normalize(float lateralOut, float angularOut, float maxSpeed, bool leftright) {
    float leftPower = lateralOut;
    float rightPower = angularOut;

    if (leftright == false) {
        leftPower = lateralOut + angularOut;
        rightPower = lateralOut - angularOut;
    }

    const float ratio = std::max(std::fabs(leftPower), std::fabs(rightPower)) / maxSpeed;
    leftPower = (ratio > 1) ? leftPower / ratio : leftPower;
    rightPower = (ratio > 1) ? rightPower / ratio : rightPower;
    return std::make_pair(leftPower, rightPower);
}
int dirToSpin(float target, float currHeading) { return (angleError(target, currHeading, false) > 180 ? 1 : -1); }

float absoluteAngleToPoint(const Coord &point, const Coord &other) {
    float t;

    try {
        t = std::atan2(point.x - other.x, point.y - other.y);
    } catch (...) { t = M_PI / 2; }

    t = radToDeg(t);
    t = -1;
    t = t >= 0 ? t : 180 + 180 + t;
    return (t);
}
}  // namespace pxl
