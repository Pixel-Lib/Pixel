#include "pxl/util.hpp"

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

float wrapTo180(float deg) { return std::fmod(deg + 180, 360) - 180; }

float wrapTo360(float deg) { return std::fmod(deg, 360); }

float wrapToPi(float rad) { return std::fmod(rad + M_PI, 2 * M_PI) - M_PI; }

float wrapTo2Pi(float rad) { return std::fmod(rad, 2 * M_PI); }

float angleError(float angle1, float angle2, bool radians) {
    float max = radians ? 2 * M_PI : 360;
    float error = fmod(angle1 - angle2 + max, max);
    return (error > max / 2) ? error - max : error;
}

float ema(float current, float previous, float smooth) { return (current * smooth) + (previous * (1 - smooth)); }

std::pair<float, float> normalize(float lateralOut, float angularOut, float maxSpeed) {
    float leftPower = lateralOut + angularOut;
    float rightPower = lateralOut - angularOut;
    const float ratio = std::max(std::fabs(leftPower), std::fabs(rightPower)) / maxSpeed;
    leftPower = (ratio > 1) ? leftPower / ratio : leftPower;
    rightPower = (ratio > 1) ? rightPower / ratio : rightPower;
    return std::make_pair(leftPower, rightPower);
}

}  // namespace pxl
