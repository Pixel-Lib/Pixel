#include <chrono>
#include <cmath>
#include "pxl/pid.hpp"
#include "pxl/util.hpp"

namespace pxl {

    // Member function to update PID
    float PID::update(const float error) {
        // Calculate time difference (dt)
        auto currentTime = std::chrono::system_clock::now();
        std::chrono::duration<float> dt = currentTime - prevTime;
        prevTime = currentTime;

        // Calculate integral
        integral += error * dt.count();
        if (pxl::sgn(error) != pxl::sgn(prevError)) integral = 0;

        // Calculate derivative
        const float derivative = (error - prevError) / dt.count();
        prevError = error;

        // Calculate output
        return error * kP + integral * kI + derivative * kD;
    }

    // Member function to reset PID
    void PID::reset() {
        integral = 0;
        prevError = 0;
    }

    // Member function to set coefficients
    void PID::setConstants(float kP, float kI, float kD) {
        this->kP = kP;
        this->kD = kD;
        this->kI = kI;
    }

}  // namespace pxl