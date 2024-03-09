#include "pxl/util.hpp"
#include "pxl/pid.hpp"

namespace pxl {
    // Constructor for a generic PID controller
    PID::PID(float kP, float kI, float kD) : kP(kP), kI(kI), kD(kD) {}

    // Constructor for a lateral PID controller
    PID::PID(float kP, float kI, float kD, float kP_d) : kP(kP), kI(kI), kD(kD), kP_d(kP_d) {}

    // Constructor for a lateral PID controller with integralMAX
    PID::PID(float kP, float kI, float kD, float kP_d, float integralMAX)
            : kP(kP), kI(kI), kD(kD), kP_d(kP_d), integralMAX(integralMAX) {}

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
}