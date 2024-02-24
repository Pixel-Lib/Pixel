#include <cmath>
#include <chrono>
#include "pxl/util.hpp"

namespace pxl {

/**
 * @class PID
 * @brief A class representing a Proportional-Integral-Derivative (PID) controller.
 *
 * This class provides a convenient way to implement and use a PID controller for feedback systems.
 */
class PID {
    public:
        /**
         * Constructor for the PID controller.
         *
         * @param kP The proportional gain.
         * @param kI The integral gain.
         * @param kD The derivative gain.
         */
        PID(float kP, float kI, float kD)
            : kP(kP),
              kI(kI),
              kD(kD) {}

        /**
         * Updates the PID controller output based on the current error.
         *
         * This function calculates the PID controller output based on the current error value and updates the internal
         * state of the controller.
         *
         * @param error The current error value.
         * @return The calculated PID controller output.
         */
        float update(const float error);

        /**
         * Resets the internal state of the PID controller.
         *
         * This function sets the integral term and the previous error value to zero.
         */
        void reset();
    protected:
        /**
         * The proportional gain (Kp).
         */
        float kP;

        /**
         * The integral gain (Ki).
         */
        float kI;

        /**
         * The derivative gain (Kd).
         */
        float kD;

        /**
         * The integral term of the PID controller.
         */
        float integral = 0;

        /**
         * The previous error value used for calculating the derivative term.
         */
        float prevError = 0;
    private:
        /**
         * The previous time point used for calculating the time difference.
         */
        time_point<system_clock> prevTime;
};

} // namespace pxl
