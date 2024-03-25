#pragma once
#include "pxl/parametrics/regression.hpp"
#include "pxl/pid.hpp"
#include "pxl/timer.hpp"
#include "pxl/util.hpp"

namespace pxl {
/**
 * @brief The SeekingController class represents a controller used for seeking a target value.
 *
 * This class provides methods to update the controller, start a timer, and check for exit conditions.
 * It also contains private member variables for storing PID parameters, regression, timeout, error, slew rate, and a
 * timer object.
 */
class SeekingController {
    public:
        /**
         * @brief Constructs a SeekingController object with the specified parameters.
         *
         * @param pid The PID object containing the PID parameters.
         * @param slew_ The slew rate for the controller.
         * @param regression The regression object for calculating the output.
         * @param globalTimeout The global timeout value for the controller.
         */
        SeekingController(PID pid, float slew_, Regression regression, float globalTimeout);

        /**
         * @brief Updates the controller with the given error value.
         *
         * @param error The error value to update the controller.
         * @param slew Flag indicating whether to apply the slew rate or not (default: true).
         * @return The updated output value of the controller.
         */
        float update(float error, bool slew = true);

        /**
         * @brief Starts the timer for the controller.
         */
        void timerStart();

        /**
         * @brief Checks if the exit condition is met based on the given error value.
         *
         * @param error The error value to check for the exit condition.
         * @return True if the exit condition is met, false otherwise.
         */
        bool getExit(float error);
        float prevOut = 0;  ///< The previous output value.
        float slew_;        ///< The slew rate for the controller.
    private:
        PID pid;                ///< The PID object containing the PID parameters.
        Regression regression;  ///< The regression object for calculating the output.
        float globalTimeout;    ///< The global timeout value for the controller.
        float error;            ///< The current error value.
        pxl::Timer timer;       ///< The timer object for the controller.
};

}  // namespace pxl
