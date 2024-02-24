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
     *PID class constructors 
     *
     *Users can create multiple PID instants and name them to their choosing and use the PID muatator to change their gain values 
     */
    /**
     * @brief Constructor for a genaric PID controller.
     *
     * Users can create multiple PID instants and name them to their choosing and use the PID muatator to change their gain values 
     *
     * @param kP The proportional gain.
     * @param kI The integral gain.
     * @param kD The derivative gain.
     */
    PID(float kP, float kI, float kD)
        : kP(kP), kI(kI), kD(kD) {}

    /**
     * @brief Constructor for a lateral PID controller.
     *
     * Users can create multiple PID instants and name them to their choosing and use the PID muatator to change their gain values 
     *
     * @param kP   The proportional gain.
     * @param kI   The integral gain.
     * @param kD   The derivative gain.
     * @param kP_d The porportinal drift gain 
     */

    PID(float kP, float kI, float kD, float kP_d)
        : kP(kP), kI(kI), kD(kD), kP_d(kP_d) {}


     /**
     * @brief Constructor for a lateral PID controller.
     *
     * @param kP          The proportional gain.
     * @param kI          The integral gain.
     * @param kD          The derivative gain.
     * @param kP_d        The porportinal drift gain 
     * @param integralMAX The max value to clamp integral to
     */

    PID(float kP, float kI, float kD, float kP_d, float integralMAX)
        : kP(kP), kI(kI), kD(kD), kP_d(kP_d), integralMAX(integralMAX) {}
    /**
     * @brief the PID controller output based on the current error.
     *
     * This function calculates the PID controller output based on the current error value and updates the internal state of the controller.
     *
     * @param error The current error value.
     * @return The calculated PID controller output.
     */
    float update(const float error);

    /**
     * @brief Accessor function for Error
     *
     * This function returns the current value of the error varible
     *
     * @return The current error of the PID class instance 
    */
    const float getError();

    /**
     * @brief The internal state of the PID controller.
     *
     * This function sets the integral term and the previous error value to zero.
     */
    void reset();

    void setConstants(PID& constants);

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
     * The porportinal drift gain (kP_d)
     */
    float kP_d;

    /**
     * The proportional term of the PID controller.
     */
    float proportional = 0;

    /**
     * The integral term of the PID controller.
     */
    float integral = 0;

    /* The derivative term of the PID controller.
     */
    float derivative = 0;

     /**
     * The max value of integral to prevent windup.
     */
    float integralMAX = 0;

    /**
     * The previous error value used for calculating the derivative term.
     */
    float prevError = 0;

private:
    /**
     * The previous time point used for calculating the time difference.
     */
    std::chrono::time_point<std::chrono::system_clock> prevTime;


    void updateIntegral(float error, float lastError, float activeDistance, float& integral);
};

} // namespace pxl

