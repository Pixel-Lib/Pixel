
#pragma once

#include <algorithm>
#include <iterator>
#include <memory>
#include <vector>

#include "main.h"
#include "math.h"
#include "pxl/timer.hpp"
#include "pxl/util.hpp"

namespace pxl {

/**
 * Represents a tracking wheel on a robot.
 * Can be constructed with an encoder, rotation sensor, or motor group.
 */
/**
 * @brief Represents a tracking wheel used for measuring distance traveled by a robot.
 */
class TrackingWheel {
public:
    /**
     * @brief Constructor for a tracking wheel with an ADIEncoder.
     * @param encoder Pointer to the ADIEncoder object.
     * @param wheelDiameter Diameter of the wheel in meters.
     * @param distance Wheel offset from the robot center in meters.
     * @param gearRatio Gear ratio of the wheel (encoder/motor).
     */
    TrackingWheel(pros::ADIEncoder *encoder, float wheelDiameter, float distance, float gearRatio = 1.0f);

    /**
     * @brief Constructor for a tracking wheel with a Rotation sensor.
     * @param rotation Pointer to the Rotation object.
     * @param wheelDiameter Diameter of the wheel in meters.
     * @param distance Wheel offset from the robot center in meters.
     * @param gearRatio Gear ratio of the wheel (encoder/motor).
     */
    TrackingWheel(pros::Rotation *rotation, float wheelDiameter, float distance, float gearRatio = 1.0f);

    /**
     * @brief Constructor for a tracking wheel with a MotorGroup.
     * @param motors Pointer to the MotorGroup object.
     * @param wheelDiameter Diameter of the wheel in meters.
     * @param distance Wheel offset from the robot center in meters.
     * @param rpm RPM of the motor group.
     */
    TrackingWheel(pros::MotorGroup *motors, float wheelDiameter, float distance, float rpm);

    /**
     * @brief Resets the optical encoder measurement.
     * @return True if the reset was successful, false otherwise.
     */
    bool reset();

    /**
     * @brief Gets the distance traveled by the wheel.
     * @return The distance traveled by the wheel in meters.
     */
    float getDistanceTraveled();

    /**
     * @brief Gets the distance delta or the change in distance.
     * @param update Whether to update the distance delta or not (default: true).
     * @return The distance delta or change in distance in meters.
     */
    float getDistanceDelta(bool update = true);

    /**
     * @brief Gets the wheel offset.
     * @return The wheel offset from the robot center in meters.
     */
    float getOffset();

    /**
     * @brief Gets the type of wheel.
     * @return 0 for encoder/rotation, 1 for motor group.
     */
    int getType();

private:
    std::unique_ptr<pros::ADIEncoder> encoder;  // Pointer to encoder (optional)
    std::unique_ptr<pros::Rotation> rotation;   // Pointer to rotation sensor (optional)
    std::unique_ptr<pros::MotorGroup> motors;   // Pointer to motor group (optional)
    float diameter;                             // Wheel diameter in meters
    float distance;                             // Wheel offset from the robot center in meters
    float gearRatio;                            // Gear ratio of the wheel (encoder/motor)
    float rpm;                                  // RPM of the motor group (optional)
    float lastAngle = 0;                        // Last angle of the wheel
};

}  // namespace pxl
