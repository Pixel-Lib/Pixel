
#pragma once

#include <memory>
#include "main.h"
#include <vector>
#include "math.h"
#include "pxl/util.hpp"
#include <algorithm>
#include "pxl/timer.hpp"
#include <iterator>

namespace pxl {

/**
 * Represents a tracking wheel on a robot.
 * Can be constructed with an encoder, rotation sensor, or motor group.
 */
class TrackingWheel {
    public:
        // Constructor with encoder
        TrackingWheel(pros::ADIEncoder* encoder, float wheelDiameter, float distance, float gearRatio);

        // Constructor with rotation sensor
        TrackingWheel(pros::Rotation* rotation, float wheelDiameter, float distance, float gearRatio);

        // Constructor with motor group
        TrackingWheel(pros::MotorGroup* motors, float wheelDiameter, float distance, float rpm);

        // Reset the optocal encoder measurement
        bool reset();

        // Get the distance traveled by the wheel
        float getDistanceTraveled();

        // Get the wheel offset
        float getOffset();

        // Get the type of wheel (0 for encoder/rotation, 1 for motor group)
        int getType();
    private:
        std::unique_ptr<pros::ADIEncoder> encoder; // Pointer to encoder (optional)
        std::unique_ptr<pros::Rotation> rotation; // Pointer to rotation sensor (optional)
        std::unique_ptr<pros::MotorGroup> motors; // Pointer to motor group (optional)
        float diameter; // Wheel diameter in meters
        float distance; // Wheel offset from the robot center in meters
        float gearRatio; // Gear ratio of the wheel (encoder/motor)
        float rpm; // RPM of the motor group (optional)
};

} // namespace pxl
