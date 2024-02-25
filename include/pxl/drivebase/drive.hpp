#pragma once

#include <memory>
#include "main.h"
#include "drive_sensors.hpp"
#include "pros/motors.hpp"

namespace pxl {

class Drivetrain {
    public:
        Drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors, float trackWidth, float wheelDiameter,
                   float rpm)
            : leftMotors(leftMotors),
              rightMotors(rightMotors),
              trackWidth(trackWidth),
              wheelDiameter(wheelDiameter),
              rpm(rpm) {}

        // Add any necessary member functions here
    private:
        pros::MotorGroup* leftMotors;
        pros::MotorGroup* rightMotors;
        float trackWidth;
        float wheelDiameter;
        float rpm;
        float chasePower;
};

class ExtendedDrivetrain {
    public:
        ExtendedDrivetrain(float verticalTrackWidth)
            : verticalTrackWidth(verticalTrackWidth) {}
    private:
        float verticalTrackWidth;
};

}; // namespace pxl
