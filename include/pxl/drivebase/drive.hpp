#pragma once

#include <memory>
#include "pxl/pid.hpp"
#include "pxl/aSync.hpp"
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
        
        /**
         * @brief Method to queue a function to happen mid movement 
         *
         * @param actionPoint the type of actionPoint to be used 
         * @param input input function
         */
        pxl::Coord addOnAction(action_Point actionPoint, void input());

        void setConstants(PID& constants);

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
