#pragma once

#include <memory>

#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "pxl/aSync.hpp"
#include "pxl/drivebase/odom.hpp"
#include "pxl/drivebase/trackingwheel.hpp"
#include "pxl/parametrics/pose.hpp"
#include "pxl/pid.hpp"
#include "pxl/seekingcontroller.hpp"

namespace pxl {
struct OdomSensors {
        OdomSensors(TrackingWheel *vertical1, TrackingWheel *vertical2, TrackingWheel *horizontal1,
                    TrackingWheel *horizontal2, pros::Imu *imu);
        TrackingWheel *vertical1;
        TrackingWheel *vertical2;
        TrackingWheel *horizontal1;
        TrackingWheel *horizontal2;
        pros::IMU *imu;
};

class Drivetrain {
    public:
        Drivetrain(pros::MotorGroup *leftMotors, pros::MotorGroup *rightMotors, float trackWidth, float wheelDiameter,
                   float rpm);

        // Add any necessary member functions here

        /**
         * @brief Method to queue a function to happen mid movement
         *
         * @param actionPoint the type of actionPoint to be used
         * @param input input function
         */
        pxl::Coord addOnAction(action_Point actionPoint, void input());

        void setConstants(PID &constants);

        // private:
        pros::MotorGroup *leftMotors;
        pros::MotorGroup *rightMotors;
        float trackWidth;
        float wheelDiameter;
        float rpm;
};

class ExtendedDrivetrain {
    public:
        ExtendedDrivetrain(float verticalTrackWidth);

    private:
        float verticalTrackWidth;
};

/**
 * @brief The Drivebase class represents the base of a robot's drivetrain.
 *
 * The Drivebase class provides methods to control the drivetrain.
 */
class Drivebase {
    public:
        bool isDriverControl();
        Drivebase(Drivetrain drivetrain, OdomSensors sensors, SeekingController linearController,
                  SeekingController angularController);
        void calibrateIMU(OdomSensors sensors);
        Odom setSensors(OdomSensors sensors);
        void calibrate(bool calibrateIMU = true);

    private:
        Drivetrain drivetrain;
        OdomSensors sensors;
        OdomSensors odomSensors = {nullptr, nullptr, nullptr, nullptr, nullptr};
        SeekingController linearController;
        SeekingController angularController;
        Odom odom;
        friend class Drive_;
};

};  // namespace pxl
