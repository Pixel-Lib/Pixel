#pragma once

#include <memory>
#include <iostream>

// #include "pxl/movements/drive.hpp"
#include "pxl/aSync.hpp"
#include "pxl/drivebase/odom.hpp"
#include "pxl/seekingcontroller.hpp"

namespace pxl {
    class Drive_;
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
        /**
         * @brief Constructs a Drivebase object.
         *
         * This constructor initializes a Drivebase object with the specified drivetrain, odometry sensors,
         * linear controller, and angular controller.
         *
         * @param drivetrain The drivetrain object used for controlling the robot's movement.
         * @param sensors The odometry sensors used for tracking the robot's position and orientation.
         * @param linearController The controller used for controlling linear movement.
         * @param angularController The controller used for controlling angular movement.
         */
        Drivebase(Drivetrain drivetrain, OdomSensors sensors, SeekingController linearController,
                  SeekingController angularController);

        /**
         * @brief callibrate the drivebase's sensors. Callibrates the IMU and resets the tracking wheels.
         * 
         * @param calibrateIMU callibrate the IMU. `true` by default
         */
        void calibrate(bool calibrateIMU = true);


        // friends

        friend class Drive_;
    private:

        OdomSensors odomSensors = {nullptr, nullptr, nullptr, nullptr, nullptr};
        void calibrateIMU(OdomSensors sensors);
        Odom setSensors(OdomSensors sensors);
        protected:
        
                SeekingController linearController;
        SeekingController angularController;
        Odom odom;
         bool isDriverControl();
                 Drivetrain drivetrain;
        OdomSensors sensors;
};

};  // namespace pxl
