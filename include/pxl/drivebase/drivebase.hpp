#pragma once

#include <iostream>
#include <memory>

#include "pxl/aSync.hpp"
#include "pxl/drivebase/odom.hpp"
#include "pxl/seekingcontroller.hpp"

namespace pxl {
class Drive_;
// class Drivetrain;
struct OdomSensors {
        OdomSensors(TrackingWheel *vertical1, TrackingWheel *vertical2, TrackingWheel *horizontal1,
                    TrackingWheel *horizontal2, pros::Imu *imu);
        TrackingWheel *vertical1;
        TrackingWheel *vertical2;
        TrackingWheel *horizontal1;
        TrackingWheel *horizontal2;
        pros::IMU *imu;
};

/**
 * @class Drivetrain
 * @brief Represents a drivetrain for a robot.
 *
 * The Drivetrain class provides functionality to control the movement of a robot's drivetrain.
 * It encapsulates the left and right motor groups, track width, wheel diameter, and RPM of the drivetrain.
 */
class Drivetrain {
    public:
        Drivetrain(pros::MotorGroup *leftMotors, pros::MotorGroup *rightMotors, float trackWidth, float wheelDiameter,
                   float rpm);
        Drivetrain() = default;
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
        friend class Drivetrain;

        //* MOTIONS *//
        pros::Mutex mutex;
        // get the current competition state. If this changes, the movement will stop
        uint8_t compstate = pros::competition::get_status();  // global variable

        struct driveParams {
                float minSpeed = 0;
                float maxSpeed = 127;
                float slew = NAN;
        };
        static std::shared_ptr<driveParams> defaultDriveParams() { return std::make_shared<driveParams>(); }
        void Drive(float target, float timeout, std::shared_ptr<driveParams> params = defaultDriveParams(),
                   bool async = true);

    private:
        OdomSensors odomSensors = {nullptr, nullptr, nullptr, nullptr, nullptr};
        void calibrateIMU(OdomSensors sensors);
        Odom setSensors(OdomSensors sensors);
std::pair<float,float> slewSpeedLimits(std::shared_ptr<driveParams> driveParams, SeekingController& seekingController);

    public:
        SeekingController linearController;
        SeekingController angularController;
        Odom odom;
        bool isDriverControl();
        Drivetrain drivetrain;
        OdomSensors sensors;
};

};  // namespace pxl
