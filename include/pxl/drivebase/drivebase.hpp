#pragma once

#include <cmath>
#include <iostream>
#include <memory>

#include "pxl/aSync.hpp"
#include "pxl/drivebase/odom.hpp"
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
        ExtendedDrivetrain() = default;
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

        //* DRIVE *//
        pros::Mutex mutex;
        uint8_t compstate = pros::competition::get_status();

        struct driveParams {
                float minSpeed = 0;
                float maxSpeed = 127;
                float slew = NAN;
        };
        static driveParams defaultDriveParams() { return driveParams(); }
        void drive(float target, float timeout, driveParams params = defaultDriveParams(), bool async = true);

        //* TURN_TO_POINT *//
        struct turnToPointParams {
                float minSpeed = 0;
                float maxSpeed = 127;
                float slew = NAN;
        };
        static turnToPointParams defaultTurnParams() { return turnToPointParams(); }
        void turnToPoint(float x, float y, float timeout, turnToPointParams params = defaultTurnParams(),
                         bool async = true);

        //* Boomerang *//
        static bool SemicircleExit(pxl::Pose target, pxl::Coord curr, float radius);
        struct boomerangParams {
                float dlead = 0.5;
                float glead = 0.2;

                bool forward = true;
                float minSpeed = 0;
                float maxSpeed = 127;
                float minAccel = 0;
                float slew = NAN;
        };
        static boomerangParams defaultBoomerangParams() { return boomerangParams(); }
        void boomerang(float x, float y, float theta, float timeout, boomerangParams params = defaultBoomerangParams(),
                       bool async = true);

        //* ARCTURN *//
        struct arcturnParams {
                float radius = NAN;
                int dir = 1;
                bool forward = true;
                float minSpeed = 0;
                float maxSpeed = 127;
                float slew = NAN;
        };
        static arcturnParams defaultArcturnParams() { return arcturnParams(); }
        void arcturn(float target, float timeout, arcturnParams params = defaultArcturnParams(), bool async = true);

    private:
        OdomSensors odomSensors = {nullptr, nullptr, nullptr, nullptr, nullptr};
        void calibrateIMU(OdomSensors sensors);
        Odom setSensors(OdomSensors sensors);
        /**
         * Calculates the slew speed limits for a given object and seeking controller.
         *
         * @tparam T The type of the object.
         * @param object A shared pointer to the object.
         * @param seekingController The seeking controller.
         * @return A pair of floats representing the minimum and maximum speed limits.
         */
        template <typename T> std::pair<float, float> slewSpeedLimits(T object, SeekingController &seekingController) {
            return !isnanf((object.slew))
                       ? (object.slew != 0
                              ? std::make_pair(slew(object.minSpeed, seekingController.prevOut, object.slew),
                                               slew(object.maxSpeed, seekingController.prevOut, object.slew))
                              : std::make_pair(object.minSpeed, object.maxSpeed))
                       : (seekingController.slew_ != 0 ? std::make_pair(
                              slew(object.minSpeed, seekingController.prevOut, seekingController.slew_),
                              slew(object.maxSpeed, seekingController.prevOut, seekingController.slew_))
                                                       : std::make_pair(object.minSpeed, object.maxSpeed));
        }

    public:
        SeekingController linearController;
        SeekingController angularController;
        Odom odom;
        bool isDriverControl();
        Drivetrain drivetrain;
        OdomSensors sensors;

        ExtendedDrivetrain extendedDrivetrain;

        SeekingController secondaryController;
};

};  // namespace pxl
