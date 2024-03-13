#pragma once

#include <memory>
#include <numeric>
#include <vector>

#include "math.h"
#include "pros/imu.hpp"
#include "pxl/drivebase/trackingwheel.hpp"
#include "pxl/parametrics/pose.hpp"
#include "pxl/timer.hpp"
#include "pxl/util.hpp"

namespace pxl {
/**
 * @brief The Odom class represents an odometry system that tracks the robot's position and orientation.
 *
 * The Odom class uses a combination of tracking wheels and IMUs (Inertial Measurement Units) to calculate
 * the robot's position and orientation on the field. It provides methods for initialization, calibration,
 * and updating the odometry values.
 */
class Odom {
    public:
        /**
         * @brief Constructs an Odom object with the specified tracking wheels and IMUs.
         *
         * @param verticals A vector of unique pointers to vertical tracking wheels.
         * @param horizontals A vector of unique pointers to horizontal tracking wheels.
         * @param drivetrain A vector of unique pointers to drivetrain tracking wheels.
         * @param imu A vector of shared pointers to IMUs.
         */
        Odom(std::vector<std::unique_ptr<TrackingWheel>> &verticals,
             std::vector<std::unique_ptr<TrackingWheel>> &horizontals,
             std::vector<std::unique_ptr<TrackingWheel>> &drivetrain, std::vector<std::shared_ptr<pros::IMU>> &imu);
        /**
         * @brief Construct a new empty Odom object
         *
         */
        Odom() = default;
        /**
         * @brief Initializes the odometry system.
         */
        void init();

        /**
         * @brief Calibrates the odometry system.
         *
         * @param calibrateIMUs Specifies whether to calibrate the IMUs. Default is true.
         */
        void calibrate(bool calibrateIMUs = true);

        /**
         * @brief Updates the odometry system by calculating the robot's position and orientation.
         */
        void update();

        /**
         * @brief Get the `Pose` of the robot
         *
         * @param radians Specifies whether to return the orientation in radians. Default is false.
         * @return The current `Pose` of the robot.
         */
        Pose getPose(bool radians = false);

        /**
         * Calculates the local position using the given tracking wheel measurements.
         *
         * @param trackers The vector of unique pointers to the tracking wheels.
         * @param deltaTheta The change in heading angle.
         * @param sinDTheta2 The sine of half the change in heading angle.
         * @return The calculated local position.
         */
        float calculateLocal(std::vector<std::unique_ptr<TrackingWheel>> &trackers, float deltaTheta, float sinDTheta2);

    private:
        std::vector<std::unique_ptr<TrackingWheel>> verticals;  // Vector of unique pointers to vertical tracking wheels
        std::vector<std::unique_ptr<TrackingWheel>>
            horizontals;  // Vector of unique pointers to horizontal tracking wheels
        std::vector<std::unique_ptr<TrackingWheel>>
            drivetrain;                               // Vector of unique pointers to drivetrain tracking wheels
        std::vector<std::shared_ptr<pros::IMU>> imu;  // Vector of shared pointers to IMUs
        Pose pose = Pose(0, 0, 0);                    // Current pose of the robot
        pros::Task *OdomTask = nullptr;               // Pointer to the odometry task
        float lastAngle = 0;                          // Last recorded angle

        /**
         * @brief Calculates the change in angle using IMUs.
         *
         * @param imu A vector of shared pointers to IMUs.
         * @param update Specifies whether to update the IMU readings. Default is true.
         * @return The change in angle in radians.
         */
        float calcDeltaTheta(std::vector<std::shared_ptr<pros::IMU>> &imu, bool update = true);

        /**
         * @brief Calculates the change in angle using tracking wheels.
         *
         * @param tracker1 The first tracking wheel.
         * @param tracker2 The second tracking wheel.
         * @return The change in angle in radians.
         */
        float calcDeltaTheta(TrackingWheel &tracker1, TrackingWheel &tracker2);
};

}  // namespace pxl