#include "pxl/drivebase/odom.hpp"

#include <iostream>

#include "pxl/util.hpp"
#include "trackingwheel.hpp"

namespace pxl {
Odom::Odom(std::vector<std::unique_ptr<TrackingWheel>> &verticals,
           std::vector<std::unique_ptr<TrackingWheel>> &horizontals,
           std::vector<std::unique_ptr<TrackingWheel>> &drivetrain, std::vector<std::shared_ptr<pros::IMU>> &imu)
    : verticals(std::move(verticals)),
      horizontals(std::move(horizontals)),
      drivetrain(std::move(drivetrain)),
      imu(std::move(imu)) {}

void Odom::calibrate(bool calibrateIMUs) {
    std::vector<std::unique_ptr<TrackingWheel>> newDrivetrain = {};
    std::vector<std::unique_ptr<TrackingWheel>> newVerticals = {};
    std::vector<std::unique_ptr<TrackingWheel>> newHorizontals = {};
    std::vector<std::shared_ptr<pros::IMU>> newIMUs = std::move(this->imu);

    // calibrate vertical tracking wheels
    for (auto it = this->verticals.begin(); it != this->verticals.end(); it++) {
        if ((*it)->getOffset()) {
            std::cout << "Vertical tracker at offset " << (*it)->getOffset() << " failed calibration!" << std::endl;
        } else
            newVerticals.push_back(std::move(*it));
    }  // move successful calibrations to new vector

    // calibrate horizontal tracking wheels
    for (auto it = this->horizontals.begin(); it != this->horizontals.end(); it++) {
        if (pxl::sgn((*it)->getOffset() == 1)) std::cout << "Left drivetrain motor failed to calibrate!" << std::endl;
    }  // move successful calibrations to new vector

    // calibrate IMUs
    for (auto &it : this->imu) it.reset();
    pxl::Timer timer(3000);  // try calibrating IMUs for 3000 ms
    while (!timer.isDone()) {
        for (auto &IMU : this->imu) {  // continuously calibrate in case of failure
            if (!IMU->is_calibrating() && !IMU->is_calibrating()) IMU->reset();
        }
        pros::delay(10);
    }  // wait for IMUs to calibrate

    for (auto it = this->imu.begin(); it != this->imu.end(); it++) {
        if (!(*it)->is_calibrating()) {
            std::cout << "IMU failed to calibrate! Removing..." << std::endl;
        } else {
            newIMUs.push_back(*it);
        }
    }  // move successful calibrations to new vector

    this->verticals = std::move(newVerticals);
    this->horizontals = std::move(newHorizontals);
    this->drivetrain = std::move(newDrivetrain);
    this->imu = std::move(newIMUs);
}

float Odom::calcDeltaTheta(std::vector<std::shared_ptr<pros::IMU>> &imu, bool update) {
    auto getRotation = [this](const std::shared_ptr<pros::IMU> &imu) {
        if (this->imu.empty()) {
            return 0.0f;  // Return 0 if the IMU vector is empty
        }
        // Create a lambda to calculate the rotation
        auto Rotation = [this](const std::shared_ptr<pros::IMU> &imu) {
            std::vector<float> rotations;
            for (const auto &imuPtr : this->imu) { rotations.push_back(imuPtr->get_rotation()); }
            return rotations;
        };
        const float rotation = M_PI_2 - degToRad(pxl::avg(Rotation(imu)));
        return rotation;
    };

    std::vector<float> deltaAngles;
    const float prevAngle = this->lastAngle;  // save lastAngle, as it will get reset when calling
                                              // getAngle() below

    for (const auto &imu : this->imu) {
        const float angle = getRotation(imu);
        deltaAngles.push_back(angle - prevAngle);
        if (update) this->lastAngle = angle;
    }

    return pxl::avg(deltaAngles);
}

float Odom::calcDeltaTheta(TrackingWheel &tracker1, TrackingWheel &tracker2) {
    return (tracker1.getDistanceDelta(false) - tracker2.getDistanceDelta(false))
           / (tracker1.getOffset() - tracker2.getOffset());
}
float Odom::calculateLocal(std::vector<std::unique_ptr<TrackingWheel>> &trackers, float deltaTheta, float sinDTheta2) {
    float localComponent = 0;
    for (auto &tracker : trackers) {
        const float radius = (deltaTheta == 0) ? tracker->getDistanceDelta()
                                               : tracker->getDistanceDelta() / deltaTheta + tracker->getOffset();
        localComponent += sinDTheta2 * radius / trackers.size();
    }
    return localComponent;
}
void Odom::update() {
    float theta = this->pose.theta;
    if (this->imu.size() > 0) {
        theta += Odom::calcDeltaTheta(this->imu);
    } else if (horizontals.size() > 1) {
        theta += Odom::calcDeltaTheta(*this->horizontals.at(0), *this->horizontals.at(1));
    } else if (verticals.size() > 1) {
        theta += Odom::calcDeltaTheta(*this->verticals.at(0), *this->verticals.at(1));
    } else if (drivetrain.size() > 1) {
        theta += Odom::calcDeltaTheta(*this->drivetrain.at(0), *this->drivetrain.at(1));
    } else {
        std::cerr << "Odom calculation failure! Not enough sensors to calculate heading" << std::endl;
        return;
    }

    const float deltaTheta = theta - this->pose.theta;  // change in angle
    const float avgTheta = this->pose.theta + deltaTheta / 2;
    Pose local(0, 0, deltaTheta);
    const float sinDTheta2 = (deltaTheta == 0) ? 1 : 2 * std::sin(deltaTheta / 2);

    const Coord LocalMatrix(calculateLocal(this->verticals, deltaTheta, sinDTheta2),
                            calculateLocal(this->horizontals, deltaTheta, sinDTheta2));

    local.y += LocalMatrix.y;
    local.x += LocalMatrix.x;
    if (this->verticals.size() == 0) {
        local.x += calculateLocal(this->drivetrain, deltaTheta, sinDTheta2);
        if (this->drivetrain.size() == 0) {
            std::cout << "No vertical tracking wheels! Assuming y movement is 0" << std::endl;
        }
    }

    this->pose += local.rotate(avgTheta);
}

Pose Odom::getPose(bool radians) { return radians ? this->pose : (pose.theta = radToDeg(pose.theta), pose); }

void Odom::init() {
    OdomTask = OdomTask ? OdomTask : new pros::Task([this]() {
        while (true) {
            update();
            pros::delay(10);
        }
    });
}
}  // namespace pxl