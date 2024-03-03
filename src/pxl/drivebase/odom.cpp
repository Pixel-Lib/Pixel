#include "pxl/drivebase/odom.hpp"
#include "pros/imu.hpp"

#include <iostream>

namespace pxl {
Odom::Odom(std::vector<std::unique_ptr<TrackingWheel>>& verticals,
           std::vector<std::unique_ptr<TrackingWheel>>& horizontals,
           std::vector<std::unique_ptr<TrackingWheel>>& drivetrain, std::vector<std::shared_ptr<pros::IMU>>& imu)
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
        } else newVerticals.push_back(std::move(*it));
    }

    // calibrate horizontal tracking wheels
    for (auto it = this->horizontals.begin(); it != this->horizontals.end(); it++) {
        if (pxl::sgn((*it)->getOffset() == 1)) std::cout << "Left drivetrain motor failed to calibrate!" << std::endl;
    }

    // calibrate IMUs
    for (auto& it : this->imu) it.reset();
    pxl::Timer timer(3000); // try calibrating IMUs for 3000 ms
    while (!timer.isDone()) {
        for (auto& IMU : this->imu) { // continuously calibrate in case of failure
            if (!IMU->is_calibrating() && !IMU->is_calibrating()) IMU->reset();
        }
        pros::delay(10);
    }

    for (auto it = this->imu.begin(); it != this->imu.end(); it++) {
        if (!(*it)->is_calibrating()) {
            std::cout << "IMU failed to calibrate! Removing..." << std::endl;
        } else {
            newIMUs.push_back(*it);
        }
    }

    this->verticals = std::move(newVerticals);
    this->horizontals = std::move(newHorizontals);
    this->drivetrain = std::move(newDrivetrain);
    this->imu = std::move(newIMUs);
}

float Odom::calcDeltaTheta(std::vector<std::shared_ptr<pros::IMU>>& imu, bool update) {
    auto getRotation = [this](const std::shared_ptr<pros::IMU>& imu) {
        if (this->imu.empty()) {
            return 0.0f; // Return 0 if the IMU vector is empty
        }
        // Create a lambda to calculate the rotation
        auto Rotation = [this](const std::shared_ptr<pros::IMU>& imu) {
            std::vector<float> rotations;
            for (const auto& imuPtr : this->imu) { rotations.push_back(imuPtr->get_rotation()); }
            return rotations;
        };
        const float rotation = M_PI_2 - degToRad(pxl::avg(Rotation(imu)));
        return rotation;
    };

    std::vector<float> deltaAngles;
    const float prevAngle = this->lastAngle; // save lastAngle, as it will get reset when calling getAngle() below

    for (const auto& imu : this->imu) {
        const float angle = getRotation(imu);
        deltaAngles.push_back(angle - prevAngle);
        if (update) this->lastAngle = angle;
    }

    return pxl::avg(deltaAngles);
}

float Odom::calcDeltaTheta(std::vector<std::unique_ptr<TrackingWheel>>& tracker1,
                           std::vector<std::unique_ptr<TrackingWheel>>& tracker2) {
    auto distanceDeltas1 = std::vector<float>();
    auto distanceDeltas2 = std::vector<float>();
    auto offsets1 = std::vector<float>();
    auto offsets2 = std::vector<float>();

    for (const auto& wheel : tracker1) {
        distanceDeltas1.push_back(wheel->getDistanceDelta(false));
        offsets1.push_back(wheel->getOffset());
    }

    for (const auto& wheel : tracker2) {
        distanceDeltas2.push_back(wheel->getDistanceDelta(false));
        offsets2.push_back(wheel->getOffset());
    }

    const float numerator = pxl::avg(distanceDeltas1) - pxl::avg(distanceDeltas2);
    const float denominator = pxl::avg(offsets1) - pxl::avg(offsets2);
    return numerator / denominator;
}

void Odom::update() {
    float theta = this->pose.theta;
    if (this->imu.size() > 0) {
        theta += Odom::calcDeltaTheta(this->imu);
    } else if (horizontals.size() > 1) {
        std::vector<std::unique_ptr<TrackingWheel>> horizontalsSubset = {std::move(this->horizontals.at(0)),
                                                                         std::move(this->horizontals.at(1))};
        theta += Odom::calcDeltaTheta(horizontalsSubset, horizontalsSubset);
    } else if (verticals.size() > 1) {
        std::vector<std::unique_ptr<TrackingWheel>> verticalsSubset = {std::move(this->verticals.at(0)),
                                                                       std::move(this->verticals.at(1))};
        theta += Odom::calcDeltaTheta(verticalsSubset, verticalsSubset);
    } else if (drivetrain.size() > 1) {
        std::vector<std::unique_ptr<TrackingWheel>> drivetrainSubset = {std::move(this->drivetrain.at(0)),
                                                                        std::move(this->drivetrain.at(1))};
        theta += Odom::calcDeltaTheta(drivetrainSubset, drivetrainSubset);
    } else {
        std::cerr << "Odom calculation failure! Not enough sensors to calculate heading" << std::endl;
        return;
    }
    const float deltaTheta = theta - this->pose.theta; // change in angle
    const float avgTheta = this->pose.theta + deltaTheta / 2;

    Pose local(0, 0, deltaTheta);
    const float sinDTheta2 = (deltaTheta == 0) ? 1 : 2 * std::sin(deltaTheta / 2);

    for (auto& tracker : this->horizontals) {
        const float radius = (deltaTheta == 0) ? tracker->getDistanceDelta()
                                               : tracker->getDistanceDelta() / deltaTheta + tracker->getOffset();
        local.y += sinDTheta2 * radius / this->horizontals.size();
    }

    if (this->verticals.size() > 0) {
        for (auto& tracker : this->verticals) {
            const float radius = (deltaTheta == 0) ? tracker->getDistanceDelta()
                                                   : tracker->getDistanceDelta() / deltaTheta + tracker->getOffset();
            local.x += sinDTheta2 * radius / this->verticals.size();
        }
    } else if (this->drivetrain.size() > 0) {
        for (auto& motor : this->drivetrain) {
            const float radius = (deltaTheta == 0) ? motor->getDistanceDelta()
                                                   : motor->getDistanceDelta() / deltaTheta + motor->getOffset();
            local.x += sinDTheta2 * radius / this->drivetrain.size();
        }
    } else {
        std::cout << "No vertical tracking wheels! Assuming y movement is 0" << std::endl;
    }

    this->pose += local.rotate(avgTheta);
}

} // namespace pxl