#include "pxl/drivebase/odom.hpp"
#include "pxl/timer.hpp"
#include "pros/imu.hpp"
#include "pxl/util.hpp"

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

float Odom::getRotationDelta(bool update) {
    auto getRotation = [this](const std::shared_ptr<pros::IMU>& imu) {
        if (!imu) {
            return 0.0f; // Return 0 if the IMU pointer is null
        }
        const float rotation = M_PI_2 - degToRad(imu->get_rotation());
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

void Odom::update() {
    // TODO: Implement update logic for odom
}

} // namespace pxl