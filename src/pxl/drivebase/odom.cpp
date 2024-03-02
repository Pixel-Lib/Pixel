#include "pxl/drivebase/odom.hpp"

namespace pxl {
Odom::Odom(std::vector<std::unique_ptr<TrackingWheel>>& verticals,
           std::vector<std::unique_ptr<TrackingWheel>>& horizontals,
           std::vector<std::unique_ptr<TrackingWheel>>& drivetrain, std::vector<std::shared_ptr<pros::IMU>>& imu)
    : verticals(std::move(verticals)),
      horizontals(std::move(horizontals)),
      drivetrain(std::move(drivetrain)),
      imu(imu) {}

void Odom::calibrate(bool calibrateIMUs) {
    // TODO: Implement calibration logic for the tracking wheels and IMUs
    if (calibrateIMUs) {
        for (auto& imu_ptr : imu) {
            imu_ptr->reset();
        }
    }
}

void Odom::update() {
    // TODO: Implement update logic for the tracking wheels and IMUs
}

} // namespace pxl