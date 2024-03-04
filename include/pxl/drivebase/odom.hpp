#pragma once

#include <memory>
#include <numeric>
#include <vector>

#include "math.h"
#include "pros/imu.hpp"
#include "pxl/drivebase/drive.hpp"
#include "pxl/drivebase/trackingwheel.hpp"
#include "pxl/parametrics/pose.hpp"
#include "pxl/timer.hpp"
#include "pxl/util.hpp"

namespace pxl {
class Odom {
    public:
    Odom(std::vector<std::unique_ptr<TrackingWheel>> &verticals,
         std::vector<std::unique_ptr<TrackingWheel>> &horizontals,
         std::vector<std::unique_ptr<TrackingWheel>> &drivetrain,
         std::vector<std::shared_ptr<pros::IMU>> &imu);
    void init();
    void calibrate(bool calibrateIMUs = true);
    void update();

    private:
    std::vector<std::unique_ptr<TrackingWheel>> verticals;
    std::vector<std::unique_ptr<TrackingWheel>> horizontals;
    std::vector<std::unique_ptr<TrackingWheel>> drivetrain;
    std::vector<std::shared_ptr<pros::IMU>> imu;
    Pose pose = Pose(0, 0, 0);
    pros::Task *OdomTask = nullptr;
    float lastAngle = 0;
    float calcDeltaTheta(std::vector<std::shared_ptr<pros::IMU>> &imu,
                         bool update = true);
    float calcDeltaTheta(TrackingWheel &tracker1, TrackingWheel &tracker2);
};

}  // namespace pxl