#pragma once

#include <memory>
#include <vector>
#include <numeric>
#include "math.h"
#include "pxl/util.hpp"
#include "pros/imu.hpp"
#include "pxl/drivebase/trackingwheel.hpp"
#include "pxl/drivebase/drive.hpp"
#include "pxl/timer.hpp"
#include "pxl/parametrics/pose.hpp"

namespace pxl {
class Odom {
    public:
        Odom(std::vector<std::unique_ptr<TrackingWheel>>& verticals,
             std::vector<std::unique_ptr<TrackingWheel>>& horizontals,
             std::vector<std::unique_ptr<TrackingWheel>>& drivetrain, std::vector<std::shared_ptr<pros::IMU>>& imu);
        void calibrate(bool calibrateIMUs = true);
        void update();
    private:
        std::vector<std::unique_ptr<TrackingWheel>> verticals;
        std::vector<std::unique_ptr<TrackingWheel>> horizontals;
        std::vector<std::unique_ptr<TrackingWheel>> drivetrain;
        std::vector<std::shared_ptr<pros::IMU>> imu;
        float lastAngle = 0;
        float calcDeltaTheta(std::vector<std::shared_ptr<pros::IMU>>& imu, bool update = true);
        float calcDeltaTheta(TrackingWheel& tracker1, TrackingWheel& tracker2);
        Pose pose = Pose(0, 0, 0);
};

} // namespace pxl