#pragma once

#include <memory>
#include <vector>
#include <numeric>
#include "math.h"
#include "pxl/util.hpp"
#include "pros/imu.hpp"
#include "pxl/drivebase/trackingwheel.hpp"
#include "pxl/drivebase/drive.hpp"

namespace pxl {
class Odom {
    public:
        Odom(std::vector<std::unique_ptr<TrackingWheel>>& verticals,
             std::vector<std::unique_ptr<TrackingWheel>>& horizontals,
             std::vector<std::unique_ptr<TrackingWheel>>& drivetrain, std::vector<std::shared_ptr<pros::IMU>>& imu);
        void calibrate(bool calibrateIMUs = true);
        float getRotationDelta(bool update = true);
        void update();
    private:
        std::vector<std::unique_ptr<TrackingWheel>> verticals;
        std::vector<std::unique_ptr<TrackingWheel>> horizontals;
        std::vector<std::unique_ptr<TrackingWheel>> drivetrain;
        std::vector<std::shared_ptr<pros::IMU>> imu;
        float lastAngle = 0;
};

} // namespace pxl