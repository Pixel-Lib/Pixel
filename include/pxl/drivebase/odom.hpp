#pragma once

#include <memory>
#include <vector>
#include "pros/imu.hpp"
#include "pxl/drivebase/trackingwheel.hpp"


namespace pxl {
class Odom{
    public:

        Odom(std::vector<TrackingWheel>& verticals, 
                        std::vector<TrackingWheel>& horizontals,
                        std::vector<TrackingWheel>& drivetrain, 
                        std::shared_ptr<pros::IMU> imu);
        static void calibrate(bool calibrateIMUs = true) ;
        void update();
    private:
        std::vector<TrackingWheel> verticals;
        std::vector<TrackingWheel> horizontals;
        std::vector<TrackingWheel> drivetrain;
        std::shared_ptr<pros::IMU> imu;
};

} // namespace pxl