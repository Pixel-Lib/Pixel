#include "pxl/drivebase/drive.hpp"
#include "pxl/drivebase/trackingwheel.hpp"
#include "pxl/util.hpp"
#include "pxl/drivebase/odom.hpp"
#include <vector>
#include <iostream>
namespace pxl{
    // static void calibrate(bool calibrateIMUs) {
    //     std::vector<TrackingWheel> newVerticals = {};
    //     std::vector<TrackingWheel> newHorizontals = {};
    //     std::vector<TrackingWheel> newDrivetrain = {};
    //     std::vector<std::shared_ptr<imu>> newIMUs = {};

    //     // calibrate vertical tracking wheels
    //     for (auto it = verticals.begin(); it != verticals.end(); it++) {
    //         if (it->reset()) {
    //             std::cout << "Vertical tracker at offset " << it->getOffset() << " failed calibration!" << std::endl;
    //         } else newVerticals.push_back(*it);
    //     }

    //     // calibrate horizontal tracking wheels
    //     for (auto it = horizontals.begin(); it != horizontals.end(); it++) {
    //         if (it->reset()) {
    //             std::cout << "Horizontal tracker at offset " << it->getOffset() << " failed calibration!" << std::endl;
    //         } else newHorizontals.push_back(*it);
    //     }

    //     // calibrate drivetrain motors
    //     for (auto it = drivetrain.begin(); it != drivetrain.end(); it++) {
    //         if (it->reset()) {
    //             if (pxl::sgn(it->getOffset() == 1)) std::cout << "Left drivetrain motor failed to calibrate!" << std::endl;
    //             else std::cout << "Right drivetrain motor failed to calibrate!" << std::endl;
    //         } else newDrivetrain.push_back(*it);
    //     }

    //     if (!calibrateIMUs) return; // return if we don't need to calibrate IMUs
    //     // calibrate IMUs
    //     for (auto& it : IMUs) it->calibrate();
    //     Timer timer(3000); // try calibrating IMUs for 3000 ms
    //     while (!timer.isDone()) {
    //         for (auto& IMU : IMUs) { // continuously calibrate in case of failure
    //             if (!IMU->isCalibrating() && !IMU->isCalibrated()) IMU->calibrate();
    //         }
    //         pros::delay(10);
    //     }

    //     for (auto it = IMUs.begin(); it != IMUs.end(); it++) {
    //         if (!(**it).isCalibrated()) {
    //             std::cout << "IMU on port " << (**it).getPort() << " failed to calibrate! Removing..." << std::endl;
    //         } else {
    //             newIMUs.push_back(*it);
    //         }
    //     }

    //     verticals = newVerticals;
    //     horizontals = newHorizontals;
    //     drivetrain = newDrivetrain;
    //     IMUs = newIMUs;
    // }


float calcDeltaTheta(const float distanceDelta1, const float offset1, const float distanceDelta2, const float offset2) {
    const float numerator = distanceDelta1 - distanceDelta2;
    const float denominator = offset1 - offset2;
    if (denominator == 0) {
        // Handle division by zero error.
        return 0.0f;
    }
    return numerator / denominator;
}


void update(){

}
} // namespace pxl