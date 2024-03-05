#include "pxl/drivebase/drive.hpp"

namespace pxl {
// pxl::OdomSensors::sensors(TrackingWheel *vertical1,
//                             TrackingWheel *vertical2,
//                             TrackingWheel *horizontal1,
//                             TrackingWheel *horizontal2, pros::Imu *imu)
//   : vertical1(vertical1),
//     vertical2(vertical2),
//     horizontal1(horizontal1),
//     horizontal2(horizontal2),
//     imu(imu) {}
pxl::Drivetrain::Drivetrain(pros::MotorGroup *leftMotors, pros::MotorGroup *rightMotors, float trackWidth,
                            float wheelDiameter, float rpm)
    : leftMotors(leftMotors),
      rightMotors(rightMotors),
      trackWidth(trackWidth),
      wheelDiameter(wheelDiameter),
      rpm(rpm) {}
pxl::Drivebase::Drivebase(Drivetrain drivetrain, OdomSensors sensors) : drivetrain(drivetrain), sensors(sensors) {}
void pxl::Drivebase::callibrate(bool callibrateIMU) {
    int attempt = 1;
    bool calibrated = false;
    // calibrate inertial, and if calibration fails, then repeat 5 times or until successful
    while (attempt <= 5 && !isDriverControl()) {
        sensors.imu->reset();
        // wait until IMU is calibrated
        do pros::delay(10);
        while (sensors.imu->get_status() != 0xFF && sensors.imu->is_calibrating() && !isDriverControl());
        // exit if imu has been calibrated
        if (!isnanf(sensors.imu->get_heading()) && !isinf(sensors.imu->get_heading())) {
            calibrated = true;
            break;
        }
        // indicate error
        pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, "---");
        std::cerr << "IMU failed to calibrate! Attempt #" << attempt << std::endl;
        attempt++;
    }
    // check if its driver control through the comp switch
    if (isDriverControl() && !calibrated) {
        sensors.imu = nullptr;
        std::cerr
            << "Driver control started, abandoning IMU calibration, defaulting to tracking wheels / motor encoders"
            << std::endl;
    }
    // check if calibration attempts were successful
    if (attempt > 5) {
        sensors.imu = nullptr;
        std::cerr << ("IMU calibration failed, defaulting to tracking wheels / motor encoders") << std::endl;
    }
}
bool Drivebase::isDriverControl() {
    return pros::competition::is_connected() && !pros::competition::is_autonomous()
           && !pros::competition::is_disabled();
}

}  // namespace pxl