#include "pxl/drivebase/drivebase.hpp"

// #include "pxl/seekingcontroller.hpp"

namespace pxl {
pxl::OdomSensors::OdomSensors(TrackingWheel *vertical1, TrackingWheel *vertical2, TrackingWheel *horizontal1,
                              TrackingWheel *horizontal2, pros::Imu *imu)
    : vertical1(vertical1), vertical2(vertical2), horizontal1(horizontal1), horizontal2(horizontal2), imu(imu) {}
pxl::Drivetrain::Drivetrain(pros::MotorGroup *leftMotors, pros::MotorGroup *rightMotors, float trackWidth,
                            float wheelDiameter, float rpm)
    : leftMotors(leftMotors),
      rightMotors(rightMotors),
      trackWidth(trackWidth),
      wheelDiameter(wheelDiameter),
      rpm(rpm) {}
pxl::Drivebase::Drivebase(Drivetrain drivetrain, OdomSensors sensors, SeekingController linearController,
                          SeekingController angularController)
    : drivetrain(drivetrain),
      sensors(sensors),
      linearController(linearController),
      angularController(angularController) {}

void pxl::Drivebase::calibrateIMU(OdomSensors sensors) {
    for (int attempt = 1; attempt <= 5 && !isDriverControl(); attempt++) {
        sensors.imu->reset();
        do pros::delay(10);
        while (sensors.imu->get_status() != 0xFF && sensors.imu->is_calibrating() && !isDriverControl());

        if (!isnanf(sensors.imu->get_heading()) && !isinf(sensors.imu->get_heading())) { return; }

        pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, "---");
        std::cerr << "IMU failed to calibrate! Attempt #" << attempt << std::endl;
    }

    sensors.imu = nullptr;
    std::cerr
        << (isDriverControl()
                ? "Driver control started, abandoning IMU calibration, defaulting to tracking wheels / motor encoders"
                : "IMU calibration failed, defaulting to tracking wheels / motor encoders")
        << std::endl;
}
bool Drivebase::isDriverControl() {
    return pros::competition::is_connected() && !pros::competition::is_autonomous()
           && !pros::competition::is_disabled();
}
Odom Drivebase::setSensors(OdomSensors sensors) {
    std::vector<std::unique_ptr<TrackingWheel>> Verticals;
    std::vector<std::unique_ptr<TrackingWheel>> Horizontals;
    std::vector<std::unique_ptr<TrackingWheel>> drive;
    std::vector<std::shared_ptr<pros::IMU>> imu;
    auto pushIfNotNull = [](auto &vec, auto &sensor) {
        sensor != nullptr ? vec.push_back(std::make_unique<TrackingWheel>(std::move(*sensor))) : void();
    };

    pushIfNotNull(Verticals, sensors.vertical1);
    pushIfNotNull(Verticals, sensors.vertical2);
    pushIfNotNull(Horizontals, sensors.horizontal1);
    pushIfNotNull(Horizontals, sensors.horizontal2);

    drive.push_back(std::make_unique<TrackingWheel>(drivetrain.leftMotors, drivetrain.wheelDiameter,
                                                    -drivetrain.trackWidth / 2, drivetrain.rpm));
    drive.push_back(std::make_unique<TrackingWheel>(drivetrain.leftMotors, drivetrain.wheelDiameter,
                                                    drivetrain.trackWidth / 2, drivetrain.rpm));
    pxl::Odom odom(Verticals, Horizontals, drive, imu);
    return odom;
}
void Drivebase::calibrate(bool calibrateImu) {
    // calibrate the IMU if it exists and the user doesn't specify otherwise
    if (sensors.imu != nullptr && calibrateImu) calibrateIMU(sensors);
    // initialize odom
    sensors.vertical1 = sensors.vertical1 ? sensors.vertical1
                                          : new pxl::TrackingWheel(drivetrain.leftMotors, drivetrain.wheelDiameter,
                                                                   -(drivetrain.trackWidth / 2), drivetrain.rpm);
    sensors.vertical2 = sensors.vertical2 ? sensors.vertical2
                                          : new pxl::TrackingWheel(drivetrain.rightMotors, drivetrain.wheelDiameter,
                                                                   drivetrain.trackWidth / 2, drivetrain.rpm);
    sensors.vertical1->reset();
    sensors.vertical2->reset();

    if (sensors.horizontal1 != nullptr) sensors.horizontal1->reset();
    if (sensors.horizontal2 != nullptr) sensors.horizontal2->reset();

    this->odom = setSensors(sensors);
    this->odom.init();

    // rumble to controller to indicate success
    pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");
}

}  // namespace pxl