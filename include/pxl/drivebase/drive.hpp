#pragma once

#include <memory>
#include "pros/motors.hpp"
#include "pxl/aSync.hpp"
#include "pxl/drivebase/trackingwheel.hpp"
#include "pxl/pid.hpp"

namespace pxl {

class Drivetrain {
public:
  Drivetrain(pros::MotorGroup *leftMotors, pros::MotorGroup *rightMotors,
             float trackWidth, float wheelDiameter, float rpm)
      : leftMotors(leftMotors),
        rightMotors(rightMotors),
        trackWidth(trackWidth),
        wheelDiameter(wheelDiameter),
        rpm(rpm) {}

  // Add any necessary member functions here

  /**
   * @brief Method to queue a function to happen mid movement
   *
   * @param actionPoint the type of actionPoint to be used
   * @param input input function
   */
  pxl::Coord addOnAction(action_Point actionPoint, void input());

  void setConstants(PID &constants);

private:
  pros::MotorGroup *leftMotors;
  pros::MotorGroup *rightMotors;
  float trackWidth;
  float wheelDiameter;
  float rpm;
};

class ExtendedDrivetrain {
public:
  ExtendedDrivetrain(float verticalTrackWidth)
      : verticalTrackWidth(verticalTrackWidth) {}

private:
  float verticalTrackWidth;
};

struct OdomSensors {
  OdomSensors(TrackingWheel *vertical1, TrackingWheel *vertical2,
              TrackingWheel *horizontal1, TrackingWheel *horizontal2,
              pros::Imu *imu);
  TrackingWheel *vertical1;
  TrackingWheel *vertical2;
  TrackingWheel *horizontal1;
  TrackingWheel *horizontal2;
  pros::Imu *imu;
};

};  // namespace pxl
