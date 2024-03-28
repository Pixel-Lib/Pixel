#include "pxl/drivebase/drivebase.hpp"

namespace pxl {
void Drivebase::turnToPoint(float x, float y, float timeout, turnToPointParams turnParams, bool async) {
    mutex.take(TIMEOUT_MAX);
    if (async) {
        pros::Task task([&]() { turnToPoint(x, y, timeout, turnParams, false); });
        pros::delay(10);
        return;
    }
    const Coord target = Coord(x, y);
    float       angularError;
    // start the timeout
    Timer localTimeout(timeout);
    localTimeout.start();
    angularController.timerStart();
    while (!localTimeout.isDone() || !angularController.getExit(angularError)) {
        // convert angular error to degrees for consistency
        angularError = wrapTo180(radToDeg(this->odom.getPose().angle(target)));
        // calculate the raw angular output from the PID controller
        float angularOutput = this->angularController.update(angularError);
        // clamp the output
        float minSpeed = turnParams.minSpeed;
        float maxSpeed = turnParams.maxSpeed;
        // if the real minSpeed and maxSpeed values are too high/low, the robot will ignore the slew when the clamping
        // happens
        std::pair<float, float> speeds = slewSpeedLimits(turnParams, angularController);
        minSpeed = speeds.first;
        maxSpeed = speeds.second;
        // clamp the output to the min and max speed
        angularOutput = std::clamp(angularOutput, minSpeed, maxSpeed);
        // if the error is negative, the robot should turn in the opposite direction
        angularOutput = (pxl::sgn(angularOutput) != pxl::sgn(angularError)) ? -angularOutput : angularOutput;
        // apply the output to the motors
        drivetrain.leftMotors->move(-angularOutput);
        drivetrain.rightMotors->move(angularOutput);
        pros::delay(10);
    }
    // stop the motors
    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
}
}  // namespace pxl