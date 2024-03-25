#include "pxl/drivebase/drivebase.hpp"

namespace pxl {
void Drivebase::Boomerang(float x, float y, float theta, float dlead, float timeout,
                          std::shared_ptr<boomerangParams> boomerangParams, bool async) {
    mutex.take(TIMEOUT_MAX);
    if (async) {
        pros::Task task([&]() { Boomerang(x, y, theta, timeout, dlead, boomerangParams, false); });
        pros::delay(10);
        return;
    }

    Pose targetPose = Pose(x, y, degToRad(theta));

    float linearError;
    float angularError;

    Timer localTimeout(timeout);
    localTimeout.start();
    linearController.timerStart();
    angularController.timerStart();

    while (!localTimeout.isDone()
           || !linearController.getExit(linearError) && !angularController.getExit(angularError)) {

        float distance = this->odom.getPose().distance(targetPose);
        Coord carrot(targetPose.x - distance * cos(theta) * dlead, targetPose.y - distance * sin(theta) * dlead);

        linearError = this->odom.getPose().distance(carrot);
        angularError = wrapTo180(radToDeg(this->odom.getPose().angle(carrot)));

        // calculate the raw linear and angular output from the PID controllers
        float linearOutput = this->linearController.update(linearError);
        float angularOutput = this->angularController.update(angularError);

        // clamp the output
        float minSpeed = boomerangParams->minSpeed;
        float maxSpeed = boomerangParams->maxSpeed;
        // if the real minSpeed and maxSpeed values are too high/low, the robot will ignore the slew when the clamping
        // happens
        std::pair<float, float> speeds = slewSpeedLimits(boomerangParams, linearController);

        minSpeed = speeds.first;
        maxSpeed = speeds.second;

        // clamp the output to the min and max speed
        linearOutput = std::clamp(linearOutput, minSpeed, maxSpeed);

        // if the error is negative, the robot should move backwards
        linearOutput = (pxl::sgn(linearOutput) != pxl::sgn(linearError)) ? -linearOutput : linearOutput;

        //* Motion Optomization *//

        // priotize angular movement
        float overturn = fabs(angularOutput) + fabs(linearOutput) - boomerangParams->maxSpeed;
        if (overturn > 0) linearOutput -= linearOutput > 0 ? overturn : -overturn;

        // calculate and normalize the left/right speeds
        std::pair<float, float> normalized = normalize(linearOutput, angularOutput, maxSpeed);

        drivetrain.leftMotors->move(normalized.first);
        drivetrain.rightMotors->move(normalized.second);

        pros::delay(10);
    }
    // stop the motors
    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
}
}  // namespace pxl