


#include "pxl/drivebase/drivebase.hpp"

namespace pxl {
void Drivebase::Drive(float target, float timeout, std::shared_ptr<driveParams> driveParams, bool async) {
    mutex.take(TIMEOUT_MAX);
    if (async) {
        pros::Task task([&]() { Drive(target, timeout, driveParams, false); });
        pros::delay(10);
        return;
    }
    // calculate the target as a coordinate for accuracy
    Pose targetPose = this->odom.getPose();
    targetPose += targetPose.rotate(odom.getPose().theta) * target;

    float linearError;
    float angularError;
    // start the timeout
    Timer localTimeout(timeout);
    localTimeout.start();
    linearController.timerStart();
    angularController.timerStart();
    while (!localTimeout.isDone()
           || !linearController.getExit(linearError) && !angularController.getExit(angularError)) {

        // calculate the linear error
        linearError = this->odom.getPose().distance(targetPose);

        // convert angular error to degrees for consistency
        angularError = wrapTo180(radToDeg(this->odom.getPose().angle(targetPose)));

        // calculate the raw linear and angular output from the PID controllers
        float linearOutput = this->linearController.update(linearError);
        float angularOutput = this->angularController.update(angularError);

        // clamp the output
        float minSpeed = driveParams->minSpeed;
        float maxSpeed = driveParams->maxSpeed;
        // if the real minSpeed and maxSpeed values are too high/low, the robot will ignore the slew when the clamping
        // happens
        std::pair<float, float> speeds = slewSpeedLimits(driveParams, linearController);

        minSpeed = speeds.first;
        maxSpeed = speeds.second;

        // clamp the output to the min and max speed
        linearOutput = std::clamp(linearOutput, minSpeed, maxSpeed);

        // if the error is negative, the robot should move backwards
        linearOutput = (pxl::sgn(linearOutput) != pxl::sgn(linearError)) ? -linearOutput : linearOutput;

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