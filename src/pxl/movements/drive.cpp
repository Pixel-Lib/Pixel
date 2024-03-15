#include "pxl/movements/drive.hpp"
#include <memory>
#include "pxl/drivebase/odom.hpp"

namespace pxl {
    Drive_::Drive_(Drivetrain& drivetrain, Odom& odom, SeekingController& linearController,
                   SeekingController& angularController)
        : drivetrain(drivetrain), odom(odom), linearController(linearController), angularController(angularController) {}
void Drive_::Drive(float target, float timeout, std::shared_ptr<Params> params, bool async) {
    mutex.take(TIMEOUT_MAX);
    if (async) {
        pros::Task task([&]() { Drive(target, timeout, params, false); });
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
        float minSpeed = params->minSpeed;
        float maxSpeed = params->maxSpeed;
        // if the real minSpeed and maxSpeed values are too high/low, the robot will ignore the slew when the clamping
        // happens
        std::pair<float, float> speeds =
            !isnanf((params->slew))
                ? (params->slew != 0 ? std::make_pair(slew(params->minSpeed, linearController.prevOut, params->slew),
                                                      slew(params->maxSpeed, linearController.prevOut, params->slew))
                                     : std::make_pair(params->minSpeed, params->maxSpeed))
                : (linearController.slew_ != 0
                       ? std::make_pair(slew(params->minSpeed, linearController.prevOut, linearController.slew_),
                                        slew(params->maxSpeed, linearController.prevOut, linearController.slew_))
                       : std::make_pair(params->minSpeed, params->maxSpeed));

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