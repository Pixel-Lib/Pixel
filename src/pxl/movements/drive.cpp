#include "pxl/movements/drive.hpp"

#include <sys/_stdint.h>

#include <cmath>

#include "pros/misc.hpp"
#include "pros/rtos.h"
#include "pxl/drivebase/drivebase.hpp"
#include "pxl/drivebase/odom.hpp"
#include "pxl/parametrics/coord.hpp"
#include "pxl/parametrics/pose.hpp"
#include "pxl/util.hpp"
namespace pxl {
void Drive_::Drive(float target, float timeout, Params *params, bool async) {
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
        if (!isnanf((params->slew))) {
            // allow users to remove the slew rate for a specific movement
            if (params->slew == 0) {
                return;
            } else {
                minSpeed = slew(params->minSpeed, linearController.prevOut, params->slew);
                maxSpeed = slew(params->maxSpeed, linearController.prevOut, params->slew);
            }

        } else if (linearController.slew_ != 0) {
            minSpeed = slew(params->minSpeed, linearController.prevOut, linearController.slew_);
            maxSpeed = slew(params->maxSpeed, linearController.prevOut, linearController.slew_);
        }
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