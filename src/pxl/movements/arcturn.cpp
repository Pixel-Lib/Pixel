#include "pros/rtos.hpp"
#include "pxl/drivebase/drivebase.hpp"
#include "pxl/seekingcontroller.hpp"
#include "pxl/util.hpp"

namespace pxl {

void Drivebase::Arcturn(float target, float timeout, std::shared_ptr<arcturnParams> params, bool async) {
    mutex.take(TIMEOUT_MAX);
    if (async) {
        pros::Task task([&]() { Arcturn(target, timeout, params, false); });
        pros::delay(10);
        return;
    }

    float curr = odom.getPose().theta;
    float theta = dirToSpin(target, curr);

    float sl = theta * (params->radius + this->extendedDrivetrain.verticalTrackWidth);
    float sr = theta * (params->radius - this->extendedDrivetrain.verticalTrackWidth);
    float ratio = sl / sr;

    // start the timeout
    Timer localTimeout(timeout);
    localTimeout.start();
    linearController.timerStart();

    while (!localTimeout.isDone() || !linearController.getExit(error)) {

        float error = angleError(target, curr, false);
        float vel = linearController.update(error);

        float minSpeed = params->minSpeed;
        float maxSpeed = params->maxSpeed;

        std::pair<float, float> speeds = slewSpeedLimits(params, linearController);
        minSpeed = speeds.first;
        maxSpeed = speeds.second;

        vel = std::max(std::abs(vel), minSpeed) * pxl::sgn(vel);

        float rvel = (2 * vel) / (ratio + 1);
        rvel = std::abs(rvel) >= maxSpeed ? (maxSpeed * pxl::sgn(rvel)) : rvel;

        float lvel = ratio * rvel;

        std::pair<float, float> normalized = normalize(lvel, rvel, maxSpeed, true);
        if (params->dir == 1) {
            drivetrain.leftMotors->move(normalized.first);
            drivetrain.rightMotors->move(normalized.second);
        } else {
            drivetrain.leftMotors->move(-normalized.second);
            drivetrain.rightMotors->move(-normalized.first);
        }

        pros::delay(10);
    }
        drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
}

}  // namespace pxl