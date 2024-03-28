#include "pxl/drivebase/drivebase.hpp"

namespace pxl {

void Drivebase::eulerTurn(float target, float rate, float timeout, eulerTurnParams params, bool async) {
    mutex.take(TIMEOUT_MAX);
    if (async) {
        pros::Task task([&]() { eulerTurn(target, rate, timeout, params, false); });
        pros::delay(10);
        return;
    }

    float curvature = 0.0f;

    // start the timeout
    Timer localTimeout(timeout);
    localTimeout.start();
    linearController.timerStart();

    while (!localTimeout.isDone() || !linearController.getExit(error)) {
        curvature += rate;

        float curr = this->odom.getPose().theta;

        float sl = target * (1 / curvature + this->drivetrain.trackWidth);
        float sr = target * (1 / curvature - this->drivetrain.trackWidth);
        float ratio = sl / sr;

        float error = angleError(target, curr, false);

        float vel = this->linearController.update(error);
        float minSpeed = params.minSpeed;
        float maxSpeed = params.maxSpeed;

        std::pair<float, float> speeds = this->slewSpeedLimits(params, this->linearController);
        minSpeed = speeds.first;
        maxSpeed = speeds.second;

        vel = std::fabs(vel) >= maxSpeed ? (maxSpeed * pxl::sgn(vel)) : vel;
        float rvel = (2 * vel) / (ratio + 1);
        float lvel = ratio * rvel;

        std::pair<float, float> normalized = normalize(lvel, rvel, maxSpeed, true);

        if (params.forward) {
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