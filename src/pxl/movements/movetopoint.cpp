#include "pxl/drivebase/drivebase.hpp"

namespace pxl {
void Drivebase::moveToPoint(float x, float y, float timeout, moveToPointParams params, bool async) {
    mutex.take(TIMEOUT_MAX);
    if (async) {
        pros::Task task([&]() { moveToPoint(x, y, timeout, params, false); });
        pros::delay(10);
        return;
    }
    const Coord target = Coord(x, y);
    float       exitAngle = this->odom.getPose().angle(target);
    // start the timeout
    Timer localTimeout(timeout);
    localTimeout.start();
    linearController.timerStart();
    angularController.timerStart();

    float linearError;
    float angularError;

    while (!localTimeout.isDone() || !linearController.getExit(linearError) && !angularController.getExit(angularError)
           || SemicircleExit(pxl::Pose(target.x, target.y, exitAngle), odom.getPose(),
                             drivetrain.trackWidth * (1 - std::cos(degToRad(angularError))))) {
        exitAngle = std::atan2(target.y - odom.getPose().y, target.x - odom.getPose().x);

        linearError = this->odom.getPose().distance(target);
        float currHeading = this->odom.getPose().theta;
        float targetHeading = absoluteAngleToPoint(odom.getPose(), target);

        float angularError = radToDeg(angleError(targetHeading, currHeading, false));

        float cre = fabs(angularError) > 90 ? 0.1 : std::cos(degToRad(angularError));

        float angularOutput = angularController.update(angularError);
        float linearOutput = cre * linearController.update(linearError);

        float minSpeed = params.minSpeed;
        float maxSpeed = params.maxSpeed;

        std::pair<float, float> speeds = this->slewSpeedLimits(params, this->linearController);
        minSpeed = speeds.first;
        maxSpeed = speeds.second;

        linearOutput = std::clamp(linearOutput, minSpeed, maxSpeed) * pxl::sgn(linearError);
        angularOutput = std::clamp(angularOutput, minSpeed, maxSpeed) * pxl::sgn(angularError);

        float rVel = (linearOutput - (fabs(angularOutput) * params.rotationBias)) + angularOutput;
        float lVel = (linearOutput - (fabs(angularOutput) * params.rotationBias)) - angularOutput;

        std::pair<float, float> normalized = normalize(lVel, rVel, maxSpeed, true);

        drivetrain.leftMotors->move(normalized.first);
        drivetrain.rightMotors->move(normalized.second);

        pros::delay(10);
    }
    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
}
}  // namespace pxl