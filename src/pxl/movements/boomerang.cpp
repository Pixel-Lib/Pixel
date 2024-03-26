#include "pxl/drivebase/drivebase.hpp"
#include "pxl/parametrics/coord.hpp"

namespace pxl {
bool Drivebase::SemicircleExit(pxl::Pose pose, pxl::Coord point, float radius) {
    // Calculate the distance from the point to the pose
    float dx = point.x - pose.x;
    float dy = point.y - pose.y;
    float distance = std::sqrt(dx * dx + dy * dy);

    // Check if the point is within the circle
    if (distance > radius) { return false; }

    // Calculate the angle from the pose to the point
    float angle = std::atan2(dy, dx);

    // Normalize the angles to the range [-pi, pi]
    pose.theta = std::fmod(pose.theta, 2 * M_PI);
    if (pose.theta < 0) { pose.theta += 2 * M_PI; }
    angle = std::fmod(angle, 2 * M_PI);
    if (angle < 0) { angle += 2 * M_PI; }

    // Check if the point is within the semicircle
    return std::abs(pose.theta - angle) <= M_PI;
}
void Drivebase::Boomerang(float x, float y, float theta, float timeout,
                          std::shared_ptr<boomerangParams> boomerangParams, bool async) {
    mutex.take(TIMEOUT_MAX);
    if (async) {
        pros::Task task([&]() { Boomerang(x, y, theta, timeout, boomerangParams, false); });
        pros::delay(10);
        return;
    }

    Pose targetPose = Pose(x, y, degToRad(theta));

    float linearError;
    float angularError;

    bool carrotSettled = false;
    Pose previousCarrot = Pose();
    const float carrotSettleThreshold = 0.01;

    //*GLEAD*//
    float distance = this->odom.getPose().distance(targetPose);
    const Coord inCarrot = Coord(targetPose.x - distance * cos(theta) * boomerangParams->dlead,
                                 targetPose.y - distance * sin(theta) * boomerangParams->dlead);

    // start the timeout
    Timer localTimeout(timeout);
    localTimeout.start();
    linearController.timerStart();
    angularController.timerStart();

    Coord carrot;
    while (!localTimeout.isDone() || !linearController.getExit(linearError) && !angularController.getExit(angularError)
           || SemicircleExit(carrot, odom.getPose(), previousCarrot.distance(carrot)) / 0.01) {

        if (!carrotSettled) {
            // calculate the carrot
            distance = this->odom.getPose().distance(targetPose);
            carrot = Coord(inCarrot.x + (carrot.x - inCarrot.x) * (1 - boomerangParams->glead),
                           inCarrot.y + (carrot.y - inCarrot.y) * (1 - boomerangParams->glead));
        } else {
            carrot = Coord(targetPose.x, targetPose.y);
        }

        if (previousCarrot.distance(carrot) < carrotSettleThreshold) { carrotSettled = true; }
        previousCarrot = carrot;

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

        // if the carrot has settled, reduce the linear output by the cosine of the angular error
        if (carrotSettled) linearOutput *= std::cos(degToRad(angularError));

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