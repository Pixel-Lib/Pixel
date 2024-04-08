
#include "pxl/drivebase/drivebase.hpp"

namespace pxl {
bool Drivebase::SemicircleExit(pxl::Pose target, pxl::Coord curr, float radius) {
    // Calculate the distance from the robot to the target
    pxl::Coord diff = pxl::Coord(curr.x - target.x, curr.y - target.y);
    float distance = diff.distance(pxl::Coord(0, 0));

    // Check if the robot is within the circle
    if (distance > radius) { return false; }

    // Calculate the angle from the target to the robot
    float angle = std::atan2(diff.y, diff.x);

    // Normalize the angles to the range [-pi, pi]
    target.theta = pxl::wrapToPi(target.theta);
    angle = pxl::wrapToPi(angle);

    // Check if the robot is within the semicircle
    return std::fabs(target.theta - angle) <= M_PI;
}
void Drivebase::boomerang(float x, float y, float theta, float timeout, boomerangParams params, bool async) {
    mutex.take(TIMEOUT_MAX);
    if (async) {
        pros::Task task([&]() { boomerang(x, y, theta, timeout, params, false); });
        pros::delay(10);
        return;
    }

    Pose targetPose = Pose(x, y, degToRad(theta));
    if (!params.forward) { targetPose.theta = wrapToPi(targetPose.theta + M_PI); }

    float linearError;
    float angularError;

    // bool carrotSettled = false;
    std::pair<bool, bool> carrotSettled = std::make_pair(false, false);
    static const float carrotSettleThreshold = 0.01;

    //*GLEAD*//
    float distance = this->odom.getPose().distance(targetPose);
    const Coord inCarrot =
        Coord(targetPose.x - distance * cos(theta) * params.dlead, targetPose.y - distance * sin(theta) * params.dlead);
    Coord carrot = inCarrot;

    Coord previousCarrot = inCarrot;
    Pose prevPose = this->odom.getPose();

    // start the timeout
    Timer localTimeout(timeout);
    localTimeout.start();
    linearController.timerStart();
    angularController.timerStart();

    while (!localTimeout.isDone() || !linearController.getExit(linearError) && !angularController.getExit(angularError)
           || SemicircleExit(targetPose, odom.getPose(), prevPose.distance(odom.getPose())) / 0.01) {

        if (!carrotSettled.first) {
            // calculate the glead carrot
            distance = this->odom.getPose().distance(targetPose);
            carrot = Coord(inCarrot.x + (carrot.x - inCarrot.x) * (1 - params.glead),
                           inCarrot.y + (carrot.y - inCarrot.y) * (1 - params.glead));
        } else if (carrotSettled.first && !carrotSettled.second) {
            // calculate the carrot
            carrot = Coord(targetPose.x - distance * cos(theta) * params.dlead,
                           targetPose.y - distance * sin(theta) * params.dlead);
        } else {
            // switch to the target
            carrot = Coord(targetPose.x, targetPose.y);
        }

        if (SemicircleExit(Pose(inCarrot.x, inCarrot.y, targetPose.theta), odom.getPose(),
                           previousCarrot.distance(carrot) / 0.01 * drivetrain.trackWidth)) {
            carrotSettled.first = true;
        }
        if (previousCarrot.distance(carrot) < carrotSettleThreshold && carrotSettled.first) {
            carrotSettled.second = true;
        }
        previousCarrot = carrot;

        linearError = this->odom.getPose().distance(carrot);
        angularError = wrapTo180(radToDeg(this->odom.getPose().angle(carrot)));

        // calculate the raw linear and angular output from the PID controllers
        float linearOutput = this->linearController.update(linearError);
        float angularOutput = this->angularController.update(angularError);

        // clamp the output
        float minSpeed = params.minSpeed;
        float maxSpeed = params.maxSpeed;
        // if the real minSpeed and maxSpeed values are too high/low, the robot will ignore the slew when the clamping
        // happens
        std::pair<float, float> speeds = this->slewSpeedLimits(params, this->linearController);

        minSpeed = speeds.first;
        maxSpeed = speeds.second;

        // clamp the output to the min and max speed
        linearOutput = std::clamp(linearOutput, minSpeed, maxSpeed);
        angularOutput = std::clamp(angularOutput, minSpeed, maxSpeed);

        // if the error is negative, the robot should move backwards
        linearOutput = (pxl::sgn(linearOutput) != pxl::sgn(linearError)) ? -linearOutput : linearOutput;

        //* Motion Optomization *//

        // priotize angular movement
        float overturn = fabs(angularOutput) + fabs(linearOutput) - params.maxSpeed;
        if (overturn > 0) linearOutput -= linearOutput > 0 ? overturn : -overturn;

        // if the carrot has settled, reduce the linear output by the cosine of the angular error
        if (carrotSettled.second) linearOutput *= (std::cos(degToRad(angularError)) + params.minAccel);

        // if the robot is moving backwards, negate the linear output
        if (!params.forward) linearOutput = -linearOutput;

        // calculate and normalize the left/right speeds
        std::pair<float, float> normalized = normalize(linearOutput, angularOutput, maxSpeed);

        drivetrain.leftMotors->move(normalized.first);
        drivetrain.rightMotors->move(normalized.second);

        prevPose = this->odom.getPose();

        pros::delay(10);
    }
    // stop the motors
    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
}

}  // namespace pxl
