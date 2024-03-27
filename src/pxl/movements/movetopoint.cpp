#include "pxl/drivebase/drivebase.hpp"
#include "pxl/util.hpp"

namespace pxl {
void Drivebase::moveToPoint(float x, float y, float timeout, moveToPointParams params, bool async) {
    mutex.take(TIMEOUT_MAX);
    if (async) {
        pros::Task task([&]() { moveToPoint(x, y, timeout, params, false); });
        pros::delay(10);
        return;
    }
    const Coord target = Coord(x, y);
    // start the timeout
    Timer localTimeout(timeout);
    localTimeout.start();
    linearController.timerStart();
    angularController.timerStart();

    float linearError;
    float angularError;

    while (!localTimeout.isDone()
           || !linearController.getExit(linearError) && !angularController.getExit(angularError)){
        linearError = this->odom.getPose().distance(target);
        float currHeading = this->odom.getPose().theta;
        float targetHeading = absoluteAngleToPoint(odom.getPose(), target);

        float angularError = radToDeg(angleError(targetHeading, currHeading, false));

        float cre = fabs(angularError) > 90 ? 0.1 : std::cos(degToRad(angularError));

        float angularOutput = angularController.update(angularError);
        float linearOutput = cre * linearController.update(linearError);

float rVel = (linearOutput - (fabs(angularOutput) * params.rotationBias)) + angularOutput;
float lVel = (linearOutput - (fabs(angularOutput) * params.rotationBias)) - angularOutput;


        }
}
}