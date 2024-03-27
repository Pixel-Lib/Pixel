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
        }
}
}