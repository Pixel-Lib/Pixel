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

//     while (!localTimeout.isDone() || !linearController.getExit(error)) {
//         float currX = this->odom.getPose().x;

// }
}
}