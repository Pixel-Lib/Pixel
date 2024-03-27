#include "pxl/drivebase/drivebase.hpp"

namespace pxl {

void Drivebase::eulerTurn(float target, float rate, float timeout, eulerTurnParams params, bool async){
    mutex.take(TIMEOUT_MAX);
    if (async) {
        pros::Task task([&]() { eulerTurn(target, rate, timeout, params, false); });
        pros::delay(10);
        return;
    }

}

}  // namespace pxl