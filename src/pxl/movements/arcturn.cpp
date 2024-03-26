#include "pxl/drivebase/drivebase.hpp"

namespace pxl {

void Drivebase::Arcturn(float target, float timeout, std::shared_ptr<arcturnParams> params, bool async){
    mutex.take(TIMEOUT_MAX);
    if (async) {
        pros::Task task([&]() { Arcturn(target, timeout, params, false); });
        pros::delay(10);
        return;
    }
}

}