#include "pxl/movements/drive.hpp"

#include <sys/_stdint.h>

#include "pros/misc.hpp"
#include "pxl/drivebase/drivebase.hpp"
#include "pxl/drivebase/odom.hpp"
#include "pxl/parametrics/coord.hpp"
#include "pxl/parametrics/pose.hpp"
namespace pxl {
void Drive_::Drive(float target, float timeout, Params *params, bool async) {
    if (async) {
        pros::Task task([&]() { Drive(target, timeout, params, true); });
        // end current motion just in case
        pros::delay(10);
        return;
    }
    Pose targetPose = drivebase.odom.getPose();
    targetPose += targetPose.rotate(drivebase.odom.getPose().theta) * target;
}

}  // namespace pxl