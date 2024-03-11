#include "pxl/movements/drive.hpp"

#include <sys/_stdint.h>

#include "pros/misc.hpp"
#include "pxl/drivebase/drive.hpp"
#include "pxl/drivebase/odom.hpp"
#include "pxl/parametrics/pose.hpp"
// TODO drivebase.drivetrain.leftMotors->move(127);
namespace pxl {
void Drive_::Drive(float target, float timeout, Params *params, bool async) {
    if (async) {
        pros::Task task([&]() { Drive(target, timeout, params, true); });
        // end current motion just in case
        pros::delay(10);
        return;
    }
    // Pose targetPose = ;
    drivebase.drivetrain.leftMotors->get_positions();
}

}  // namespace pxl