#include "pxl/movements/movements.hpp"

namespace pxl {
Movements::Movements(Drivetrain& drivetrain, Odom& odom, SeekingController& linearController,
                      SeekingController& angularController)
    : drivetrain(drivetrain), odom(odom), linearController(linearController), angularController(angularController) {}

}