#pragma once

// #include "drive.hpp"
#include "pxl/drivebase/drivebase.hpp"
namespace pxl {
class Movements {
        Movements(Drivetrain &drivetrain, Odom &odom, SeekingController &linearController,
                  SeekingController &angularController);
        SeekingController &linearController;
        SeekingController &angularController;
        Drivetrain &drivetrain;
        Odom &odom;

    public:
        friend class Drive_;
};
}  // namespace pxl