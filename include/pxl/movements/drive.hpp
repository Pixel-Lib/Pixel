#pragma once
#include <memory>
#include <cmath>
#include "main.h"
#include "pxl/drivebase/drivebase.hpp"
#include "pxl/drivebase/odom.hpp"
// #include "pxl/drivebase/drivetrain.hpp"
#include "pxl/seekingcontroller.hpp"

namespace pxl {
class Drivebase;
class Drivetrain;
class Drive_ {
    private:
        // Drivebase drivebase;
        pros::Mutex mutex;
        // get the current competition state. If this changes, the movement will stop
        uint8_t compstate = pros::competition::get_status();
        friend class Drivebase;

    public:
    Drive_(pxl::Drivetrain& drivetrain, pxl::Odom& odom, pxl::SeekingController& linearController,
                          pxl::SeekingController& angularController);
        struct Params {
                float minSpeed = 0;
                float maxSpeed = 127;
                float slew = NAN;
        };
        static std::shared_ptr<Params> defaultParams() { return std::make_shared<Params>(); }
        void Drive(float target, float timeout, std::shared_ptr<Params> params = defaultParams(), bool async = true);

        // non-modifiable Drivebase derived

        pxl::SeekingController& linearController;
        pxl::SeekingController& angularController;
        pxl::Drivetrain& drivetrain;
        pxl::Odom& odom;
};

}  // namespace pxl