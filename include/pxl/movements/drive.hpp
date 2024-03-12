#pragma once
#include <memory>
#include "pxl/drivebase/drivebase.hpp"
#include "pxl/movements/drive.hpp"


namespace pxl {
    // class Drivebase;
class Drive_ : public Drivebase{
    private:
        // Drivebase drivebase;
        struct Params {
                float minSpeed = 0;
                float maxSpeed = 127;
                float slew = NAN;
        };
        pros::Mutex mutex;
        // get the current competition state. If this changes, the movement will stop
        uint8_t compstate = pros::competition::get_status();
        friend class Drivebase;

    public:
        void Drive(float target, float timeout, Params *params = {}, bool async = true);
};

}  // namespace pxl