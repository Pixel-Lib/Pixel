#pragma once
#include <memory>

#include "pxl/drivebase/drivebase.hpp"

namespace pxl {
// class Drivebase;
class Drive_ : public Drivebase {
    private:
        // Drivebase drivebase;
        pros::Mutex mutex;
        // get the current competition state. If this changes, the movement will stop
        uint8_t compstate = pros::competition::get_status();
        friend class Drivebase;

    public:
        struct Params {
                float minSpeed = 0;
                float maxSpeed = 127;
                float slew = NAN;
        };
        static std::shared_ptr<Params> defaultParams() { return std::make_shared<Params>(); }
        void Drive(float target, float timeout, std::shared_ptr<Params> params = defaultParams(), bool async = true);
};

}  // namespace pxl