#pragma once
#include "pxl/aSync.hpp"
#include "pxl/drivebase/api.hpp"
#include "pxl/drivebase/drivebase.hpp"
#include "pxl/pid.hpp"
namespace pxl {
class Drive_ {
        private:
    Drivebase drivebase;
    struct Params{
        float minSpeed = 0;
        float maxSpeed = 127;
        float slew = NAN;
    }; 
    Params* params;
    // get the current competition state. If this changes, the movement will stop
    uint8_t compstate = pros::competition::get_status();
    friend class Drivebase;
    public: 
    void Drive(float target, float timeout, Params* params, bool async=true);
    

    };
    
}