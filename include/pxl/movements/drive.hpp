#pragma once
#include "pxl/aSync.hpp"
#include "pxl/drivebase/api.hpp"
#include "pxl/drivebase/drive.hpp"
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
    friend class Drivebase;
    public: 
    void Drive(float target, float timeout, Params* params, bool async=true);
    

    };
    
}