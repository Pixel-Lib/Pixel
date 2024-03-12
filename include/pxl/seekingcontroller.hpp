#pragma once
#include "pxl/pid.hpp"
#include "pxl/timer.hpp"
#include "pxl/parametrics/regression.hpp"
#include "pxl/util.hpp"

namespace pxl {
class SeekingController {
public:
    SeekingController(PID pid, float slew_, Regression regression, float globalTimeout);

    float update(float error, bool slew = true);
void timerStart();
    bool getExit(float error);
private:
PID pid;
Regression regression;
float globalTimeout;
float error;
float prevOut = 0;
float slew_;
pxl::Timer timer;
friend class Drive_;
};

}
