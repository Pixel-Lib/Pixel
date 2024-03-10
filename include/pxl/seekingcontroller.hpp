#pragma once
#include "pxl/pid.hpp"
#include "pxl/timer.hpp"
#include "pxl/parametrics/regression.hpp"

namespace pxl {
class SeekingController {
public:
    SeekingController(PID pid, Regression regression, float globalTimeout);

    float update(float error);

    bool isExit(float error);
private:
PID pid;
Regression regression;
float globalTimeout;
float error;
pxl::Timer timer;
};

}
