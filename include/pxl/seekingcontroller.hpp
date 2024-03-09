#pragma once
#include "pxl/pid.hpp"
#include "pxl/timer.hpp"
#include "pxl/parametrics/regression.hpp"

namespace pxl {
class seekingcontroller {
public:
    seekingcontroller(PID pid, Regression regression, float globalTimeout);

    float update(float error);
private:
PID pid;
Regression regression;
float globalTimeout;
float error;
};

}
