#include "pxl/drivebase/exit_conditions.hpp"

#include <cmath>

#include "pros/rtos.hpp"

namespace pxl {

void stand_still::setStandStill(movement_Type type, uint8_t maxCycles, float maxStep) {
    if (type == lateral) {
        // Deactivate standstill if maxcycles is 0
        SSActive = !(maxCycles == 0);
        SSMaxCount_lateral = maxCycles;
        maxStepDistance_t = maxStep;
    } else if (type == turn) {
        // Deactivate standstill if maxcycles is 0
        SSActive_t = !(maxCycles == 0);
        SSMaxCount_turn = maxCycles;
        maxStepTurn_d = maxStep;
    }
}

bool stand_still::updateStandstill(movement_Type type, bool &standStill, float error, float lastError,
                                   uint8_t &standStillCount) {
    if (type == lateral) {
        if (SSActive && fabs(lastError - error) <= maxStepDistance_t) {
            standStillCount++;
            if (standStillCount > SSMaxCount_lateral) { return standStill = true; }
        } else {
            standStillCount = 0;
        }
    }

    else if (type == turn) {
        if (SSActive_t && fabs(lastError - error) <= maxStepTurn_d) {
            standStillCount++;
            if (standStillCount > SSMaxCount_turn) { return standStill = true; }
        } else {
            standStillCount = 0;
        }
    }
}

void global_timeOuts::timeOut(float timeOut_S) {
    float startTime = pros::millis();
    const uint32_t endTime = pros::millis() + timeOut_S * 1000;

    do pros::delay(10);
    while (startTime < endTime);
}

}  // namespace pxl