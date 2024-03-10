#include "seekingcontroller.hpp"
#include "timer.hpp"

namespace pxl {

    SeekingController::SeekingController(PID pid, Regression regression, float globalTimeout) 
        : pid(pid), regression(regression), globalTimeout(globalTimeout) { timer.start();}

    float SeekingController::update(float error) {
        return pid.update(this->error = error);
    }

bool SeekingController::isExit(float error) {
    return (timer.get_elapsed_time() >= globalTimeout || timer.get_elapsed_time() >= regression.predict(this->error)) ? true : false;
}

}