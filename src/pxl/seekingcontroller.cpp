#include "seekingcontroller.hpp"
#include "timer.hpp"

namespace pxl {

        SeekingController::SeekingController(PID pid,float slew_, Regression regression, float globalTimeout) 
            : pid(pid), slew_(slew_), regression(regression), globalTimeout(globalTimeout) { }

float SeekingController::update(float error) {
    prevOut = pid.update(this->error);
    return (slew_ != 0) ? slew(pid.update(this->error), prevOut, slew_) : pid.update(this->error);
}
    void SeekingController::timerStart(){
        timer.start();
    }

bool SeekingController::getExit(float error) {
    return (timer.get_elapsed_time()>=globalTimeout||timer.get_elapsed_time()>=regression.predict(this->error))?true:false;
}

}