#include "seekingcontroller.hpp"
#include "timer.hpp"

namespace pxl {

        SeekingController::SeekingController(PID pid,float slew_, Regression regression, float globalTimeout) 
            : pid(pid), slew_(slew_), regression(regression), globalTimeout(globalTimeout) { }

        float SeekingController::update(float error) {
            if (slew_!=0) {
                prevOut = pid.update(this->error);
                return slew(pid.update(this->error),prevOut,slew_);
            }
            prevOut = pid.update(this->error);
            return pid.update(this->error);

        }
    void SeekingController::timerStart(){
        timer.start();
    }

bool SeekingController::isExit(float error) {
    return (timer.get_elapsed_time()>=globalTimeout||timer.get_elapsed_time()>=regression.predict(this->error))?true:false;
}

}