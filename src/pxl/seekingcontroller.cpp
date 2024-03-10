#include "seekingcontroller.hpp"
#include "timer.hpp"

namespace pxl {

    seekingcontroller::seekingcontroller(PID pid, Regression regression, float globalTimeout) 
        : pid(pid), regression(regression), globalTimeout(globalTimeout), error(0) { timer.start();}

    float seekingcontroller::update(float error) {
        return pid.update(this->error = error);
    }

bool seekingcontroller::isExit(float error) {
    return (timer.get_elapsed_time() >= globalTimeout || timer.get_elapsed_time() >= regression.predict(this->error)) ? true : false;
}

}