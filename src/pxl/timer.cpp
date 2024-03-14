#include "pxl/timer.hpp"

#include <sys/_stdint.h>

#include "pros/rtos.hpp"

namespace pxl {
// Constructor
Timer::Timer(uint32_t stop_time) { this->stop_time = (stop_time != -1) ? stop_time + pros::millis() : -1; }

// Destructor
Timer::~Timer() {
    // Nothing to do here
}

// Start the timer
void Timer::start() { start_time = pros::millis(); }

void Timer::stop() { this->start_time = -1; }

// Get the elapsed time since starting the timer
uint32_t Timer::get_elapsed_time() { return (start_time == -1) ? -1 : pros::millis() - start_time; }

uint32_t Timer::get_time_left() { return (stop_time == -1) ? -1 : stop_time - pros::millis(); }

bool Timer::isDone() { return Timer::get_time_left() <= 0; }

bool Timer::isRunning() { return start_time != -1; }
}  // namespace pxl