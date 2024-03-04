#include "pxl/timer.hpp"
#include <sys/_stdint.h>
#include "pros/rtos.hpp"

namespace pxl {
// Constructor
Timer::Timer(uint32_t stop_time) : stop_time(stop_time) {
  if (stop_time != -1)
    stop_time += pros::millis();
}

// Destructor
Timer::~Timer() {
  // Nothing to do here
}

// Start the timer
void Timer::start() { start_time = pros::millis(); }

void Timer::stop(uint32_t start_time) { this->start_time = -1; }

// Get the elapsed time since starting the timer
uint32_t Timer::get_elapsed_time() {
  if (start_time == -1)
    return -1;  // Timer not started
  return pros::millis() - start_time;
}

uint32_t Timer::get_time_left() {
  if (stop_time == -1)
    return -1;  // stop time not set
  return pros::millis() - stop_time;
}

bool Timer::isDone() { return Timer::get_time_left() == 0; }
}  // namespace pxl