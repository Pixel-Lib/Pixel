#pragma once
#include <sys/types.h>

#include "pros/rtos.hpp"

namespace pxl {
class Timer {
  public:
  // Constructor
  Timer(uint32_t stop_time = -1);

  // Destructor
  ~Timer();

  /**
   * The start() function sets the start time to the current time in
   * milliseconds using `pros/rtos`.
   */
  void start();

  /**
   * The stop function sets the start time of the timer to -1.
   *
   * @param start_time Optional` parameter to define the start time of the
   * timer.
   */
  void stop(uint32_t start_time = -1);

  /**
   * `get_elapsed_time` calculates the elapsed time since the timer was started
   * in milliseconds.
   *
   * @return the elapsed time in milliseconds since the timer
   * was started. If the timer has not been started (when `start_time` is -1),
   * it returns -1 to indicate that the timer is not running.
   */
  uint32_t get_elapsed_time();

  /**
   * `get_time_left` calculates the time left based on the stop time set.
   *
   * @return the time left in milliseconds until the stop
   * time set in the `stop_time` variable. If the `stop_time` is not set (i.e.,
   * it is -1), the function returns -1 to indicate that the stop time is not
   * set.
   */
  uint32_t get_time_left();
  /**
   * Returns true if the time left on the timer is 0.
   *
   * @return  a boolean value indicating whether the timer has finished or
   * not.
   */
  bool isDone();

  private:
  uint32_t start_time;
  uint32_t stop_time;
};
}  // namespace pxl