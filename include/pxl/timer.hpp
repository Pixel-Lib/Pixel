#pragma once
#include <sys/types.h>

#include "pros/rtos.hpp"

namespace pxl {
/**
 * The Timer class provides a simple timer functionality for measuring and checking time
 */
class Timer {
    public:
        /**
         * @brief Constructor for the Timer class.
         * @param stop_time The stop time for the timer in milliseconds. Default value is -1.
         */
        Timer(uint32_t stop_time = -1);

        /**
         * @brief Destructor for the Timer class.
         */
        ~Timer();

        /**
         * The start() function sets the start time to the current time in
         * milliseconds using `pros/rtos`.
         */
        void start();

        /**
         * The stop function sets the start time of the timer to -1.
         */
        void stop();

        /**
         * `get_elapsed_time` calculates the elapsed time since the timer was
         * started in milliseconds.
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
         * time set in the `stop_time` variable. If the `stop_time` is not set
         * (i.e., it is -1), the function returns -1 to indicate that the stop time
         * is not set.
         */
        uint32_t get_time_left();

        /**
         * Returns true if the time left on the timer is 0.
         *
         * @return  a boolean value indicating whether the timer has finished or
         * not.
         */
        bool isDone();

        /**
         * Returns true if the timer is running.
         *
         * @return  a boolean value indicating whether the timer is running or
         * not.
         */
        bool isRunning();

    private:
        /**
         * The start time of the timer in milliseconds.
         */
        uint32_t start_time;

        /**
         * The stop time of the timer in milliseconds.
         */
        uint32_t stop_time;
};
}  // namespace pxl