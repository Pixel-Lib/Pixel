#include <chrono>
#include <cmath>
#include "pxl/util.hpp"

namespace pxl {
class PID {
public:
  // Constructor
  PID(float kP, float kI, float kD) : kP(kP), kI(kI), kD(kD) {}

  // Member function to update PID
  float update(const float error) {
    // Calculate time difference (dt)
    auto currentTime = std::chrono::system_clock::now();
    std::chrono::duration<float> dt = currentTime - prevTime;
    prevTime = currentTime;

    // Calculate integral
    integral += error * dt.count();
    if (pxl::sgn(error) != pxl::sgn(prevError))
      integral = 0;

    // Calculate derivative
    const float derivative = (error - prevError) / dt.count();
    prevError = error;

    // Calculate output
    return error * kP + integral * kI + derivative * kD;
  }

  // Member function to reset PID
  void reset() {
    integral = 0;
    prevError = 0;
  }

protected:
  // Member variables
  float kP;
  float kI;
  float kD;
  float integral = 0;
  float prevError = 0;

  // Member functions can also be added here!
private:
  std::chrono::time_point<std::chrono::system_clock> prevTime;
};
}  // namespace pxl
