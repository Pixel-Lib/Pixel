// will be used for tracking object and return sensed when found or failed

#include <limits>

#include "pxl/aSync.hpp"

namespace pxl {
enum sensed { SENSED = 1, SENSING_FAILED = std::numeric_limits<int>::max() };

class ObjectTracker {};

};  // namespace pxl
