#pragma once
// #include "pxl/drivebase/api.hpp"
#include "drive.hpp"
#include "pxl/movements/drive.hpp"

namespace pxl {
class Movement {
    public:
        
        // Add other movement methods here
        friend class Drive_; // Make Drive a friend of Movement

    friend class Drivebase; // Make Drivebase a friend of Movement
};
}