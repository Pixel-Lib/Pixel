#include <memory>
#include "main.h"
#include "drive_sensors.hpp"
#include "pros/motors.hpp"
namespace pxl {
 class Drive : protected Drive_Sensors{
 public:
 //take in as a drive constructor param? or make users proprely define motors with correct cart?
 enum cartridge_Type{
    greenCart,
    blueCart,
    redCart
 };

 protected:
 std::shared_ptr<pros::MotorGroup> leftMotors;
 std::shared_ptr<pros::MotorGroup> rightMotors;
 cartridge_Type cartType;
 float inputGear;
 float outputGear;
 Drive_Sensors sensors;


 private:


 };

}; // namespace pxl
