#include <memory>
#include "main.h"
#include "drive_sensors.hpp"
#include "pros/motors.hpp"

namespace pxl {

 class Drive{
 public:

 /**
 * @brief Constructs a drivetrain 
 *
 * @param leftMotors   shared smart pointer to the left side motors
 * @param rightMotors  shared smart pointer to the right side motors
 * @param cartType     the cartridge type being used in the drive motors
 * @param inputGear    the teeth count of the input gears 
 * @param outputGear   the teeth count of the out2put gears 
 * @param sensors      Drive_senors object to consume for the drivetrain
 */

 Drive(std::shared_ptr<pros::MotorGroup> leftMotors, std::shared_ptr<pros::MotorGroup> rightMotors,
       pros::motor_gearset_e_t cartType, float inputGear, float outputGear, Drive_Sensors sensors) :
       leftMotors(leftMotors), rightMotors(rightMotors), cartType(cartType),
       inputGear(inputGear), outputGear(outputGear), sensors(sensors) {};


 
 
 protected:
 std::shared_ptr<pros::MotorGroup> leftMotors;
 std::shared_ptr<pros::MotorGroup> rightMotors;
 pros::motor_gearset_e_t cartType;
 float inputGear;
 float outputGear;
 Drive_Sensors sensors;


 private:


 };

}; // namespace pxl
