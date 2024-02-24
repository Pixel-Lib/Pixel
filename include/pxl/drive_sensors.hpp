#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include <memory>
namespace pxl {
    class Drive_Sensors{
     public:

     Drive_Sensors() {}


     std::shared_ptr<pros::Imu> imu;
     std::shared_ptr<pros::Rotation> horizontalTracker1;
     std::shared_ptr<pros::Rotation> verticalTracker1;
     std::shared_ptr<pros::Rotation> horizontalTracker2;
     std::shared_ptr<pros::Rotation> verticalTracker2;

     std::shared_ptr<pros::ADIEncoder> horizontalTracker1_E;
     std::shared_ptr<pros::ADIEncoder> verticalTracker1_E;
     std::shared_ptr<pros::ADIEncoder> horizontalTracker2_E;
     std::shared_ptr<pros::ADIEncoder> verticalTracker2_E;

     std::shared_ptr<pros::Distance> distanceSensor;

     std::shared_ptr<pros::Optical> opticalSensor;



     //allow users to make either rotation or shaft encoder odom pods?

    };

}; // namespace pxl 