#include "pros/imu.hpp"
#include <memory>
namespace pxl {
    class Drive_Sensors{
     public:

     protected:
     std::shared_ptr<pros::Imu> imu;

     //allow users to make either rotation or shaft encoder odom pods?

    };

}; // namespace pxl 