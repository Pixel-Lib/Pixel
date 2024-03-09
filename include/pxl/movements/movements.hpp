// Movement.hpp
//* #include "Drive.hpp"
//* #include "Turn.hpp"

namespace pxl {
class Movement {
    public:
        //* Drive drive;
        //* Turn turn;
        // Add other movement methods here
        friend class Drivebase; // Make Drivebase a friend of Movement
};
}