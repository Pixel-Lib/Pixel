#include <cmath>

namespace pxl {

class Coord {
    public:
        float x, y;

        Coord(float x, float y)
            : x(x),
              y(y) {}

        Coord operator+(const Coord& other) const;
        Coord operator-(const Coord& other) const;
        float operator*(const Coord& other) const;
        Coord operator*(const float& scalar) const;
        Coord operator/(const float& scalar) const;
        Coord lerp(Coord other, float t) const;
        float distance(const Coord& other) const;
        float angle(const Coord& other) const;
        Coord rotate(float angle) const;
};

} // namespace pxl
