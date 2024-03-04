#include <cmath>
#include "pxl/parametrics/coord.hpp"

namespace pxl {

Coord Coord::operator+(const Coord& other) const { return Coord(x + other.x, y + other.y); }

Coord Coord::operator-(const Coord& other) const { return Coord(x - other.x, y - other.y); }

float Coord::operator*(const Coord& other) const { return this->x * other.x + this->y * other.y; }

Coord Coord::operator*(const float& scalar) const { return Coord(this->x * scalar, this->y * scalar); }

Coord Coord::operator/(const float& scalar) const { return Coord(x / scalar, y / scalar); }

Coord& Coord::operator+=(const Coord& other) {
    this->x += other.x;
    this->y += other.y;
    return *this;
}

Coord Coord::lerp(Coord other, float t) const {
    return Coord(this->x + (other.x - this->x) * t, this->y + (other.y - this->y) * t);
}

float Coord::distance(const Coord& other) const { return std::hypot(this->x - other.x, this->y - other.y); }

float Coord::angle(const Coord& other) const { return std::atan2(other.y - this->y, other.x - this->x); }

Coord Coord::rotate(float angle) const {
    return Coord(this->x * std::cos(angle) - this->y * std::sin(angle),
                 this->x * std::sin(angle) + this->y * std::cos(angle));
}

} // namespace pxl
