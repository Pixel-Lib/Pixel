#include "pxl/drivebase/trackingwheel.hpp"



namespace pxl {

// Constructor with encoder
TrackingWheel::TrackingWheel(pros::ADIEncoder *encoder, float wheelDiameter, float distance, float gearRatio)
    : encoder(encoder), diameter(wheelDiameter), distance(distance), gearRatio(gearRatio) {}

// Constructor with rotation sensor
TrackingWheel::TrackingWheel(pros::Rotation *rotation, float wheelDiameter, float distance, float gearRatio)
    : rotation(rotation), diameter(wheelDiameter), distance(distance), gearRatio(gearRatio) {}

// Constructor with motor group
TrackingWheel::TrackingWheel(pros::MotorGroup *motors, float wheelDiameter, float distance, float rpm)
    : motors(motors), diameter(wheelDiameter), distance(distance), rpm(rpm) {
    motors->set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
}

// Reset the wheel measurement
bool TrackingWheel::reset() { return this->encoder->reset(); }

// Get the distance traveled by the wheel
float TrackingWheel::getDistanceTraveled() {
    if (this->encoder != nullptr)
        return (float(this->encoder->get_value()) * this->diameter * M_PI / 360) / this->gearRatio;
    else if (this->rotation != nullptr)
        return (float(this->rotation->get_position()) * this->diameter * M_PI / 36000) / this->gearRatio;
    else if (this->motors != nullptr) {
        auto gearsets = this->motors->get_gearing();
        auto positions = this->motors->get_positions();
        std::vector<float> distances;
        std::transform(positions.begin(), positions.end(), gearsets.begin(), std::back_inserter(distances),
                       [&](double pos, pros::motor_gearset_e_t gearset) {
                           float in = (gearset == pros::E_MOTOR_GEARSET_36)   ? 100
                                      : (gearset == pros::E_MOTOR_GEARSET_18) ? 200
                                      : (gearset == pros::E_MOTOR_GEARSET_06) ? 600
                                                                              : 200;
                           return pos * diameter * M_PI * rpm / in;
                       });
        return pxl::avg(distances);
    } else {
        return 0;
    }
}

// Get the distance delta or the change in distance
float TrackingWheel::getDistanceDelta(bool update) {
    const float prevAngle = this->lastAngle;
    const float angle = this->encoder->get_value();
    if (!update) this->lastAngle = prevAngle;
    return (angle - prevAngle) / 2 * this->diameter;
}

// Get the wheel offset
float TrackingWheel::getOffset() { return this->distance; }

// Get the type of wheel (0 for encoder/rotation, 1 for motor group)
int TrackingWheel::getType() { return (this->motors != nullptr) ? 1 : 0; }

}  // namespace pxl