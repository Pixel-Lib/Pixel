

#include "pxl/api.hpp"
#include <yaml-cpp/yaml.h>
#include "pxl/config.hpp"
// Load the YAML file
YAML::Node config = YAML::LoadFile("src/pxl/config.yaml");

// Read the motor ports
std::vector<int8_t> rightPorts = config["motors"]["right"].as<std::vector<int8_t>>();
std::vector<int8_t> leftPorts = config["motors"]["left"].as<std::vector<int8_t>>();

// Create the motor groups
pros::MotorGroup rightMotors(rightPorts);
pros::MotorGroup leftMotors(leftPorts);


// Read the drivetrain configuration
double trackWidth = config["drivetrain"]["trackWidth"].as<double>();
double wheelDiameter = config["drivetrain"]["wheelDiameter"].as<double>();
int rpm = config["drivetrain"]["rpm"].as<int>();

// Create the drivetrain
pxl::Drivetrain drivetrain(&leftMotors, &rightMotors, trackWidth, wheelDiameter, rpm);

