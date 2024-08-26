// config.hpp
#pragma once

#include "pxl/api.hpp"
#include <yaml-cpp/yaml.h>
#include <vector>

// Declare the motor groups and drivetrain
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;
extern pxl::Drivetrain drivetrain;

// Function to load the configuration
void loadConfig(const std::string &filePath);