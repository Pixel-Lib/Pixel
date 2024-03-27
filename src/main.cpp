#include "main.h"

#include "pxl/api.hpp"

pros::Motor leftFront(1);
pros::Motor leftMiddle(2);
pros::Motor leftBack(3);

pros::Motor rightFront(4);
pros::Motor rightMiddle(5);
pros::Motor rightBack(6);

pros::MotorGroup leftMotors({leftFront, leftMiddle, leftBack});
pros::MotorGroup rightMotors({rightFront, rightMiddle, rightBack});

// Create drivetrain object
pxl::Drivetrain drivetrain(&leftMotors,   // left motors
                           &rightMotors,  // right motors
                           12.5,          // track width
                           3.25,          // diameter of wheel
                           360            // rpm of drivetrain. If unsure, type (input gear / output gear) * motor rpm
);
pros::IMU imu(8);
pros::Rotation leftRotation(7);
pxl::TrackingWheel horizontal1(&leftRotation,  // rotation sensor or shaft encoder
                               2.75,           // diameter of wheel
                               4.5             // offset from tracking center in inches
);
pxl::OdomSensors sensors(nullptr,       // vertical1
                         nullptr,       // vertical2
                         &horizontal1,  // horizontal1
                         nullptr,       // horizontal2
                         &imu           // imu
);

pxl::SeekingController linearSettings(  // linear settings
    pxl::PID(                           // PID constants
        25.0,                           // kP
        0.0,                            // kI
        4.0                             // kD
        ),
    20,  // slew rate of 20. Approximately 20 max volts per second increase
    pxl::Regression({
        // regression points. You may add as many error-timeout pairs as you want
        pxl::Coord(1.0, 100),  // error of 1 inch, timeout of 100 ms
        pxl::Coord(3.0, 500),  // error of 3 inches, timeout of 500 ms
    }),
    5000.0  // global timeout
);

pxl::SeekingController angularSettings(  // angular settings
    pxl::PID(                            // PID constants
        25.0,                            // kP
        0.0,                             // kI
        4.0                              // kD
        ),
    20,  // slew rate of 20. Approximately 20 max volts per second increase
    pxl::Regression({
        // regression points. You may add as many error-timeout pairs as you want
        pxl::Coord(0.5, 100),  // error of 0.5 degree, timeout of 100 ms
        pxl::Coord(5.0, 700),  // error of 5 degtees, timeout of 700 ms
    }),
    5000.0  // global timeout
);
pxl::Drivebase drivebase(drivetrain, sensors, linearSettings, angularSettings);  // make the drivebase
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
    static bool pressed = false;
    pressed = !pressed;
    if (pressed) {
        pros::lcd::set_text(2, "I was pressed!");
    } else {
        pros::lcd::clear_line(2);
    }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize();
    drivebase.calibrate();
    pros::Task display_coordinates([] {
        while (true) {
            pxl::Pose pose = drivebase.odom.getPose();
            pros::lcd::print(0, "X: %f", pose.x);
            pros::lcd::print(1, "Y: %f", pose.y);
            pros::lcd::print(2, "Theta: %f", pose.theta);
            pros::delay(50);
        }
    });

    pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
    //? Drive 24 inches forward with 2000 ms timeout
    drivebase.drive(24, 2000);
    //? Drive 6 inches backward with 1000 ms timeout, with a minimum speed of 20 and a maximum speed of 100
    drivebase.drive(-6, 1000, {.minSpeed = 20, .maxSpeed = 100});
    //? Turn to (18,0) degrees with a 1000 ms timeout
    drivebase.turnToPoint(18, 10, 1000);
    //? Arc turn to 135 degrees with a 2000 ms timeout
    drivebase.arcTurn(135, 2000);
    //? Boomerang to (18,0,0) with a 2000 ms timeout
    drivebase.boomerang(18, 0, 0, 2000);
    //? Arc turn to 90 degrees with a 2000 ms timeout, with a radius of 6 inches and a forward direction of false
    drivebase.arcTurn(90, 2000, {.radius = 6, .forward = false});
    //? Boomerang to (0,0,180) with a 3000 ms timeout, with a forward direction of false
    drivebase.boomerang(0, 0, 180, 3000.0, {.forward = false});
    //? Euler turn to 90 degrees with a 4000 ms timeout, forward direction of false, minimum speed of 20, and maximum
    //speed of 100
    drivebase.eulerTurn(90.0, -0.1, 4000.0, {.forward = false, .minSpeed = 20, .maxSpeed = 100});
    //? boomerang to (0,0,0) with a 1000 ms timeout, with a dlead of 0.2, glead of 0.5, and slew of 10
    drivebase.boomerang(0, 0, 0, 1000, {.dlead = 0.2, .glead = 0.5, .slew = 10});
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    pros::Motor left_mtr(1);
    pros::Motor right_mtr(2);

    while (true) {
        pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
                         (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
                         (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
        int left = master.get_analog(ANALOG_LEFT_Y);
        int right = master.get_analog(ANALOG_RIGHT_Y);

        left_mtr = left;
        right_mtr = right;

        pros::delay(20);
    }
}
