#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "liblvgl/llemu.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "global.cpp"

/* CONTROLLER */
pros::Controller controller(pros::E_CONTROLLER_MASTER);

/* MOTORS */
pros::MotorGroup leftMotors({-11, 13, -5}, pros::MotorGearset::blue); // left motor group - ports 11 (reversed), 13, 5 (reversed)
pros::MotorGroup rightMotors({7, -8, 9}, pros::MotorGearset::blue); // right motor group - ports 7, 8 (reversed), 9
pros::Motor intakeTop(18, pros::v5::MotorGears::blue);
pros::Motor intakeBottom(19, pros::v5::MotorGears::blue);

/* IMUS */
// measures a robot's rotation and acceleration
// to track its orientation (heading, pitch, roll, etc)
pros::Imu imu(13);
pros::Imu imu2(18);

/* PNEUMATICS */
pros::adi::Pneumatics tongueMech('A', false);

/* DRIVETRAIN SETTINGS*/
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11.45, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              450, // drivetrain rpm is 450
                              2 // horizontal drift is 2.
);

/* TRACKING WHEELS */
// vertical tracking wheel encoder. Rotation sensor, port 14
pros::Rotation verticalEncoder(14);
// vertical tracking wheel. 2" diameter, 1" offset
lemlib::TrackingWheel vertical(&verticalEncoder, lemlib::Omniwheel::NEW_2, -1);

/* ODOMETRY SETTINGS (setting up PID) */
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

/* MOTION CONTROLLER SETTINGS */
// lateral motion controller (forward and backward motion)
lemlib::ControllerSettings linearController(10, // proportional gain (kP) : controls power based on error
                                            0, // integral gain (kI) : not relevant (for now?)
                                            50, // derivative gain (kD) : fights overshoot based on the change in error
                                            0, // anti windup : 0 cuz kI = 0
                                            .5, // small error range, in inches : how close to be considered "precise"
                                            100, // small error range timeout, in milliseconds : how long the robot must stay before it stops the move
                                            3, // large error range, in inches : how close to be considered "close enough"
                                            500, // large error range timeout, in milliseconds : how long the robot must stay before it's "close enough" and stops the move
                                            127 // maximum acceleration (slew)
);

// angular motion controller (turning/rotation)
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             80, // derivative gain (kD)
                                             0, // anti windup
                                             .5, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             50, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

/* DRIVER CONTROLLER SETTINGS */
// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

/* CHASIS */
// create the chassis
lemlib::Chassis chassis(drivetrain,
        linearController,
       angularController,
                        sensors,
         &throttleCurve,
            &steerCurve);

/* FUNCTIONS */
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(0, "Initializing...");

    // calibrating imus
    imu.reset();
    imu2.reset();

    pros::lcd::set_text(1, "Calibrating IMUs...");
    while (imu.is_calibrating() || imu2.is_calibrating()) {
        pros::delay(25);
    }
    pros::lcd::set_text(1, "IMUs calibrated!");

    // calibrating chasis
    pros::lcd::set_text(2, "Calibrating chassis...");
    chassis.calibrate();   // calibrates wheels + odometry
    pros::lcd::set_text(2, "Chassis calibrated!");

    // --- DEBUG TASK ---
    // prints position + telemetry to LCD every 50ms
    pros::Task screenTask([&]() {
        while (true) {
            lemlib::Pose pose = chassis.getPose();

            pros::lcd::print(0, "X: %.2f", pose.x);
            pros::lcd::print(1, "Y: %.2f", pose.y);
            pros::lcd::print(2, "T: %.2f", pose.theta);

            lemlib::telemetrySink()->info("Pose: {}", chassis.getPose());

            pros::delay(100);
        }
    });

    pros::lcd::set_text(3, "Done initializing!");
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {
    pros::lcd::initialize();
}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    // Move to x: 20 and y: 15, and face heading 90. Timeout set to 4000 ms
    chassis.moveToPose(20, 15, 90, 4000);
    // Move to x: 0 and y: 0 and face heading 270, going backwards. Timeout set to 4000ms
    chassis.moveToPose(0, 0, 270, 4000, {.forwards = false});
    // cancel the movement after it has traveled 10 inches
    chassis.waitUntil(10);
    chassis.cancelMotion();
    // Turn to face the point x:45, y:-45. Timeout set to 1000
    // dont turn faster than 60 (out of a maximum of 127)
    chassis.turnToPoint(45, -45, 1000, {.maxSpeed = 60});
    // Turn to face a direction of 90º. Timeout set to 1000
    // will always be faster than 100 (out of a maximum of 127)
    // also force it to turn clockwise, the long way around
    chassis.turnToHeading(90, 1000, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 100});
    // Follow the path in path.txt. Lookahead at 15, Timeout set to 4000
    // following the path with the back of the robot (forwards = false)
    // see line 116 to see how to define a path
    chassis.follow(example_txt, 15, 4000, false);
    // wait until the chassis has traveled 10 inches. Otherwise the code directly after
    // the movement will run immediately
    // Unless its another movement, in which case it will wait
    chassis.waitUntil(10);
    pros::lcd::print(4, "Traveled 10 inches during pure pursuit!");
    // wait until the movement is done
    chassis.waitUntilDone();
    pros::lcd::print(4, "pure pursuit finished!");
}

/**
 * Runs in driver control
 */
void opcontrol() {
    competition_initialize();
    // loop to continuously update motors
    while (true) {

        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.tank(leftY, rightX);
        // delay to save resources
        pros::delay(25);
    }
}