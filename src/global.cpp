#include "main.h"
#include "global.h"
#include "lemlib/api.hpp"

/* GLOBALS */

/* INTAKE */
pros::Motor intakeOne(14, pros::v5::MotorGears::blue); // intake motor on port 14
pros::Motor intakeM(-12, pros::v5::MotorGears::green); // middle intake motor on port 12, reversed
pros::Motor intakeT(19, pros::v5::MotorGears::green); // top intake motor on port 19

/* MOTORS */
pros::MotorGroup leftMotors({11, -13, -4}, pros::MotorGearset::blue); // left motor group - ports 11, 13 (reversed), 4 (reversed)
pros::MotorGroup rightMotors({7, -8, 9}, pros::MotorGearset::blue); // right motor group - ports 7, 8 (reversed), 9

/* IMUS */
// measures a robot's rotation and acceleration
// to track its orientation (heading, pitch, roll, etc)
pros::Imu imu(15);
pros::Imu imu2(20);

/* DISTANCE SENSORS*/
pros::Distance distanceL(17);
pros::Distance distanceR(18);

/* DRIVETRAIN SETTINGS*/
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11.06, // track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              450, // drivetrain rpm is 450
                              8 // horizontal drift
);

/* PNEUMATICS */
pros::adi::Pneumatics tongue('H', false);
pros::adi::Pneumatics wing('E', false);

/* TRACKING WHEELS */
// vertical tracking wheel encoder. Rotation sensor, port 6 reversed
pros::Rotation verticalEncoder(-6);
// vertical tracking wheel. 2" diameter, 1" offset from center
lemlib::TrackingWheel vertical(&verticalEncoder, lemlib::Omniwheel::NEW_2, 0);

/* ODOMETRY SETTINGS (setting up PID) */
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

/* MOTION CONTROLLER SETTINGS */
// lateral motion controller (forward and backward motion)
lemlib::ControllerSettings lateral_controller(9, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              20, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular motion controller (turning/rotation)
lemlib::ControllerSettings angular_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              80, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

/* DRIVER CONTROLLER SETTINGS */
// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

/*
// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);
*/

/* CHASIS */
// create the chassis
lemlib::Chassis chassis(drivetrain,
        linearController,
       angularController,
                        sensors,
         &throttleCurve
);