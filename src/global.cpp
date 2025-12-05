#include "main.h"
#include "global.h"
#include "lemlib/api.hpp"

/* GLOBALS */

/* INTAKE */
pros::Motor intakeTop(18, pros::v5::MotorGears::blue);
pros::Motor intakeBottom(19, pros::v5::MotorGears::blue);

/* MOTORS */
pros::MotorGroup leftMotors({-11, 13, -5}, pros::MotorGearset::blue); // left motor group - ports 11 (reversed), 13, 5 (reversed)
pros::MotorGroup rightMotors({7, -8, 9}, pros::MotorGearset::blue); // right motor group - ports 7, 8 (reversed), 9

/* IMUS */
// measures a robot's rotation and acceleration
// to track its orientation (heading, pitch, roll, etc)
pros::Imu imu(15);
pros::Imu imu2(16);

/* DRIVETRAIN SETTINGS*/
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11.45, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              450, // drivetrain rpm is 450
                              2 // horizontal drift is 2.
);

/* PNEUMATICS */
pros::adi::Pneumatics tongueMech('A', false);

/* TRACKING WHEELS */
// vertical tracking wheel encoder. Rotation sensor, port 14
pros::Rotation verticalEncoder(14);
// vertical tracking wheel. 2" diameter, 1" offset from center
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