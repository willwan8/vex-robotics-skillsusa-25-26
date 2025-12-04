#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "liblvgl/llemu.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "global.cpp"
#include "helpers.cpp"

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

		pros::lcd::set_text(0, "Done initializing!");
		pros::delay(1000); // so the message can appear on screen before telemetry

    // --- TELEMETRY TASK ---
    // prints position + telemetry to LCD every 50ms
    pros::Task screenTask([&]() {
        while (true) {
            lemlib::Pose pose = chassis.getPose();

            pros::lcd::print(0, "X: %f", pose.x);
            pros::lcd::print(1, "Y: %f", pose.y);
            pros::lcd::print(2, "Theta: %f", pose.theta);

            pros::lcd::print(3, "IMU1 Heading: %f", imu.get_heading());
						pros::lcd::print(4, "IMU2 Heading: %f", imu2.get_heading());
						pros::lcd::print(5, "AVG IMU Heading: %f", averageImuHeading(imu.get_heading(), imu2.get_heading()));
						pros::lcd::print(6, "IMU1 Orientation: %f", imu.get_physical_orientation());
						pros::lcd::print(7, "IMU2 Orientation: %f", imu2.get_physical_orientation());

						pros::lcd::print(8, "Rotation Sensor: %i", verticalEncoder.get_position());

            pros::delay(100);
        }
    });

    
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
//ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    // code auton :)
}

/**
 * Runs in driver control
 */
void opcontrol() {
    competition_initialize();
    // loop to continuously update motors
    while (true) {

			/* CONTROLS */
			// running intake motors forward/backward
			setSpeedIntakeTop((controller.get_digital(DIGITAL_R1) - controller.get_digital(DIGITAL_R2)) * 115);
			setSpeedIntakeBottom((controller.get_digital(DIGITAL_L1) - controller.get_digital(DIGITAL_L2)) * 115);

			// pneumatic controls
			if (controller.get_digital_new_press(DIGITAL_UP)) {
				tongueMech.extend();
			}
			if (controller.get_digital_new_press(DIGITAL_DOWN)) {
				tongueMech.retract();
			}

			/* DRIVING */
      // get joystick positions
      int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
      int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
      // move the chassis with curvature drive
     	chassis.tank(leftY, rightX);
      // delay to save resources
      pros::delay(25);
    }
}