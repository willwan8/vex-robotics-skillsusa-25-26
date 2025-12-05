#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "liblvgl/llemu.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "global.h"
#include "helpers.h"
#include "auton.h"
#include <algorithm>

/* CONTROLLER */
pros::Controller controller(pros::E_CONTROLLER_MASTER); // not in global since not used anywhere else

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
    // initializing starting position
    double averageHeading = averageImuHeading(imu.get_heading(), imu2.get_heading());
    chassis.setPose(0, 0, averageHeading);
    tongueMech.retract(); // just to ensure tongue is retracted as we will not be using the loaders for this routine
    
    // block pickup whilst traveling to long goal
    setSpeedIntakeBottom(115);

    // route one (collecting blocks and scoring)
    autonRouteOne();

    // route two (go to new start point, collect blocks and score)
    autonRouteTwo();

    // route three (parking)
    autonRouteThree();
    

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
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        // move the chassis with curvature drive
        chassis.tank(leftY, rightY);
        // delay to save resources
        pros::delay(25);
    }
}