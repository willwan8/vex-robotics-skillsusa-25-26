#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "liblvgl/llemu.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "global.h"
#include "helpers.h"
#include "auton.h"
#include "intakeTasks.h"
#include <algorithm>

/* CONTROLLER */
pros::Controller master(pros::E_CONTROLLER_MASTER); // not in global since not used anywhere else

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

    // intake tasks
    pros::Task intakeTask1(intake1Task);
    pros::Task intakeTask2(intake2Task);

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
	skillsAuton();
}

/**
 * Runs in driver control
 */
void opcontrol() {
    competition_initialize();
    // loop to continuously update motors
    while (true) {
        pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
            (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
            (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
        
        if(master.get_digital_new_press(DIGITAL_Y) && master.get_digital_new_press(DIGITAL_X)){
            autonomous();
        }

        if (master.get_digital_new_press(DIGITAL_L1) && !(wing.is_extended())) {
            wing.extend();            
        }
        else if (master.get_digital_new_press(DIGITAL_L2) && wing.is_extended()) {
            wing.retract();
        }

        setIntakeTwo((master.get_digital(DIGITAL_L1) - master.get_digital(DIGITAL_L2)) * 127);

        //Hack to fix PROS key stroke issues. it gives button press even when it not pressed.
        if (master.get_digital(DIGITAL_X)) {
            setIntakeM(90);
        } else {
            setIntakeOne((master.get_digital(DIGITAL_R1) - master.get_digital(DIGITAL_R2)) * 127);
        }



		if (master.get_digital_new_press(DIGITAL_UP)) {
		    tongue.toggle();
		}

        if (master.get_digital_new_press(DIGITAL_B)) {
		    dec.toggle();
		}

        if (master.get_digital_new_press(DIGITAL_A)) {
            wing.toggle();
        }

        // **Tank drive control**
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        chassis.tank(leftY, rightY);

        pros::delay(25);  // Prevents CPU overuse
    }
}
