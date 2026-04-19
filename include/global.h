#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "lemlib/api.hpp"
#include "pros/adi.hpp"

extern pros::Motor intakeOne;
extern pros::Motor intakeM;
extern pros::Motor intakeT;
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;
extern pros::Imu imu;
extern pros::Imu imu2;
extern lemlib::Drivetrain drivetrain;
extern pros::adi::Pneumatics tongue;
extern pros::adi::Pneumatics wing;
extern pros::Rotation verticalEncoder;
extern lemlib::TrackingWheel vertical;
extern lemlib::OdomSensors sensors;
extern lemlib::ControllerSettings linearController;
extern lemlib::ControllerSettings angularController;
extern lemlib::ExpoDriveCurve throttleCurve;
extern lemlib::ExpoDriveCurve steerCurve;
extern lemlib::Chassis chassis;