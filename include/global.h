#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "lemlib/api.hpp"
#include "pros/adi.hpp"

extern pros::Motor intakeTop;
extern pros::Motor intakeBottom;
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;
extern pros::Imu imu;
extern pros::Imu imu2;
extern lemlib::Drivetrain drivetrain;
extern pros::adi::Pneumatics tongueMech;
extern pros::Rotation verticalEncoder;
extern lemlib::TrackingWheel vertical;
extern lemlib::OdomSensors sensors;
extern lemlib::ControllerSettings linearController;
extern lemlib::ControllerSettings angularController;
extern lemlib::ExpoDriveCurve throttleCurve;
extern lemlib::ExpoDriveCurve steerCurve;
extern lemlib::Chassis chassis;