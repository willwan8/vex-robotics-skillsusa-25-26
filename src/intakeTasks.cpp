#include "pros/motors.hpp"
#include "main.h"
#include "helpers.h"

// Global variables
int intake1Speed = 0;
int intake2Speed = 0;




// Intake control task
void intake1Task() {
    int lastSpeed = 0; // Track last intake speed
    while (true) {
        // Only update if speed changes
        if (intake1Speed != lastSpeed) { 
            setIntakeOne(intake1Speed);
            lastSpeed = intake1Speed;
        }
        pros::delay(10); // Prevent CPU overload
    }
}

// Intake control task
void intake2Task() {
    int lastSpeed = 0; // Track last intake speed
    while (true) {
        // Only update if speed changes
        if (intake2Speed != lastSpeed) { 
            setIntakeTwo(intake2Speed);
            lastSpeed = intake2Speed;
        }
        pros::delay(10); // Prevent CPU overload
    }
}