#include "main.h"
#include <cmath>
#include "global.h"
#include "helpers.h"

void setSpeedIntakeTop(int speed) {
    intakeTop.move(speed);
    pros::delay(10);
}

void setSpeedIntakeBottom(int speed) {
    intakeBottom.move(speed);
    pros::delay(10);
}

double averageImuHeading(double h1, double h2) {
    // convert to unit circle values to prevent naive averaging
    // ex of naive averaging: 359 + 1 => avg of 180, but actual avg is 0!
    // but if we converted them into unit circle value first, then we'd get an avg of 0!
    // then add together to get an angle in the middle of h1 and h2 (basically an average)
    double xSum = cos((h1 * (2 * M_PI)) / 360.0) + cos((h2 * (2 * M_PI)) / 360.0);
    double ySum = sin((h1 * (2 * M_PI)) / 360.0) + sin((h2 * (2 * M_PI)) / 360.0);

    // convert back to degrees
    double angle = (atan2(ySum, xSum) * 360.0) / (2*M_PI);
    
    // convert angles from [0, 360]
    // add 360 in case of negatives
    return fmod(angle + 360.0, 360.0);
}