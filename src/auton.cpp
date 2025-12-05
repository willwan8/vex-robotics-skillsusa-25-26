#include "main.h"
#include "auton.h"
#include "helpers.h"
#include "lemlib/api.hpp"

void scoreBlocks() {
  setSpeedIntakeTop(-115);
  pros::delay(3000);
  setSpeedIntakeTop(0);
}

void autonRouteOne() {
  chassis.moveToPose(0, 24.14, 90, 1000, {.maxSpeed=80}); // (24-16.86) + 24 - 7 for y coord (of center) (now that we know the center, we can do other calculations "normally")
  pros::delay(200);
  chassis.moveToPose(33,24.14, 0, 1500, {.maxSpeed=100});
  pros::delay(200);
  chassis.moveToPose(33, 72.14, 270, 1500, {.maxSpeed=115});
  pros::delay(200);
  chassis.moveToPose(9, 72.14, 0, 1000, {.maxSpeed=80});
  pros::delay(200);
  chassis.moveToPose(9, 86.14, 90, 700, {.maxSpeed=70});
  pros::delay(200);
  chassis.moveToPose(45, 86.14, 0, 1500, {.maxSpeed=100});
  pros::delay(200);

  // scoring
  chassis.moveToPose(45, 82.14, 0, 500, {.forwards=false, .maxSpeed=50});
  pros::delay(200);
  scoreBlocks();
}

void autonRouteTwo() {
  // return to new starting point
  chassis.moveToPose(45, 86.14, 270, 500, {.forwards=true, .maxSpeed=50});
  pros::delay(200);
  chassis.moveToPose(0, 86.14, 180, 1500, {.maxSpeed=115});
  pros::delay(200);
  chassis.moveToPose(0, 76, 180, 700, {.forwards=false, .maxSpeed=70});
  pros::delay(200);

  // collecting blocks and scoring
  chassis.moveToPose(0, 51.86, 270, 1000, {.maxSpeed=80});
  pros::delay(200);
  chassis.moveToPose(-33,51.86, 180, 1500, {.maxSpeed=100});
  pros::delay(200);
  chassis.moveToPose(-33, 3.86, 90, 1500, {.maxSpeed=115});
  pros::delay(200);
  chassis.moveToPose(-9, 3.86, 180, 1000, {.maxSpeed=80});
  pros::delay(200);
  chassis.moveToPose(-9, 17.86, 270, 700, {.maxSpeed=70});
  pros::delay(200);
  chassis.moveToPose(-45, 17.86, 180, 1500, {.maxSpeed=100});
  pros::delay(200);

  // scoring
  chassis.moveToPose(-45, 21.86, 180, 500, {.forwards=false, .maxSpeed=50});
  pros::delay(200);
  scoreBlocks();
}

void autonRouteThree() {
  // going back to park
  chassis.moveToPose(-45, 21.86, 90, 500, {.forwards=true, .maxSpeed=50});
  pros::delay(200);
  chassis.moveToPose(0, 21.86, 180, 1500, {.maxSpeed=115});
  pros::delay(200);
  chassis.moveToPose(0, -9.86, 180, 1000, {.forwards=false, .maxSpeed=70});
  pros::delay(200);
}
