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
  chassis.moveToPoint(0, 24.14, 1000, {.maxSpeed=80});
  chassis.turnToHeading(90, 500);
  chassis.moveToPoint(28, 24.14, 1000, {.maxSpeed=80});
  chassis.moveToPoint(24, 24.14, 500, {.forwards=false});
  chassis.turnToHeading(0, 500);
  chassis.moveToPoint(24, 84.14, 1500, {.forwards=true, .maxSpeed=115});
  chassis.turnToHeading(90, 500);
  chassis.moveToPoint(48, 84.14, 1000, {.maxSpeed=80});

  // scoring
  chassis.turnToHeading(0, 500);
  chassis.moveToPoint(48, 79.14, 1000, {.forwards=false, .maxSpeed=50});
  scoreBlocks();

  
  /*
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
  */
}

void autonRouteTwo() {
  // return to new start
  chassis.moveToPoint(48, 84.14, 1000, {.forwards=true, .maxSpeed=50});
  chassis.turnToHeading(270, 500);
  chassis.moveToPoint(-48, 84.14, 2000, {.maxSpeed=115});
  chassis.turnToHeading(180, 500);
  chassis.moveToPoint(-48, 12.14, 2000, {.maxSpeed=115});
  chassis.turnToHeading(270, 500);
  chassis.moveToPoint(-72, 12.14, 1000, {.maxSpeed=80});

  //scoring
  chassis.turnToHeading(180, 500);
  chassis.moveToPoint(-72, 17.14, 1000, {.forwards=false, .maxSpeed=50});
  scoreBlocks();
  
  /*
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
  */
}

void autonRouteThree() {
  // going back to park
  chassis.moveToPoint(-72, 12.14, 1000, {.fowards=true, .maxSpeed=50});
  chassis.turnToHeading(90, 500);
  chassis.moveToPoint(0, 12.14, 1000, {.maxSpeed=80});
  chassis.turnToHeading(180, 500);
  chassis.moveToPoint(0, -15, 1000, {.maxSpeed=80});
  
  /*
  pros::delay(200);
  chassis.moveToPose(0, 21.86, 180, 1500, {.maxSpeed=115});
  pros::delay(200);
  chassis.moveToPose(0, -9.86, 180, 1000, {.forwards=false, .maxSpeed=70});
  pros::delay(200);
  */
}
