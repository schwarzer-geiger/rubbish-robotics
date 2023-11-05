/*Example sketch to control a stepper motor with A4988 stepper motor driver,
  AccelStepper library and Arduino: number of steps or revolutions.
  More info: https://www.makerguides.com */

// Include the AccelStepper library:
#include "AccelStepper.h"
#include <MultiStepper.h>

// Define stepper motor connections and motor interface type.
// Motor interface type must be set to 1 when using a driver
#define dirPin 5
#define stepPin 6
#define motorInterfaceType 1

// Create a new instance of the AccelStepper class:
AccelStepper stepper1 = AccelStepper(motorInterfaceType, stepPin, dirPin);
AccelStepper stepper2 = AccelStepper(1, 9, 8);
MultiStepper steppers;

void setup() {
  Serial.begin(9600);
  // Set the maximum speed in steps per second:
  int SPEED = 80;
  stepper1.setMaxSpeed(SPEED);
  stepper1.setAcceleration(20);
  stepper2.setMaxSpeed(SPEED);
  stepper2.setAcceleration(20);
  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);
}

void loop() {

  int range1 = 40;
  int range2 = 40;

  long positions[2]; // Array of desired stepper positions

  positions[0] = range1;
  positions[1] = range2;

  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  Serial.println("Moved positive");
  delay(1000);

  // Move to a different coordinate
  positions[0] = -range1;
  positions[1] = -range2;
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  Serial.println("Moved negative");
  delay(1000);
}
