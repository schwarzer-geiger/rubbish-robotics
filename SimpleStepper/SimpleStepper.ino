/*Example sketch to control a stepper motor with A4988 stepper motor driver,
  AccelStepper library and Arduino: number of steps or revolutions.
  More info: https://www.makerguides.com */

// Include the AccelStepper library:
#include "AccelStepper.h"

// Define stepper motor connections and motor interface type.
// Motor interface type must be set to 1 when using a driver
#define dirPin 5
#define stepPin 6
#define motorInterfaceType 1

// Create a new instance of the AccelStepper class:
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

void setup() {
  // Set the maximum speed in steps per second:
  stepper.setMaxSpeed(50);
  stepper.setAcceleration(5);
}

void loop() {
  stepper.setCurrentPosition(0);
  delay(1000);
  stepper.setCurrentPosition(0);
  stepper.runToNewPosition(15);
  delay(1000);
  stepper.runToNewPosition(0);
  stepper.setCurrentPosition(0);

  delay(3000);
}
