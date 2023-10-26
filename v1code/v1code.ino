#include <AFMotor.h>
#include <Encoder.h>

// Macros
#define CW 1
#define CCW 2

// Debugging: 0 - off, 1 - on
#define SERIAL_PRINT 1

// A dc object consists of a motor and an encoder object, and a position variable to track the motor position
// Clockwise (CW) rotation increases the position value.

class dc {
  public:
    dc(int powerPort, int enc1Pin, int enc2Pin) : mDriver(powerPort), enc(enc1Pin, enc2Pin) {
      position = 0;
    }

    AF_DCMotor mDriver;
    Encoder enc;
    int position;
};

// A stepper object is defined by its dirPin which sets the pin that controls the direction of rotation (HIGH/LOW) and a stepPin.
// Setting the stepPin HIGH and then LOW defines one step movement.
class stepper {
  public:
    stepper(int dirPin, int stepPin) : dirPin(dirPin), stepPin(stepPin) {
      pinMode(stepPin, OUTPUT);
      pinMode(dirPin, OUTPUT);
      position = 0;
    }

    int dirPin;
    int stepPin;
    int position;  
};

// Describes a position with coordinates x and y in [cm] with (0, 0) at TBD
struct xyPosition {
  double x;
  double y;
}

// Describes angles of the end effectors relative to TBD
struct t1t2Angles {
  double theta1;
  double theta2;
}

// create dc motors specifying power port (M1-4), encoder pin 1 and encoder pin 2.
dc motor1(1, 2, 3);
dc motor2(4, 5, 6);

void setup() {
  if (SERIAL_PRINT) {
    Serial.begin(9600);
  }

  // define robot dimensions in [cm]

  // Arm connected to base
  double L1 = 10;

  // Arm connected to end effector
  double L2 = 10;
}

// Moves a DC motor by nSteps encoder steps into direction dir (CW or CCW) at speed 'speed'.
void moveNStepsDC(dc motor, int nSteps, int dir, int speed) {

  int lastSignal = motor.enc.read();
  int currentSignal;
  // steps run during the current function call, direction not considered
  int stepsRun = 0;
  int initPosition = motor.position;
  int targetPosition;

  if (dir == CW) {
    targetPosition = initPosition + nSteps;
  } else {
    targetPosition = initPosition - nSteps;
  }

  // start motor
  motor.mDriver.setSpeed(speed);
  motor.mDriver.run(dir);

  // check whether the number of encoder steps passed 'stepsRun' has reached the desired number of steps 'nSteps' continuously
  while (stepsRun < nSteps) {
    currentSignal = motor.enc.read();
    if (currentSignal != lastSignal) {
      stepsRun++;
      lastSignal = currentSignal;
    }
  }

  motor.mDriver.run(RELEASE);
  motor.enc.write(0);
  Serial.println("Motor stopped");

  if (dir == CW) {
    motor.position += stepsRun;
  } else {
    motor.position -= stepsRun;
  }

  if (SERIAL_PRINT) {
    Serial.println("Initial position: " + initPosition);
    Serial.println("Target Position: " + targetPosition);
    Serial.println("Actual position: " + motor.position);
  }

}

void moveNStepsStepper(stepper motor, int nSteps, int dir, int speed) {
  int initPosition = motor.position;
  // CW/CCW takes values 1/2, need to convert to values 0/1 for LOW/HIGH
  motor.dirPin = dir - 1;

  for (int stepsRun = 0; stepsRun < nSteps; stepsRun++) {
    digitalWrite(motor.stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(motor.stepPin, LOW);
    delayMicroseconds(500);
  }
}

// Returns current (x,y) position of the end effector
struct xyPosition getCurrentXY() {

}

// Returns current (theta1, theta2) position of the robot arms
struct t1t2Angles getCurrentT1T2() {

}

// Returns -1 if xTarget and/or yTarget are out of range of motion
// Returns -2 if robot didn't manage to move to (xTarget, yTarget)
int moveToXY(int xTarget, int yTarget) {
  // populate with inverse kinematics
  double theta1Target = -1;
  double theta2Target = -1;
  
} 

void loop() {
  moveNStepsDC(motor1, 2000, CW, 200);
}
