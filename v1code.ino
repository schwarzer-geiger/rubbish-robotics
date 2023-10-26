#include <AFMotor.h>
#include <Encoder.h>

// Macros
#define CW 1
#define CCW 2

// Debugging: 0 - off, 1 - on
#define SERIAL_PRINT 1



// #----------USER PARAMETERS START----------#

// Arm connected to base [cm]
double L1 = 10;
// Arm connected to end effector [cm]
double L2 = 10;
// encoder resolution for DC motors [steps per rotation]
int dcStepsPerTurn = 200;

// stepper motor resolution [steps per rotation]
int stepperStepsPerTurn = 48;

// microstepping factor (1/16, 1/32 etc.) [N/A]
float microstepFactor = 1;

// #----------USER PARAMETERS END----------#



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
};

// Describes angles of the end effectors relative to TBD
struct t1t2Angles {
  double theta1;
  double theta2;
};

// create dc motors specifying power port (M1-4), encoder pin 1 and encoder pin 2.
dc motor1(1, 2, 3);
dc motor2(4, 5, 6);

// turn an angle in degrees to number of steps
float deg2StepsDC(float angle){
  return dcStepsPerTurn*angle/360;
}

float rad2StepsDC(float radians){
 return dcStepsPerTurn*angle/(PI*2);
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
int moveToXY(float xTarget, float yTarget) {

  float D = sqrt(pow(xTarget, 2) + pow(yTarget, 2));
  if (D > L1 + L2) {
    return -1;
  }

  float gamma = atan2(yTarget, xTarget);
  // assuming theta 1 controls the lower arm and theta 2 is for the upper arm,
  // assuming robot has been zeroed in a vertical upper arm, horisontal lower arm position
  // assuming when looking at the robot from its right side,  theta 1 is positive in CW and theta 2 is positive in CCW

  float theta1Target = asin( (pow(L1, 2) + pow(D, 2) - pow(L2, 2)) / (2 * L1 * D) ) - gamma;
  float theta2Target = asin( (pow(D, 2) - pow(L1, 2) - pow(L2, 2)) / (2 * L1 * L2) ) - theta1Target;
  // TODO; moveNSteps adds to the current position! Better to drive motor exactly to this position, there has to be a prettier way
  float theta1Steps = abs(rad2StepsDC(theta1Target) - motor1.position);
  int theta1Dir = 2 - (rad2StepsDC(theta1Target) > motor1.position) // CW if pos, CCW if neg
  float theta2Steps = abs(rad2StepsDC(theta2Target) - motor2.position);
  int theta2Dir = 1 + (rad2StepsDC(theta2Target) > motor2.position) // CCW if pos, CW if neg
  moveNStepsDC(motor1, theta1Steps, theta1Dir, 20);
  moveNStepsDC(motor2, theta2Steps, theta2Dir, 20);
}

void setup() {
  
  // executes zeroing process
  Serial.begin(9600);
  Serial.println("Please position the robot arm so that its lower arm is vertical and its upper arm is horizontal.");
  Serial.println("Please use the following format:");
  Serial.println("               +0120|2           ");
  Serial.println("            steps ^  ^ motor index");
  Serial.println("Once the zero position has been achieved, type 'alldone' to zero the encoders");
  
  bool isDone = false;
  String inputs = String(7);
  
  while (!isDone) {
    // to prevent risk of broken data, only read if there are at least 7 characters
    if (Serial.available() >= 7) {
      int steps;
      int dir;
      int index;
      for (int i = 0; i < 7; i++) {
        inputs[i] = Serial.read();
      }
      
      // discard anything after the first 7 characters just in case
      while (Serial.available()) {
        Serial.read();
      }
      
      if (inputs[0] == '+' || inputs[0] == '-') {
        steps = inputs.substring(1, 5).toInt();
        dir = 2 - (inputs[0] == '+'); // CW (1) if +, CCW (2) if -
        index = inputs[6] - '0';
        
        Serial.print("Set steps ");
        Serial.print(steps);
        Serial.print(" to index ");
        Serial.println(index);
        
        // TODO; this is ugly. figure out how to make it handle more than 2 motors if needed?
        if (index == 1)
          moveNStepsDC(motor1, steps, dir, 20);
        if (index == 2)
          moveNStepsDC(motor2, steps, dir, 20);
      }
      // quickly checking if inputs is alldone by just comparing the first letter
      if (inputs[0] == 'a')
        isDone = true;
    }
  }

  motor1.enc.write(0);
  motor2.enc.write(0);
  motor1.position = 0;
  motor2.position = 0;
  Serial.println("Encoders successfully zeroed!");

}

void loop() {
  //moveNStepsDC(motor1, 2000, CW, 200);
}
