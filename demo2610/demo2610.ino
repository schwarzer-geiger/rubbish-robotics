#include <Encoder.h>

// Macros
#define CW 1
#define CCW 2

// Debugging: 0 - off, 1 - on
#define SERIAL_PRINT 1

class motor {
  public:
    int position;
    double angle;
    int powerPort;
    int stepsPerDeg;
};

class dc : motor {
  public:
    dc(int powerPin1, int powerPin2, int enc1Pin, int enc2Pin)
      : powerPin1(powerPin1), powerPin2(powerPin2), enc(enc1Pin, enc2Pin) {
    }

    // Moves motor by nSteps encoder steps into direction dir (CW or CCW) at speed 'speed'.
    void moveNSteps(int nSteps, int dir, int speed) {
      // start with motor off
      analogWrite(powerPin1, 0);
      analogWrite(powerPin2, 0);

      int lastSignal = enc.read();
      int currentSignal;
      // steps run during the current function call, direction not considered
      int stepsRun = 0;
      int initPosition = position;
      int targetPosition;

      // calculate target position
      if (dir == CW) {
        targetPosition = initPosition + nSteps;
      }
      if (dir == CCW) {
        targetPosition = initPosition - nSteps;
      }

      // start motor
      // speed requires: between 0 and 255
      // If direction pin is high, speed has to be inverted
      if (dir == CW) {
        analogWrite(powerPin1, speed);
      }
      if (dir == CCW) {
        analogWrite(powerPin2, speed);
      }

      // check whether the number of encoder steps passed 'stepsRun' has reached the desired number of steps 'nSteps' continuously
      while (stepsRun < nSteps) {
        currentSignal = enc.read();
        if (currentSignal != lastSignal) {
          stepsRun++;
          lastSignal = currentSignal;
        }
      }

      analogWrite(powerPin1, 0);
      analogWrite(powerPin2, 0);
      enc.write(0);
      Serial.println("Motor stopped");

      if (dir == CW) {
        position += stepsRun;
      }
      if (dir == CCW) {
        position -= stepsRun;
      }

      if (SERIAL_PRINT) {
        Serial.print("Initial position: ");
        Serial.println(initPosition);
        Serial.print("Target Position: ");
        Serial.println(targetPosition);
        Serial.print("Actual position: ");
        Serial.println(position);

      }
    }

    int powerPin1;
    int powerPin2;
    Encoder enc;
};

/*
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
  float theta1Steps = abs(rad2StepsDC(theta1Target) - m1.position);
  int theta1Dir = 2 - (rad2StepsDC(theta1Target) > m1.position) // CW if pos, CCW if neg
  float theta2Steps = abs(rad2StepsDC(theta2Target) - m2.position);
  int theta2Dir = 1 + (rad2StepsDC(theta2Target) > m2.position) // CCW if pos, CW if neg
  m1.moveNSteps(theta1Steps, theta1Dir, 20);
  m2.moveNSteps(theta2Steps, theta2Dir, 20);
  }
*/
int MOTOR_SPEED = 255;

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  dc m1(11, 10, 2, 4); //red, black, blue, yellow
  dc m2(9, 6, 3, 5); //red, black, blue, yellow
  m1.zeroPos();
  m2.zeroPos();
  Serial.println("Please position the robot arm so that its lower arm is vertical and its upper arm is horizontal.");
  Serial.println("Please use the following format:");
  Serial.println("               +0120|2           ");
  Serial.println("            steps ^  ^ motor index");
  Serial.println("Once the zero position has been achieved, type 'alldone' to zero the encoders");

  bool isDone = false;
  char inputs[7] = "000000";

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
        steps = (inputs[1] - '0') * 1000 + (inputs[2] - '0') * 100 + (inputs[3] - '0') * 10 + (inputs[4] - '0');
        dir = 2 - (inputs[0] == '+'); // CW (1) if +, CCW (2) if -
        index = inputs[6] - '0';

        Serial.print("Set steps ");
        Serial.print(steps);
        Serial.print(" in dir ");
        Serial.print(dir);
        Serial.print(" to index ");
        Serial.println(index);

        // TODO; this is ugly. figure out how to make it handle more than 2 motors if needed?
        if (index == 1)
          m1.moveNSteps(steps, dir, MOTOR_SPEED);
        if (index == 2)
          m2.moveNSteps(steps, dir, MOTOR_SPEED);
      }
      // quickly checking if inputs is alldone by just comparing the first letter
      if (inputs[0] == 'a') {
        isDone = true;
      }
    }
  }
}

void loop() {
}
