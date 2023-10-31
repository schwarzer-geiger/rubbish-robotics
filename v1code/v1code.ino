#include <Encoder.h>

// Macros
#define CW 0
#define CCW 1

// Debugging: 0 - off, 1 - on
#define SERIAL_PRINT 1

// #----------USER PARAMETERS START----------#

// Arm connected to base [cm]
float L1 = 15;
// Arm connected to end effector [cm]
float L2 = 10;

// #----------USER PARAMETERS END----------#

// A dc object consists of a motor and an encoder object, and a position variable to track the motor position
// Clockwise (CW) rotation increases the position value.

class motor {
  public:
    virtual void moveNSteps(int, int, int) {
    }

    // returns the angle of the robot arm in degrees [see drawing]
    float getAngle() {
      return (float) position / stepsPerDeg;
    }

    // moves motor/arm to desired angle at desired speed
    void setAngle(float targetAngle, int speed) {
      float currentAngle = getAngle();
      float difference = targetAngle - currentAngle;
      int stepsToTurn = (int) abs(difference * stepsPerDeg);
      int dir;
      if (difference > 0) {
        dir = CW;
      } else {
        dir = CCW;
      }
      Serial.print("here6: ");
      Serial.println(stepsToTurn);
      moveNSteps(stepsToTurn, dir, speed);
    }

    int position = 0;
    float angle;
    int powerPort;
    float stepsPerDeg;
    float calibAngle;
    int defaultSpeed;

  protected:
    // Helper function, contains all the zero'ing code compatible with both motor types
    void manualMov() {

      // executes zeroing process
      Serial.println("Please use the following command format to move the arm.");
      Serial.println("               +0120           ");
      Serial.println("            steps ^");
      Serial.println("Once the desired position has been achieved, type 'alldone'");

      bool isDone = false;
      char inputs[6] = "00000";

      while (!isDone) {
        // to prevent risk of broken data, only read if there are at least 7 characters
        if (Serial.available() >= 5) {
          int steps;
          int dir;
          int index;
          for (int i = 0; i < 5; i++) {
            inputs[i] = Serial.read();
          }

          // discard anything after the first 5 characters just in case
          while (Serial.available()) {
            Serial.read();
          }

          // TODO: Check this with new input format, don't understand
          if (inputs[0] == '+' || inputs[0] == '-') {
            steps = (inputs[1] - '0') * 1000 + (inputs[2] - '0') * 100 + (inputs[3] - '0') * 10 + (inputs[4] - '0');
            dir = !(inputs[0] == '+');  // CW (1) if +, CCW (2) if -
            // not needed anymore? index = inputs[6] - '0';

            Serial.print("Set steps ");
            Serial.print(steps);
            Serial.print(" to index ");
            Serial.println(index);

            moveNSteps(steps, dir, defaultSpeed);
          }
          // quickly checking if inputs is alldone by just comparing the first letter
          if (inputs[0] == 'a')
            isDone = true;
        }
      }
    }
};



class dc : public motor {
  public:
    dc(int powerPin1, int powerPin2, int enc1Pin, int enc2Pin, float calibAngle)
      : powerPin1(powerPin1), powerPin2(powerPin2), enc(enc1Pin, enc2Pin) {
        defaultSpeed = 255;
        calibAngle = calibAngle;
    }

    // Moves motor by nSteps encoder steps into direction dir (CW or CCW) at speed 'speed'.
    void moveNSteps(int nSteps, int dir, int speed) override {
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
      } else {
        targetPosition = initPosition - nSteps;
      }

      // start motor
      // speed requires: between 0 and 255
      if (dir == CW) {
        analogWrite(powerPin1, speed);
      } else {
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
      } else {
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
    
    void zero() {
      manualMov();
      enc.write(0);
      position = 0;
      Serial.println("This arm successfully zero'd!");
    }

    void calibrate() {
      manualMov();
      stepsPerDeg = abs(position / calibAngle);
      enc.write(0);
    }

    int powerPin1;
    int powerPin2;
    Encoder enc;
};

// A stepper object is defined by its dirPin which sets the pin that controls the direction of rotation (HIGH/LOW) and a stepPin.
// Setting the stepPin HIGH and then LOW defines one step movement.
class stepper : motor {
  public:
    stepper(int dirPin, int enablePin, int pulsePin, float calibAngle)
      : dirPin(dirPin), enablePin(enablePin), pulsePin(pulsePin) {
      pinMode(dirPin, OUTPUT);
      pinMode(enablePin, OUTPUT);
      pinMode(pulsePin, OUTPUT);
      defaultSpeed = 10;
      calibAngle = calibAngle;
    }

    void moveNSteps(int nSteps, int dir, int speed) override {
      digitalWrite(dirPin, dir);
      digitalWrite(enablePin, HIGH);
      
      pulseDelay = 1/speed * 10000;
      
      for (int i = 0; i < nSteps; i++) {
        digitalWrite(pulsePin, HIGH);
        delayMicroseconds(pulseDelay);
        digitalWrite(pulsePin, LOW);
        delayMicroseconds(pulseDelay);
      }

      Serial.println("Motor stopped");

      if (dir == CW) {
        position += nSteps;
      } else {
        position -= nSteps;
      }
    }

    void zero() {
      manualMov();
      position = 0;
      Serial.println("This arm successfully zero'd!");
    }
    
    void calibrate() {
      manualMov();
      stepsPerDeg = abs(position / calibAngle);
      Serial.println("This arm successfully calibrated!");
    }

    int dirPin;
    int enablePin;
    int pulsePin;
    int pulseDelay;
};

// Returns -1 if xTarget and/or yTarget are out of range of motion
// Returns -2 if robot didn't manage to move to (xTarget, yTarget)
// NOTE: IF THESE ARE DC INSTEAD OF MOTOR, IT USES THE PLACEHOLDER MOVENSTEPS CLASS
// AND IT DOESNT DRIVE THE MOTORS!
int moveToXY(motor m1, motor m2, float xTarget, float yTarget) {

  float D = sqrt(pow(xTarget, 2) + pow(yTarget, 2));
  Serial.print("here1:");
  Serial.println(D);

  if (D > (L1 + L2)) {
    Serial.println("oops");
    return -1;
  }

  Serial.println("here3");
  float gamma = atan2(yTarget, xTarget);
  // assuming theta 1 controls the lower arm and theta 2 is for the upper arm,
  // assuming robot has been zeroed in a vertical upper arm, horisontal lower arm position
  // assuming when looking at the robot from its right side,  theta 1 is positive in CW and theta 2 is positive in CCW

  Serial.print("here4: ");
  Serial.println(gamma);

  float theta1Target = asin((pow(L1, 2) + pow(D, 2) - pow(L2, 2)) / (2 * L1 * D)) - gamma;
  float theta2Target = asin((pow(D, 2) - pow(L1, 2) - pow(L2, 2)) / (2 * L1 * L2)) - theta1Target;

  Serial.print("here5: ");
  Serial.print(theta1Target * 180 / PI);
  Serial.print(" ");
  Serial.print(theta2Target * 180 / PI);

  m1.setAngle(theta1Target * 180 / PI, m1.defaultSpeed);
  m2.setAngle(theta2Target * 180 / PI, m2.defaultSpeed);

  return 0;
}

void setup() {

  // create dc motors specifying power pin, direction pin, encoder pin 1 and encoder pin 2.
  dc motor1(9, 6, 3, 5, 22.5);
  dc motor2(11, 10, 2, 4, 22.5);

  Serial.begin(9600);
  Serial.println("Please position the robot arm so that its lower arm is vertical and its upper arm is horizontal.");
  Serial.println("Start with the lower arm by moving it to the vertical position.");
  motor1.zero();
  Serial.println("Now move the upper arm into the horizontal position.");
  motor2.zero();
  Serial.println("Now calibrate the motors by moving the arms to the marked positions.");
  Serial.println("You can now move motor 1.");
  motor1.calibrate();
  Serial.println("You can now move motor 2.");
  motor2.calibrate();
  Serial.println("Motor calibration done. Proceeding with IK!");
  while (true) {
    moveToXY(motor1, motor2, 12.0, 6.0);
    delay(1000);
    moveToXY(motor1, motor2, 12.0, 11.0);
    delay(1000);
  }

  // (12, 6) (12, 2)
}

void loop() {
  // moveNStepsDC(motor1, 2000, CW, 200);
}
