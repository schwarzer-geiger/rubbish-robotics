#include <AFMotor.h>
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
  dc(int powerPin, int dirPin, int enc1Pin, int enc2Pin)
    : powerPin(powerPin), dirPin(dirPin), enc(enc1Pin, enc2Pin) {
  }

  // Moves motor by nSteps encoder steps into direction dir (CW or CCW) at speed 'speed'.
  void moveNSteps(int nSteps, int dir, int speed) {
    // start with motor off
    analogWrite(powerPin, 0);

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
    // If direction pin is high, speed has to be inverted
    if (dir == CW) {
      analogWrite(powerPin, 255 - speed);
      digitalWrite(dirPin, HIGH);
    } else {
      analogWrite(powerPin, speed);
      digitalWrite(dirPin, LOW);
    }

    // check whether the number of encoder steps passed 'stepsRun' has reached the desired number of steps 'nSteps' continuously
    while (stepsRun < nSteps) {
      currentSignal = enc.read();
      if (currentSignal != lastSignal) {
        stepsRun++;
        lastSignal = currentSignal;
      }
    }

    analogWrite(powerPin, 0);
    enc.write(0);
    Serial.println("Motor stopped");

    if (dir == CW) {
      position += stepsRun;
    } else {
      position -= stepsRun;
    }

    if (SERIAL_PRINT) {
      Serial.println("Initial position: " + initPosition);
      Serial.println("Target Position: " + targetPosition);
      Serial.println("Actual position: " + position);
    }
  }

  int powerPin;
  int dirPin;
  Encoder enc;
};

void setup() {
  // put your setup code here, to run once:
  dc m1(1, 2, 3, 4);
  m1.moveNSteps(1000, CW, 50);
}

void loop() {
  // put your main code here, to run repeatedly:
}
