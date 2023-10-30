/*!
 * @file  DRI0043.ino
 * @brief TB6600 arduino Stepper Motor Driver is an easy-to-use professional stepper motor driver, which could control a two-phase stepping motor.
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license  The MIT License (MIT)
 * @author  DFRobot
 * @version  V1.0
 * @date  2023-08-03
 */
int sdelay = 3000;
int PUL=7; //define Pulse pin
int DIR=6; //define Direction pin
int ENA=5; //define Enable Pin
void setup() {
  pinMode (PUL, OUTPUT); // this is for doing steps. A signal change = step taken.
  pinMode (DIR, OUTPUT); // this is for the direction. HIGH spins one dir, LOW spins in another
  pinMode (ENA, OUTPUT); // HIGH = enable driver (holds position if not stepped), LOW = disable driver

}

void loop() {
  for (int i=0; i<100; i++)    //Forward 5000 steps
  {
    digitalWrite(DIR,LOW);
    digitalWrite(ENA,HIGH);
    digitalWrite(PUL,HIGH);
    delayMicroseconds(sdelay);
    digitalWrite(PUL,LOW);
    delayMicroseconds(sdelay);
  }
  delay(1000);
  for (int i=0; i<100; i++)   //Backward 5000 steps
  {
    digitalWrite(DIR,HIGH);
    digitalWrite(ENA,HIGH);
    digitalWrite(PUL,HIGH);
    delayMicroseconds(sdelay);
    digitalWrite(PUL,LOW);
    delayMicroseconds(sdelay);
  }
    delay(1000);
}
