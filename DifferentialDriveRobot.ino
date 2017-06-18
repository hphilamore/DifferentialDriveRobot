#include <Servo.h>

Servo LeftServo;
Servo RightServo;

// the control signal for the left servo
const int leftServoPin = 2;

// the control signal for the right servo
const int rightServoPin = 4;

void setup()
{
   LeftServo.attach(leftServoPin);
   RightServo.attach(rightServoPin);
}

int leftServoSpeed = 90;
int rightServoSpeed = 0;

void loop() {
//  
//  if (leftServoSpeed < 0) {leftServoSpeed = 0;}     
//  else if (leftServoSpeed > 180) {leftServoSpeed = 180;}
//
//  if (rightServoSpeed < 0) {rightServoSpeed = 0;}     
//  else if (rightServoSpeed > 180) {rightServoSpeed = 180;}

  //Send the command to the servos
  LeftServo.write(leftServoSpeed);
  RightServo.write(rightServoSpeed);
  delay(1000);
}


