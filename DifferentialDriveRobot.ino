//********************************************************************
//Differential steering with continuous rotation servos and Arduino Uno
//Version: 1.0
//Date: January 2014
//Author: Stan
//Web-Site: http://42bots.com/tutorials/differential-steering-with-continuous-rotation-servos-and-arduino/
//********************************************************************
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

void loop() {
  //Drive forward for 3 seconds
   Drive(110, 70);
   delay(1000);
   
   Drive(70, 110);
   delay(1000);
   
   Drive(90, 90);
   delay(1000);
}



void Drive(int leftServoSpeed, int rightServoSpeed) {

  if (leftServoSpeed < 0) {leftServoSpeed = 0;}     
  else if (leftServoSpeed > 180) {leftServoSpeed = 180;}

  if (rightServoSpeed < 0) {rightServoSpeed = 0;}     
  else if (rightServoSpeed > 180) {rightServoSpeed = 180;}

  //Send the command to the servos
  LeftServo.write(leftServoSpeed);
  RightServo.write(rightServoSpeed);
  
}


