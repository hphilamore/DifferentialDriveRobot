/******************************************************************************
Code based on:
Differential steering with continuous rotation servos and Arduino Uno:
http://42bots.com/tutorials/differential-steering-with-continuous-rotation-servos-and-arduino/
L298 motor driver:
https://tronixlabs.com.au/news/tutorial-l298n-dual-motor-controller-module-2a-and-arduino/
Optical tachometer:
https://learn.sparkfun.com/tutorials/qrd1114-optical-detector-hookup-guide?_ga=1.57060067.834043405.1459523277
www.instructables.com/id/Arduino-Based-Optical-Tachometer/
4 --> left servo
3 --> right servo
2 --> Photodetector Collector pin
******************************************************************************/
#include <Servo.h>

const float pi = 3.142;

// the number of pulses on the encoder wheel
float EnRes = 5;

// the time (in mS) increment to record the encoder output for before outputting to serial 
//int TachoIncrement = 2000;
int TachoIncrement = 1000;

// timer and counter variables
// sets the period an operation lasts for in mS
long DrivePeriod;
long TachoPeriod;
long OpPeriod;
volatile unsigned long minElapsed = 100;
volatile unsigned long maxElapsed = 2500;
volatile unsigned long elapsedTime;
volatile unsigned long previousTime;
unsigned long DriveStartTime;
unsigned long TachoStartTime;
unsigned long count;
unsigned long time;
unsigned long timeold;
float timer;    
float rps;
float radps;
int PWM_signal;
int PWM_min;
int PWM_max;

Servo LeftServo;
Servo RightServo;

// the control signal for the left servo
const int leftServoPin = 4;

// the control signal for the right servo
const int rightServoPin = 3;

void setup()
{
   LeftServo.attach(leftServoPin);
   RightServo.attach(rightServoPin);
   
   Serial.begin(9600);
   
   //Interrupt 0 is digital pin 2, so that is where the IR detector is connected
   //Triggers on RISING (change from HIGH to LOW)
   attachInterrupt(0, rps_fun, FALLING);
  
   count = 0;
   timeold = 0;
   
   Serial.print("PWM");
   Serial.print("\t");
   Serial.print("rps");
   Serial.print("\t");
   Serial.println("radps");
}

void loop() 
{

  // Stop      
  Drive(0,180,10000);					
//
//   for (int i = 0; i < 190; i+=10)
//  {
//    Serial.print(i);
//     Drive(i,180-i,3000);    
//  }
			
}

void Drive(int leftServoSpeed, int rightServoSpeed, long DrivePeriod) 
  {
  DriveStartTime = millis() ; 
//  Serial.println(millis());	
//  Serial.println(DriveStartTime);	
//  Serial.println(DrivePeriod);	
//  Serial.println("-1");				        
  while ((millis() - DriveStartTime) < DrivePeriod)	        
  {
    if (leftServoSpeed < 0) {leftServoSpeed = 0;}     
    else if (leftServoSpeed > 180) {leftServoSpeed = 180;}
    
    if (rightServoSpeed < 0) {rightServoSpeed = 0;}     
    else if (rightServoSpeed > 180) {rightServoSpeed = 180;}
  
    //Send the command to the servos
    LeftServo.write(leftServoSpeed);
    RightServo.write(rightServoSpeed);
    
    tachometer(leftServoSpeed, DrivePeriod);
    }
  }

void rps_fun()
 {
   
   
   
  elapsedTime = millis() - previousTime;
  if (elapsedTime < minElapsed )  //false interrupt
  {
    return;
  }
  if (elapsedTime >= minElapsed && elapsedTime <= maxElapsed )  //in range
  {
    previousTime = millis();
    count +=1; 
  }
  if (elapsedTime > maxElapsed )  //timeout
  {
    previousTime = millis();
    //timeout = 1;
  }  
   
   //count +=1;      
 }
 
 void tachometer(int PWM_val, long OpPeriod)
 {
   // The period of the operation to measure
   OpPeriod = int(OpPeriod);
  
   // time for maximum number of tachometer readings without exceeding Drive period 
   TachoPeriod = long((OpPeriod/TachoIncrement)*TachoIncrement); 
   
   // reset the counter and timer in case the speed has changed since the last count
   timeold = millis();
   count = 0; 
   //Serial.print("1");
   TachoStartTime = millis() ; 				// record the operation start time

   while ((millis() - TachoStartTime) < TachoPeriod)    // while time elapsed < desired period  
  {
      // the delay determines the total recording time
      delay(TachoIncrement);    
      //Serial.print("2");
      // don't process interrupts during calculations
      detachInterrupt(0);  
    
      // calculate the rotational speed  
      time = millis();
      timer = float(time - timeold);
      rps = 1000*count/(EnRes * timer); 
      radps = 2*pi*1000*count/(EnRes * timer); 
      //Serial.print(count);
      //Serial.print("\t");
      //Serial.print("3");
      // reset the counter and timer to begin the next increment
      timeold = millis();   
      count = 0;   
      
      // print everything
      Serial.print(PWM_val);
      Serial.print("\t");
      Serial.print(rps);
      Serial.print("\t");
      Serial.println(radps);
      
      //Restart the interrupt processing
      previousTime = millis();
      attachInterrupt(0, rps_fun, FALLING);
      
    
    }    
  }

