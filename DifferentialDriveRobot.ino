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
#include <NewPing.h>

#define LEFT 0
#define RIGHT 1

float coder[2] = {0,0};
float rps[2] = {0,0};
float radps[2] = {0,0};

#define TRIGGER_PIN_SIDE  12//3  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN_SIDE     12  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define TRIGGER_PIN_FRONT  8  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN_FRONT     8//4  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define RED_PIN  A0  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define GREEN_PIN  A1  // Arduino pin tied to echo pin on the ultrasonic sensor.

NewPing side_sonar(TRIGGER_PIN_SIDE, ECHO_PIN_SIDE, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing front_sonar(TRIGGER_PIN_FRONT, ECHO_PIN_FRONT, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

const float pi = 3.142;
const float DistanceFromWall = 15;
const float DistanceFromObstacle = 15;

// the number of pulses on the encoder wheel
float EnRes = 10;

// motor one
int enA = 5;
int in1 = 7;
int in2 = 6;
// motor two
int enB = 9;
int in3 = 11;
int in4 = 10;

// the time (in mS) increment to record the encoder output for before outputting to serial 
int TachoIncrement = 500;

//// timer and counter variables
//// sets the period an operation lasts for in mS
long DrivePeriod;
unsigned long DriveStartTime;
//unsigned long Time;
//unsigned long timer;
unsigned long timer = 0;                //print manager timer
float T;    

void setup()
{
   
   Serial.begin(9600);
   
  //Interrupt 0 is digital pin 2,Interrupt 1 is digital pin 3. 
  attachInterrupt(LEFT, LwheelSpeed, FALLING);    //init the interrupt mode for the digital pin 2
  attachInterrupt(RIGHT, RwheelSpeed, FALLING);   //init the interrupt mode for the digital pin 3
   
    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    
    pinMode(RED_PIN, OUTPUT);
    pinMode(GREEN_PIN, OUTPUT);
}

void loop() 
{
followWall();
//  Drive(200, -200, 2000);

// for (int i = 0; i < 255; i = i + 10)  
//  {
//    Drive(i, i, 2000); 
//    
////        Serial.print(side_sonar.ping_cm());
////        Serial.print('\t');
////        Serial.print(front_sonar.ping_cm()); 
////        Serial.println('\t'); 
//  }
//  // decelerate from maximum speed to zero
//  for (int i = 255; i >= 0; i = i - 10)
//  {
//    Drive(i, i, 2000); 
    
//        Serial.print(side_sonar.ping_cm());
//        Serial.print('\t');
//        Serial.print(front_sonar.ping_cm());  
//        Serial.println('\t');
  }  


  void LwheelSpeed()
{
  coder[LEFT] ++;  //count the left wheel encoder interrupts
}


void RwheelSpeed()
{
  coder[RIGHT] ++; //count the right wheel encoder interrupts
}

void Drive(int leftMotorSpeed, int rightMotorSpeed, long DrivePeriod) 

  {  


    
     // set motor direction
    if(leftMotorSpeed < 0) 
    {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      leftMotorSpeed = abs(leftMotorSpeed);

    }
    
    else
    {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
    }
  
    // set motor direction
    if(rightMotorSpeed < 0) 
    {
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      rightMotorSpeed = abs(rightMotorSpeed);
    }
    
    else
    {
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
    }
  
    DriveStartTime = millis() ; 	
            
    while ((millis() - DriveStartTime) < DrivePeriod)	        
    {        
      analogWrite(enA, leftMotorSpeed);
      analogWrite(enB, rightMotorSpeed); 
      tachometer(leftMotorSpeed, rightMotorSpeed);  
      //sonar();

//        Serial.print(side_sonar.ping_cm());
//        Serial.print('\t');
//        Serial.print(front_sonar.ping_cm());  
//        Serial.println('\t');
      
     }

    }
 
 
 void tachometer(float leftMotorSpeed, float rightMotorSpeed)
 {
//      if(millis() - timer > 500)
      if(millis() - timer > TachoIncrement)
      {      
//        Serial.print(coder[LEFT]);
//        Serial.print("\t");
//        Serial.println(coder[RIGHT]);

//        Time = millis();
//        T = float(Time - timer);

        T = float(millis() - timer);
 
//      rps[LEFT] = 1000*float(coder[LEFT])/(EnRes * T); 
//      rps[RIGHT] = 1000*float(coder[RIGHT])/(EnRes * T); 

        radps[LEFT] = 2*pi*1000*float(coder[LEFT])/(EnRes * T); 
        radps[RIGHT] = 2*pi*1000*float(coder[RIGHT])/(EnRes * T);

        Serial.print(leftMotorSpeed);
        Serial.print("\t");        
//      Serial.print(rps[LEFT]);   
        Serial.print(radps[LEFT]);         
        Serial.print("\t");
        Serial.print(rightMotorSpeed);
        Serial.print("\t");
//      Serial.println(rps[RIGHT]);
        Serial.println(radps[RIGHT]);

//        Serial.print('\t');
//        Serial.print(side_sonar.ping_cm());
//        Serial.print('\t');
//        Serial.println(front_sonar.ping_cm());      
        
        coder[LEFT] = 0;                 //clear the data buffer
        coder[RIGHT] = 0;       
        
        timer = millis();
    }    
  }

  void followWall()
  {
//    Serial.print(side_sonar.ping_cm());
//    Serial.print('\t');
//    Serial.print(front_sonar.ping_cm());  
//    Serial.println('\t');

  delay(50);

  // if we are at a corner or away from the wall but heading for an obstacle, turn left

  
  if((front_sonar.ping_cm() < DistanceFromObstacle)&&(front_sonar.ping_cm() > 0))
  {        
   
    //Drive(0,255,100);
    Drive(-100,100,200);   
    Serial.print("front");
     Serial.print("\t"); 
    Serial.println(front_sonar.ping_cm()); 
    digitalWrite(GREEN_PIN, HIGH); 
    digitalWrite(RED_PIN, HIGH); 
  }  

  //if ((side_sonar.ping_cm() < DistanceFromWall)&& (side_sonar.ping_cm() > 0))
  
  else if((side_sonar.ping_cm() < DistanceFromWall)&& (side_sonar.ping_cm() > 0))
  {    
    Drive(90,110,10);     
    //Drive(255,0,10); 
    Serial.print("close");  
    Serial.print("\t"); 
    Serial.println(side_sonar.ping_cm()); 
    digitalWrite(GREEN_PIN, LOW); 
    digitalWrite(RED_PIN, HIGH); 
//     analogWrite(GREEN_PIN, 0); 
//    analogWrite(RED_PIN, 255);  
  }

  // if we are too far, turn in to the wall
  
  else
  {
    Drive(130, 90, 10);
    //Drive(0,255,10);  
    Serial.print("far");  
    Serial.print("\t"); 
    Serial.println(side_sonar.ping_cm()); 
//    analogWrite(RED_PIN, 0); 
//    analogWrite(GREEN_PIN, 255);  
    digitalWrite(RED_PIN, LOW); 
    digitalWrite(GREEN_PIN, HIGH);  
  }
    
  }
