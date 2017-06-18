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
#include <NewPing.h>

#define TRIGGER_PIN_SIDE  12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN_SIDE     11  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define TRIGGER_PIN_FRONT  10  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN_FRONT     9  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define RED_PIN  5  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define GREEN_PIN     6  // Arduino pin tied to echo pin on the ultrasonic sensor.

NewPing side_sonar(TRIGGER_PIN_SIDE, ECHO_PIN_SIDE, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing front_sonar(TRIGGER_PIN_FRONT, ECHO_PIN_FRONT, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

const float pi = 3.142;
const float DistanceFromWall = 15;
const float DistanceFromObstacle = 15;

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

   pinMode(RED_PIN, OUTPUT);
   pinMode(GREEN_PIN, OUTPUT);
   
   
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

   delay(5000);
}

void loop() 
{
  delay(30);  // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.

//  // 9v battery
//  if(sonar.ping_cm() < DistanceFromWall)
//  {        // if we are too close, turn away
//    Drive(70,150,20);  
//  }
//  else
//  {
//    Drive(30,110,20);
//  }

//  if(sonar.ping_cm() < DistanceFromWall)
//  {        // if we are too close, turn away
//    Drive(70,140,20);  
//  }
//  else
//  {
//    Drive(40,110,20);
//  }

//// 9V also ok
//  if(sonar.ping_cm() < DistanceFromWall)
//  {        // if we are too close, turn away
//    Drive(60,180,20); 
//    digitalWrite(GREEN_PIN, LOW); 
//    digitalWrite(RED_PIN, HIGH);
//  }
//  else
//  {
//    Drive(0,120,20);
//    digitalWrite(RED_PIN, LOW); 
//    digitalWrite(GREEN_PIN, HIGH);
//  }

//// 9V CORRECT DIRECTION FOR FRONT SENSOR
//  if(sonar.ping_cm() < DistanceFromWall)
//  {        // if we are too close, turn away
//    Drive(110,30,10);
//    
//    digitalWrite(GREEN_PIN, LOW); 
//    digitalWrite(RED_PIN, HIGH);
//  }
//  else
//  {
//    Drive(150,70,10);  
//    digitalWrite(RED_PIN, LOW); 
//    digitalWrite(GREEN_PIN, HIGH);
//  }


  // if we are at a corner or away from the wall but heading for an obstacle, turn left
  if((front_sonar.ping_cm() < DistanceFromObstacle)&&(front_sonar.ping_cm() > 0))
  {        // if we are too close, turn away
    //Drive(70,70,50);
    Drive(0,0,100);
    
    digitalWrite(GREEN_PIN, HIGH); 
    digitalWrite(RED_PIN, HIGH);
  }  

  // if we are too close, turn away
  //else if(side_sonar.ping_cm() < DistanceFromWall)
  else if((side_sonar.ping_cm() < DistanceFromWall)&& (side_sonar.ping_cm() > 0))
  {        
    Drive(110,40,10);
    
    digitalWrite(GREEN_PIN, LOW); 
    digitalWrite(RED_PIN, HIGH);
  }

  // if we are too far, turn in to the wall
  else
  {
    Drive(140,70,10);  
    digitalWrite(RED_PIN, LOW); 
    digitalWrite(GREEN_PIN, HIGH);
  }
  Serial.print(side_sonar.ping_cm());
  Serial.print('\t');
  Serial.println(front_sonar.ping_cm());
     
//  Serial.print(sonar.ping_cm()); // Send ping, get distance in cm and print result (0 = outside set distance range)
//  Serial.print("\t");
//  Serial.print(sonar.ping_cm()); // Send ping, get distance in cm and print result (0 = outside set distance range)
//  Serial.print("\t");
//  Serial.print(sonar.ping_cm()); // Send ping, get distance in cm and print result (0 = outside set distance range)
//  Serial.print("\t");
  				

			
}

void Drive(int leftServoSpeed, int rightServoSpeed, long DrivePeriod) 
  {
  DriveStartTime = millis() ; 
  
//  Serial.print(leftServoSpeed); // Send ping, get distance in cm and print result (0 = outside set distance range)
//  Serial.print("\t");
//  Serial.print(rightServoSpeed); // Send ping, get distance in cm and print result (0 = outside set distance range)
//  Serial.print("\t");
//  Serial.println(side_sonar.ping_cm()); // Send ping, get distance in cm and print result (0 = outside set distance range)
  //Serial.print("\t");			
          
  while ((millis() - DriveStartTime) < DrivePeriod)	        
  {
    if (leftServoSpeed < 0) {leftServoSpeed = 0;}     
    else if (leftServoSpeed > 180) {leftServoSpeed = 180;}
    
    if (rightServoSpeed < 0) {rightServoSpeed = 0;}     
    else if (rightServoSpeed > 180) {rightServoSpeed = 180;}
  
    //Send the command to the servos
    LeftServo.write(leftServoSpeed);
    RightServo.write(rightServoSpeed);
    
    //tachometer(leftServoSpeed, DrivePeriod);
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
  }  
   
 }
 
 void tachometer(int PWM_val, long OpPeriod)
 {
   // The period of the operation to measure
   OpPeriod = int(OpPeriod);
  
   // time for maximum number of tachometer readings without exceeding Drive period 
   TachoPeriod = long((OpPeriod/TachoIncrement)*TachoIncrement); 

   TachoStartTime = millis() ; 				// record the operation start time

   timeold = millis();
   
   count = 0; 

   while ((millis() - TachoStartTime) < TachoPeriod)    // while time elapsed < desired period  
  {
    if ((millis() - timeold) > TachoIncrement)
      {
         detachInterrupt(0);   
        // calculate the rotational speed  
        time = millis();
        timer = float(time - timeold);
        rps = 1000*count/(EnRes * timer); 
        radps = 2*pi*1000*count/(EnRes * timer); 
        // reset the counter and timer to begin the next increment
        timeold = millis();   
        count = 0; 
        //Restart the interrupt processing
        previousTime = millis();
        attachInterrupt(0, rps_fun, FALLING);
      }
                 
      //delay(50);  // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.    
      Serial.print(side_sonar.ping_cm()); // Send ping, get distance in cm and print result (0 = outside set distance range)
      Serial.print("\t");
      Serial.print(PWM_val);
      Serial.print("\t");
      Serial.print(rps);
      Serial.print("\t");
      Serial.println(radps);  
    }    
  }
