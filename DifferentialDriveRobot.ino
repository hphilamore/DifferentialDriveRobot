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
#include <PID_v1.h>

// define variables
#define LEFT 0
#define RIGHT 1
#define WHITE 0
#define BLACK 1
#define TURN_LEFT 0
#define TURN_RIGHT 1
#define CHECK_LINE 2
#define GO_STRAIGHT 3
#define FRONT_OBSTACLE 4
#define MAX_DISTANCE 200      // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

// define pin numbers
#define TRIGGER_PIN_SIDE  12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN_SIDE     12  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define TRIGGER_PIN_FRONT  8  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN_FRONT     8  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define RED_PIN           A0 
#define GREEN_PIN         A1 
#define IRRight           13
#define IRLeft             4

// object avoidance parameters
const float pi = 3.142;
const float DistanceFromWall = 15;
const float DistanceFromObstacle = 17;

// PID Tuning parameters
float Kp=1; //Initial Proportional Gain 
float Ki=0; //Initial Integral Gain 
float Kd=0; //Initial Differential Gain

// PID controller varables
double Setpoint, Input, Output, OutputLeft, OutputRight; 
//const int sampleRate = 1;       // Variable that determines how fast our PID loop runs

// encoder variables
float coder[2] = {0,0};
float rps[2] = {0,0};
float radps[2] = {0,0}; 

// switch case variable
int mode;

// untrasonic sensor 
NewPing side_sonar(TRIGGER_PIN_SIDE, ECHO_PIN_SIDE, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing front_sonar(TRIGGER_PIN_FRONT, ECHO_PIN_FRONT, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

// PID controller
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);    

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

// PID controller
int setPoint = 35;
int range = 15;
int maxDistance = setPoint + range;
int minDistance = setPoint - range;
int minSpeed = 40;
int maxSpeed = 100;
int Distance;

// Line follower
int speedLow = 80;
int speedHigh = 120;

// Tachometer
int TachoIncrement = 500;   // the time (in mS) increment to record the encoder output for before outputting to serial 
long DrivePeriod;
unsigned long DriveStartTime;
unsigned long timer = 0;               
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
    pinMode(IRLeft, INPUT);
    pinMode(IRRight, INPUT);    
    pinMode(RED_PIN, OUTPUT);
    pinMode(GREEN_PIN, OUTPUT);
    
    Setpoint = map(setPoint, minDistance, maxDistance, minSpeed, maxSpeed);
    myPID.SetMode(AUTOMATIC); //Turn on the PID loop 
    //myPID.SetSampleTime(sampleRate); //Sets the sample rate
    myPID.SetOutputLimits(minSpeed, maxSpeed);

    Serial.print("Setpoint = "); 
    Serial.print(Setpoint);
    Serial.print("\n");    
}

void loop() 
{

  Sense();

  Serial.print(digitalRead(IRLeft));
  Serial.print('\t');
  Serial.print(digitalRead(IRRight));
  Serial.print('\n');

  switch (mode)
  {
       
    case TURN_RIGHT:
      turn_right(); 
       break;

    case TURN_LEFT:
       turn_left();  
       break;
       
    case GO_STRAIGHT:
       drive_straight(); 
       break; 
       
    case CHECK_LINE:
       check_line();
       break;

    case FRONT_OBSTACLE:
       front_obstacle();
       break;
  }

  //  FollowWallPID();

} 

//-------------------------------------------------------------------------------------------
  void LwheelSpeed()
{
  coder[LEFT] ++;  //count the left wheel encoder interrupts
}

//-------------------------------------------------------------------------------------------

void RwheelSpeed()
{
  coder[RIGHT] ++; //count the right wheel encoder interrupts
}

//-------------------------------------------------------------------------------------------

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
  
    DriveStartTime = millis();   
            
    while ((millis() - DriveStartTime) < DrivePeriod)         
    {        
      analogWrite(enA, leftMotorSpeed);
      analogWrite(enB, rightMotorSpeed); 
      
      //tachometer(leftMotorSpeed, rightMotorSpeed);  
      //sonar();

//        Serial.print(side_sonar.ping_cm());
//        Serial.print('\t');
//        Serial.print(front_sonar.ping_cm());  
//        Serial.println('\t');
      
     }

    }
    
//------------------------------------------------------------------------------------------- 
 
 void tachometer(float leftMotorSpeed, float rightMotorSpeed)
 {
//      if(millis() - timer > 500)
      if(millis() - timer > TachoIncrement)
      {      

        T = float(millis() - timer);

        radps[LEFT] = 2*pi*1000*float(coder[LEFT])/(EnRes * T); 
        radps[RIGHT] = 2*pi*1000*float(coder[RIGHT])/(EnRes * T);

//        Serial.print(leftMotorSpeed);
//        Serial.print("\t");        
////      Serial.print(rps[LEFT]);   
//        Serial.print(radps[LEFT]);         
//        Serial.print("\t");
//        Serial.print(rightMotorSpeed);
//        Serial.print("\t");
////      Serial.println(rps[RIGHT]);
//        Serial.println(radps[RIGHT]);     
        
        coder[LEFT] = 0;                 //clear the data buffer
        coder[RIGHT] = 0;       
        
        timer = millis();
    }    
  }
  
//------------------------------------------------------------------------------------------- 

  void FollowWallPID()
  {

    if((front_sonar.ping_cm() < DistanceFromObstacle)&&(front_sonar.ping_cm() > 0))
    {        
      //Drive(0,255,100);
      Drive(-100,100,500);   
      digitalWrite(GREEN_PIN, HIGH); 
      digitalWrite(RED_PIN, HIGH); 
    }  

    int D = side_sonar.ping_cm(); 
    Distance = (D < 1) ? maxDistance : D;
    Input = map(Distance, minDistance, maxDistance, minSpeed, maxSpeed); 
    myPID.Compute();  
    OutputLeft = 2 * Setpoint - Output;
    OutputRight = Output;

    Drive(OutputLeft, OutputRight, 100);

    if (OutputLeft > OutputRight)
    {
     digitalWrite(RED_PIN, LOW); 
     digitalWrite(GREEN_PIN, HIGH);   
    }

    if (OutputLeft < OutputRight)
    {
     digitalWrite(GREEN_PIN, LOW); 
     digitalWrite(RED_PIN, HIGH);   
    }        
      
    Serial.print("Sensor = "); 
    Serial.print(Distance); 
    Serial.print(" Input = "); 
    Serial.print(Input); 
    Serial.print(" Output = "); 
    Serial.print(Output); 
    Serial.print(" Output = "); 
    Serial.print(2 * Setpoint - Output); 
    Serial.print("\n"); 
   
  }

  //-------------------------------------------------------------------------------------------

  
  void Sense()
  {
    
  if ((front_sonar.ping_cm() < DistanceFromObstacle) &&(front_sonar.ping_cm() > 0))
  
    { 
     mode = FRONT_OBSTACLE;
    }

  else if ((digitalRead(IRLeft) == WHITE) && digitalRead(IRRight) == BLACK)
  
    {
     //Drive(?, ?, ?);  
     mode = TURN_RIGHT;
    }  

  else if ((digitalRead(IRLeft) == BLACK) && digitalRead(IRRight) == WHITE)
   
    { 
     //Drive(?, ?, ?);  
     mode = TURN_LEFT;
    }  

  else if ((digitalRead(IRLeft) == BLACK) && digitalRead(IRRight) == BLACK)
  
    {        
      //Drive(?, ?, ?);  
     mode = CHECK_LINE;
    }

  else

  {
    mode = GO_STRAIGHT;
  }
  
 }

 //-------------------------------------------------------------------------------------------

 void drive_straight()
 {
  Drive(speedHigh, speedHigh, 10); 
  analogWrite(RED_PIN, 0); 
  analogWrite(GREEN_PIN, 0);
 }

 //-------------------------------------------------------------------------------------------

 void turn_left()
 {
  Drive(speedLow, speedHigh, 10); 
  analogWrite(RED_PIN, 0); 
  analogWrite(GREEN_PIN, 255);
 }

 //-------------------------------------------------------------------------------------------

 void turn_right()
 {
  Drive(speedHigh, speedLow, 10); 
  analogWrite(RED_PIN, 255); 
  analogWrite(GREEN_PIN, 0);
 }

 //-------------------------------------------------------------------------------------------
 
 void check_line()
 {
  // move forward by one step 
  Drive(speedLow, speedLow, 500);
  analogWrite(RED_PIN, 255); 
  analogWrite(GREEN_PIN, 255);

  // check line again
  if ((digitalRead(IRLeft) == BLACK) && digitalRead(IRRight) == BLACK)
  
    {
      turn_around();
    }
   else
   
   {
     drive_straight();
   }
 }
 
 //-------------------------------------------------------------------------------------------

 void turn_around()
 {
  Drive(speedLow, -speedLow, 3000);
 }

 //-------------------------------------------------------------------------------------------

 void front_obstacle()
 {
  Drive(0, 0, 2000);
  analogWrite(RED_PIN, 255); 
  analogWrite(GREEN_PIN, 255);
 }
  
  

