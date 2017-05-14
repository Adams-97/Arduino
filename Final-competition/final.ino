///////////////////////////  
// ELEC299 - Winter2017  
////////////////////////////  
// “Final 299 Code (Bluetooth Only)”  
//  
// Group #: 
//  
// Group Members:  
// 
// 
//
//  
///////////////////////////

//003CB8B11422 Bluetooth Module MAC Address

//Included Libraries
#include <Servo.h>
#include "QSerial.h"
QSerial IRSerial;


///Digital Pins///

//Pushbutton Pin
#define BUTTON 2

//Pan Pin
#define PAN_PIN 3

// Motor Pins
#define LD 4
#define LS 5
#define RS 6
#define RD 7

//Bumper Pins
#define RIGHT_BUMPER 8
#define LEFT_BUMPER 9

//Grip Pin
#define TILT_PIN 13
#define GRIP_PIN 10

//Encoder Pins
#define RIGHT_ENCODER 11
#define LEFT_ENCODER 12

//Tilt Pin
#define TILT_PIN 13


///Analog Pins///

//Infared Sensor
#define LTR A2  //Linetracker Right
#define LTC A0  //Linetracker Centre
#define LTL A1  //Linetracker Left

#define IR_SENSOR A4

//Clamper Sensor
#define SENSOR_PIN A3

// Predefined Constants for Threshold Signal
#define CTHRES 960
#define LTHRES 960
#define RTHRES 960
#define HI 120 //High MotorSpeed
#define LO 70 //Low MotorSpeed

// Attributes
int degree; // Degree for Claw

double motorSpeed; // Motors

int rightCount; //Encoders
int leftCount;
int rightFlag;
int leftFlag;

byte var; // Bluetooth Var

int ITrigger;
int isClosed = 0; // Toggle 

Servo servo1,servo2, servo3;

void setup() {
//Baudrate
Serial.begin(115200);

 
//Servo//
servo1.attach(PAN_PIN);
servo2.attach(TILT_PIN);
servo3.attach(GRIP_PIN);
pinMode(SENSOR_PIN, INPUT);
pinMode(BUTTON, INPUT);
servo1.write(90);
servo2.write(100);
servo3.write(0);
degree = 0;

// Transmitter & Reciever//
//IRSerial.attach(RXPIN, -1);



pinMode(RD, OUTPUT);
pinMode(LD, OUTPUT);
pinMode(BUTTON, INPUT);

motorSpeed = 0.5 * 255;
rightCount = 0;
 leftCount = 0;
  rightFlag = 0;
  leftFlag = 0;
  ITrigger = 0;
  
while(digitalRead(BUTTON) == HIGH){}
while(digitalRead(BUTTON) == LOW){}

}



 
void loop() 
{
// Main Function that runs through the Programs
ITrigger = 2;
while(ITrigger = 2){
  
if(Serial.available())
{
var=Serial.read();

// Main Function that goes towards beacon 0 and resets in front of the pipe
if(var=='L'){
Serial.print("0");
IanChang();
Adams();
TurnL();
leftCount= 0;
rightCount=0;
IanChang();
while(true)
{
int bumpers = checkBumpers();
  if (bumpers == 3) 
  {
    servo3.write(120); //closed position
    isClosed = 1;
    delay(1000);
    servo2.write(150); 
    break;
  }
}
Back();
Turn180();
leftCount= 0;
rightCount=0;
HitsBlack();
IanChang();
Adams();
TurnR();
leftCount= 0;
rightCount=0;
HitsBlack();
Kingsley();
servo2.write(100);
}

// Main Function that goes towards beacon 1 and resets in front of the pipe
if(var=='C')
{
Serial.println("1");
Kingsley();
int bumpers = checkBumpers();
        
        if (bumpers == 3)
        {
          servo3.write(120); //closed position
          isClosed = 1;
          delay(1000);
          servo2.write(150); 
        }
Back();
Turn180();
leftCount= 0;
rightCount=0;
HitsBlack();
Kingsley();
servo2.write(100);
  }
  
// Main Function that goes towards beacon 2 and resets in front of the pipe
  else if(var=='R'){
  Serial.print("2");
  IanChang();
  Adams();
  TurnL();
  leftCount= 0;
  rightCount=0;
  IanChang();
  while(true)
  {
  int bumpers = checkBumpers();
    if (bumpers == 3) 
    {
     servo3.write(120); //closed position
      isClosed = 1;
           
      delay(1000);
      servo2.write(150); 
      break;
    }
  }
  Back();
  Turn180();
  leftCount= 0;
  rightCount=0;
  HitsBlack();
  IanChang();
  Adams();
  TurnR();
  leftCount= 0;
  rightCount=0;
  HitsBlack();
  Kingsley();
  servo2.write(100);
  }

delay(50);
}
}
}


// If stop when the Middle IR Sensor see a black line
void HitsBlack()
{
  while(true)
  {
  int centerVal = analogRead(LTC);
  if(centerVal>CTHRES)
  {
   analogWrite(RS,0);
   analogWrite(LS,0);
   break;
  }
  }

}

// Stop
void Stop()
{
  int centerVal = analogRead(LTC);
  while(centerVal<CTHRES)
  {
  analogWrite(RS, 0);
  analogWrite(LS, 0);
  }
}


// Turn robot 90 deg left
void TurnL()
{ 
 while (leftCount < 6 && rightCount < 6) 
 {
    updateCount();
    digitalWrite(RD, HIGH);
    digitalWrite(LD, LOW);
    analogWrite(RS, motorSpeed);
    analogWrite(LS, motorSpeed);
  }
}

// Turn robot 90 deg right
void TurnR()
{
 while (leftCount < 6 && rightCount < 6) 
 {
    updateCount();
    digitalWrite(RD, LOW);
    digitalWrite(LD, HIGH);
    analogWrite(RS, motorSpeed);
    analogWrite(LS, motorSpeed);

 }


}
// Turn robot 180 deg right
void Turn180()
{
   while (leftCount < 11 && rightCount < 11) 
 {
    updateCount();
    digitalWrite(RD, LOW);
    digitalWrite(LD, HIGH);
    analogWrite(RS,0.9*motorSpeed);
    analogWrite(LS,0.9*motorSpeed);

 }


}

//Line Tracking mainly used to grab ball in beacons 0 or 2
void IanChang()
{

 while( true )
{
    int leftVal = analogRead(LTL);
    int rightVal = analogRead(LTR);
    int centerVal = analogRead(LTC);
    digitalWrite(RD, HIGH);
    digitalWrite(LD, HIGH);

 if (analogRead(LTC)>CTHRES && analogRead(LTR)>RTHRES && analogRead(LTL)>LTHRES)
 {
 analogWrite(RS,0);
 analogWrite(LS,0);
 break;
 }
 else if(  centerVal>CTHRES )
 {
 analogWrite(RS,120);
 analogWrite(LS,120);  
 }
 
 //Veering right, move power to right motor
 else if(leftVal>LTHRES)
 {
 analogWrite(RS,HI);
 analogWrite(LS,LO);
 }
 
 else if(leftVal>LTHRES && centerVal>CTHRES)
 {
 analogWrite(RS,HI);
 analogWrite(LS,LO);
 }
 
 //Veering left, move power to left motor
 else if(rightVal>RTHRES)
 {
 analogWrite(RS,LO);
 analogWrite(LS,HI);
 }
 
 else if( centerVal>CTHRES && rightVal>RTHRES)
 {
 analogWrite(RS,LO);
 analogWrite(LS,HI);
 }

 // Motors Stop if bumpers are hit or if the IR Sensor reads that it's close to the wall
 int bumpers = checkBumpers();
 if(bumpers==3 || isClosed == 1 && analogRead(IR_SENSOR) >= 550)
 {
  analogWrite(RS,0);
  analogWrite(LS,0);
  break;
 }
 }
 }

// Move up a small distance forward
void Adams()
{
  digitalWrite(LD,HIGH);
  digitalWrite(RD,HIGH);
  analogWrite(LS,motorSpeed);
  analogWrite(RS,motorSpeed);
  delay(200);
}

//Line Tracking mainly used to grab ball in beacon 1
void Kingsley()
{
while(true)
{
    int leftVal = analogRead(LTL);
    int rightVal = analogRead(LTR);
    int centerVal = analogRead(LTC);
    digitalWrite(RD, HIGH);
    digitalWrite(LD, HIGH);
    
 if( centerVal>CTHRES )
 {
 analogWrite(RS,120);
 analogWrite(LS,120);  
 }
 
 //Veering right, move power to right motor
 else if(leftVal>LTHRES)
 {
 analogWrite(RS,HI);
 analogWrite(LS,LO);
 }
 
 else if(leftVal>LTHRES && centerVal>CTHRES)
 {
 analogWrite(RS,HI);
 analogWrite(LS,LO);
 }
 
 //Veering left, move power to left motor
 else if(rightVal>RTHRES)
 {
 analogWrite(RS,LO);
 analogWrite(LS,HI);
 }
 
 else if( centerVal>CTHRES && rightVal>RTHRES)
 {
 analogWrite(RS,LO);
 analogWrite(LS,HI);
 }

 // If all the IR sensors are on black the robots moves slowly
 else if( analogRead(LTC)>CTHRES && analogRead(LTR)>RTHRES && analogRead(LTL)>LTHRES)
 {
 analogWrite(RS,LO);
 analogWrite(LS,LO);
 }

 //If both bumpers are hit then stop the motor
 int bumpers = checkBumpers();
 if(bumpers==3)
 {
 analogWrite(RS,0);
 analogWrite(LS,0);
 break;
 }
 
 // If the Gripper is close and the IR_Sensor distance is 550 then open gripper
 else if  (isClosed == 1 && analogRead(IR_SENSOR) >= 550)
 {
    analogWrite(RS,0);
    analogWrite(LS,0);
    delay(400);
    servo3.write(0);
    delay(1000);
    Back();
    delay(100);
    Turn180();
    leftCount= 0;
    rightCount=0;
    HitsBlack();
    isClosed = 0;
    break;  
 }
 }
 }


// Check bumpers to see which ones are hit
int checkBumpers() {
  delay(50);
  if (digitalRead(RIGHT_BUMPER) == LOW && digitalRead(LEFT_BUMPER) == LOW) {
    return 3;
  } else if (digitalRead(RIGHT_BUMPER) == HIGH && digitalRead(LEFT_BUMPER) == LOW) {
    return 1;
  } else if (digitalRead(RIGHT_BUMPER) == LOW && digitalRead(LEFT_BUMPER) == HIGH) {
    return 2;
  } else {
    return 0;
  }
}


//Check the IRSensor to see if robot is on black
void CheckBlack()
{
  while(true)
  {
    int centerVal = analogRead(LTC);
    if(centerVal>CTHRES)
    {
      break;
    }
  }
}


// These are the encoders update counter
void updateCount() {
if (digitalRead(RIGHT_ENCODER) == HIGH) 
{
rightFlag = 1;
} else if (digitalRead(RIGHT_ENCODER) == LOW) 
{
    if (rightFlag == 1) 
    {
      rightCount++;
      rightFlag = 0;
    }
  }
  if (digitalRead(LEFT_ENCODER) == HIGH) {
    leftFlag = 1;
  } else if (digitalRead(LEFT_ENCODER) == LOW) {
    if (leftFlag == 1) {
      leftCount++;
      leftFlag = 0;
    }
  }
}



// Reverse on 80 speed for 400ms
void Back()
{
digitalWrite(LD,LOW);
digitalWrite(RD,LOW);
analogWrite(LS,80);
analogWrite(RS,80);
delay(400);
}






