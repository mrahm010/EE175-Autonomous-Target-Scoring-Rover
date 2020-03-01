#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Arduino.h"
#include <limits.h>
#include <math.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Servo.h>
Adafruit_GPS GPS(&Serial2);
uint32_t timer = millis();

#define GPSECHO  true
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55);
adafruit_bno055_offsets_t calib;
sensors_event_t event;
//Chung
float originLat = 0;
float originLong = 0;
int originKey = 1;

float currlat1prec = 0;
float currlong1prec = 0;
float currlat1 = 0;
float currlong1 = 0;
// HUB
float lat2 = 33.9758;
float long2 = -117.3299;

float lat3 = 33.9757;
float long3 = -117.3299;

float lat4 = 33.9757;
float long4 = -117.3301;

/////////////////////////////////////////
//Global Variables
float magnetichead;
double bearing;
double distance;
double startOrientation = 0;
int key = 1;
int counter = 0;
//////////////
//servo
int myservoPIN  = 5;
////////
// Servo code & vibration
unsigned long duration;
int angle = 105;    // initial angle  for servo
int angleStep = 10;
int vibration_pin = 8;
const int minAngle = 0;
const int maxAngle = 106;
int targetStart = 1;
int targetHIT = 0;
int pos = 0;
int servoInit = 130;
int detecting = 1;
Servo myservo;
const int trig = 41;
const int echo = 40;

float distance,duration;
bool detect = false;
/* 
L298N H-Bridge driving DC motor on Arduino
*/
int ENA = 13; // MCU PWM Pin 10 to ENA on L298n Board
int IN1 = 12;  // MCU Digital Pin 9 to IN1 on L298n Board 
int IN2 = 11;  // MCU Digital Pin 8 to IN2 on L298n Board

int ENB = 4;  // MCU PWM Pin 5 to ENB on L298n Board
int IN3 = 10;  // MCU Digital pin 7 to IN3 on L298n Board
int IN4 = 9;  // MCU Digital pin 6 to IN4 on L298n Board

int GPSLED = 46; //GPS done calibrating LED to pin 46
int calibrationLED = 47; //orientation calibration LED to pin 47

void setup()
{
  Serial.begin(115200);
//  Serial.print("hello");
  ///Servo
  myservo.attach(myservoPIN);
  pinMode(vibration_pin, INPUT);
  if(targetStart) { // Initializes target to zero
    for(pos = 180; pos >= 0; pos -= 1) {
      myservo.write(pos);
      delay(15);
    }
  }
  Serial.println("Adafruit GPS library basic test!");
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  Serial3.println(PMTK_Q_RELEASE);

  delay(1000);
  //Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  //LED pins
  pinMode(calibrateLED,OUTPUT);
  pinMode(GPSLED,OUTPUT);
  
  pinMode(ENA, OUTPUT); //Set all the L298n Pin to output
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  if(!bno.begin(bno.OPERATION_MODE_COMPASS))
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
}
float anglecalc(float lat1, float lat2, float long1, float long2 ) {
  float dy = lat2 - lat1;

  float dx = cosf(PI / 180*lat1)*(long2 - long1);
  float angle = atan2(dx,dy)/ PI/2 *360;
  //Serial.print(angle);
  //Serial.print("\n");

  
  //Note: angle will be between -180 and 180 degrees
  //---Added by meh meh
  if(angle < 0) {
    angle = angle + 360;
  }
  //----meh meh
  return angle;
}
int GetCoordinates() {
  int confirm;
  
  char c = GPS.read();          
 if ((c) && (GPSECHO))                       //GPSECHO too continue updating coordinates
   // Serial.write(c);                      //output of all the NMEA
  if (GPS.newNMEAreceived()) {              //if nmea found,parse
    if (!GPS.parse(GPS.lastNMEA()))         // this also sets the newNMEAreceived() flag to false
      return 0;                               // we can fail to parse a sentence in which case we should just wait for another
  }
  if (timer > millis())  timer = millis();  //reset timer if needed
  if (millis() - timer > 2000) {
    timer = millis();                       // reset the timer
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix && ((int)GPS.fixquality >= 1)) {
      digitalWrite(GPSLED,HIGH);
      Serial.print("Location: ");
      Serial.print(GPS.latitudeDegrees, 6); 
      Serial.print(", ");
      Serial.print(GPS.longitudeDegrees, 6);
      Serial.print("\n");
      counter++;
      if (counter > 1) {
        if (originKey == 1) {
          originLat = floor(GPS.latitudeDegrees*10000 + 0.5)/10000;
          originLong = floor(GPS.longitudeDegrees*10000 + 0.5)/10000;
          //originLat = 33.9758;
          //originLong = -117.3302;
          originKey = 0;
          Serial.print("Origin: ");
          Serial.print(originLat, 4);
          Serial.print(originLong, 4);
        }
        currlat1prec = GPS.latitudeDegrees;
        currlong1prec = GPS.longitudeDegrees;
        currlat1 = floor(GPS.latitudeDegrees*10000 + 0.5)/10000;
        currlong1 = floor(GPS.longitudeDegrees*10000 + 0.5)/10000;
        counter = 0;
        return 1;
      }
      else {
        return 0;
      }
    }
  }
  return 0;
}
void GoToCoordinate(float latitude, float longitude) {
  char c;
  float functionAngle = 0;
  float currentLatprec = 0;
  float currentLongprec = 0;
  float currentLat = 0;
  float currentLong = 0;
  int coordinateReached = 0;
  float angle;
  float ex1;
  float ex2;
  float ex3;
  float ex4;
  int adjustmentCtr = 0;
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  while (coordinateReached == 0) {
    //c = GPS.read();
    while(!GetCoordinates()) {
    }
    Serial.print("Precise Lat: ");
    Serial.println(currlat1prec, 6);
    Serial.print("Precise Long: ");
    Serial.println(currlong1prec, 6);
    Serial.print("currentLat: ");
    Serial.println(currlat1, 4);
    Serial.print("currentLong: ");
    Serial.println(currlong1, 4);
    Serial.print("angle:");
    functionAngle = anglecalc(currlat1prec, latitude, currlong1prec, longitude);
    Serial.println(functionAngle);
//    if((abs(currlat1prec) <= abs(latitude) + 0.00002) && 
//       (abs(currlat1prec) >= abs(latitude) - 0.00002) &&
//       (abs(currlong1prec) <= abs(longitude) + 0.00002) &&
//       (abs(currlong1prec) >= abs(longitude) + 0.00002)){
    if((currlat1 == latitude) && 
       (currlong1 == longitude)) {
      coordinateReached = 1;
      return;
    }
    else {
      forward(2000);
      adjustmentCtr++;
      if (adjustmentCtr >= 3) {
        euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
//        if ((euler.x() > 330) && (functionAngle < 30)) {
//          adjustment1();
//        }
//        else if ((euler.x() < 30) && (functionAngle > 330)) {
//          adjustment2();
//        }
        if(functionAngle == 0) {
          leftTurn(10);
        }
        if (euler.x() > functionAngle) {
          leftTurn(functionAngle);
        }
        else if (euler.x() < functionAngle) {
          rightTurn(functionAngle);
        }
        adjustmentCtr = 0;
      }
    }
  }
  return;
}

void TargetSystem() {
  detecting = 1;
  targetStart = 1;
  if(targetStart) {
      for(pos = 0; pos <= 107; pos += 1) {
       myservo.write(pos);
       delay(15);
      }
    targetStart = 0;
  }
  delay(1000);
  while(detecting) { 
    int val;
    val = digitalRead(vibration_pin);
    if(targetStart) {
      for(pos = 0; pos <= 107; pos += 1) {
       myservo.write(pos);
       delay(15);
      }
    targetStart = 0;
    }
    if(val == 1 ) { // Detects the target hit
      duration = pulseIn(vibration_pin, HIGH);
      Serial.print("duration: ");
      Serial.println(duration);
      if(duration >= 22000) {
        targetHIT = 1; 
      }
    }    
    if(targetHIT) { 
      angle = angle - angleStep;
        for(pos = 180; pos >= 0; pos -= 1) {
        myservo.write(pos);
        delay(15);
    }
         targetHIT = 0;
      
     myservo.write(angle); 
     detecting = 0;  
     delay(100); 
    }
  }
}

void ReturnToHome()
{
  //
}
void avoid()
{
  backward(750);
  brake(500);
  left(750);
  brake(500);
  forward(1000);
  brake(500);
  right(750);
}
void go()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);  
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}
void forward(int time)
{
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);  
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(time);
}

void backward(int time)
{
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH); 
  delay(time);
}

void adjustment1() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);  
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW); 
  delay(200);
}

void adjustment2() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(200); 
}

void right(int time)
{
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(time); 
}

void left(int time)
{
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);  
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW); 
  delay(time);
}
void brake(int time)
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  delay(time);
}
void rightTurn(float turnangle)
{
  float currAngle = 0;
  float firstAngle = 0;
  int axisPass = 0;
  int axisCtr = 0;
  Serial.println("Right turn function");
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  delay(1000);
  while(!euler.x()){
    euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  }
  firstAngle = euler.x();
  Serial.print("First Angle: ");
  Serial.print(firstAngle);
  while((euler.x() < turnangle) && (!axisPass)){
    euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    delay(10);
    Serial.print("eulerx: ");
    Serial.println(euler.x());
    currAngle = euler.x();
    axisCtr++;
    if((currAngle < firstAngle) && (axisCtr > 5)){
      axisPass = 1;
      axisCtr = 0;
    }
    right(75);
    brake(100);
  }
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  if (euler.x() >= turnangle)
  {
    delay(10);
    brake(100);
    Serial.print("End Angle: ");
    Serial.println(euler.x());
  }
//  else if (event.orientation.x - startOrientation > angle)
//  {
//    left(100);
//    brake(100);
//  }
}
void leftTurn(float turnangle) {
  Serial.print("Left turn function");
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  Serial.println("After imu dec");
  float xeuler = euler.x();
  int axisPass = 0;
  int axisCtr = 0;
  while(!euler.x()){
    euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    Serial.print("Euler calibration");
  }
  Serial.println("After while loop");
  float firstAngle;
  xeuler = euler.x();
  firstAngle = euler.x();
  Serial.print("xeuler: ");
  Serial.println(euler.x());
  while((xeuler > turnangle) && (!axisPass)){
    Serial.println("In while loop");
    euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    delay(10);
    Serial.print("eulerx: ");
    Serial.println(xeuler);
    Serial.print("angle: ");
    Serial.println(turnangle);
    xeuler = euler.x();
    axisCtr++;
    if((xeuler > firstAngle) && (axisCtr > 5)){
      axisPass = 1;
      axisCtr = 0;
    }
    left(75);
    brake(100);
  }
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  xeuler = euler.x();
  delay(10);
  if (xeuler <= turnangle)
  {
    Serial.println("Less than");
    brake(100);
  }
}

void leftTurnPWM(float turnangle) {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float xeuler = euler.x();
  int axisPass = 0;
  int axisCtr = 0;
  while(!euler.x()){
    euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  }
  Serial.println("got vector: ");
  float firstAngle;
  xeuler = euler.x();
  firstAngle = euler.x();
  while((xeuler > turnangle) && (!axisPass)){
    Serial.println("in while loop");
    euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    delay(10);
    xeuler = euler.x();
    Serial.print("Euler.x(): ");
    Serial.println(euler.x());
    axisCtr++;
    if((xeuler > firstAngle) && (axisCtr > 5)){
      axisPass = 1;
      axisCtr = 0;
    }
    leftPWM(75);
    delay(10);
    //brake(100);
  }
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  xeuler = euler.x();
  delay(10);
  if (xeuler <= turnangle)
  {
    Serial.println("done");
    brake(100);
  }
}

void leftPWM(int time) {
  analogWrite(ENA, 230);
  analogWrite(ENB, 230);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);  
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW); 
  delay(time);
}

void testMotor()
{
  forward(3000);
  brake(1000);
  left(450);
  brake(1000);
  //TargetSystem();
  delay(1000);
  forward(1400);
  brake(750);
  left(450);
  brake(500);
  //TargetSystem();
  delay(1000);
}

int calibrate() {
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  return mag;
}

void speedControl() {
  // Turn on motors
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  
  // Accelerate from zero to maximum speed
  for (int i = 0; i < 256; i++) {
    analogWrite(ENA, i);
    analogWrite(ENB, i);
    delay(20);
  }
  delay(2000);
  // Decelerate from maximum speed to zero
  for (int i = 255; i >= 0; --i) {
    analogWrite(ENA, i);
    analogWrite(ENB, i);
    delay(20);
  }
  
  // Now turn off motors
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void performTurn(float start, float angle) {
  if (start > angle) {
    leftTurn(angle);
  }
  else if (start < angle) {
    rightTurn(angle);
  }
  return;
}

void distanceSense () {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(5);
  digitalWrite(trig, LOW);

  duration = pulseIn(echo, HIGH);
  distance = (duration * .0343)/2;
  Serial.print("Distance: ");
  Serial.println(distance);
  delay(1000);

  if(distance < 30) {
    detect = true;
    Serial.print("1\n");
  }
  else {
    detect = false;
    Serial.print("0\n");
  }
}


void loop()
{
  while(!distanceSense()){
    forward(10);
    distanceSense();
  }
    //testMotor();
    //TargetSystem();
    while(!calibrate()) {
    }
    digitalWrite(calibrateLED,HIGH);
    Serial.println("Cal");
    delay(6000);
    leftTurnPWM(50);
    float turningangle = 0.0;
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    while(!calibrate()) {
    }
    Serial.println("Calibrated");
    while(!GetCoordinates()) {
    }
    Serial.println("Fix!");
    delay(1000);
    Serial.println("starting");
    while (key == 1) {
      euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      startOrientation = euler.x();
      if(startOrientation != 0.0) {
        key = 0;
      }
    }
    key = 1;
    Serial.print("lat1: ");
    Serial.println(currlat1, 4);
    Serial.print("long1: ");
    Serial.println(currlong1, 4);
    
    turningangle = anglecalc(currlat1prec, lat2, currlong1prec, long2);
    Serial.print("startOrientation (unplug): ");
    performTurn(startOrientation, turningangle);
    
//    if (startOrientation > turningangle) {
//        leftTurn(turningangle);
//    }
//    else if (startOrientation < turningangle) {
//        rightTurn(turningangle);
//    }


    GoToCoordinate(lat2, long2); 
    Serial.println("Coordinate Reached!");
    brake(1000);
    turningangle = anglecalc(currlat1, originLat, currlong1, originLong);
    while (key == 1) {
      euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      startOrientation = euler.x();
      if(startOrientation != 0.0) {
        key = 0;
      }
    }
    key = 1;
    performTurn(startOrientation, turningangle);
    
//    if (startOrientation > turningangle) {
//        leftTurn(turningangle);
//    }
//    else if (startOrientation < turningangle) {
//        rightTurn(turningangle);
//    }


    TargetSystem();
    //GoToCoordinate(originLat, originLong);
//----------------2 Points---------------------------
    turningangle = anglecalc(currlat1, lat3, currlong1, long3);
    while (key == 1) {
      euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      startOrientation = euler.x();
      if(startOrientation != 0.0) {
        key = 0;
      }
    }
    key = 1;
    performTurn(startOrientation, turningangle);
    GoToCoordinate(lat3, long3);
    brake(1000);
    
    turningangle = anglecalc(currlat1, originLat, currlong1, originLong);
    while (key == 1) {
      euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      startOrientation = euler.x();
      if(startOrientation != 0.0) {
        key = 0;
      }
    }
    key = 1;
    performTurn(startOrientation, turningangle);
//    if (startOrientation > turningangle) {
//        leftTurn(turningangle);
//    }
//    else if (startOrientation < turningangle) {
//        rightTurn(turningangle);
//    }
    TargetSystem();
    turningangle = anglecalc(currlat1, lat4, currlong1, long4);
    while (key == 1) {
      euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      startOrientation = euler.x();
      if(startOrientation != 0.0) {
        key = 0;
      }
    }
    key = 1;
    performTurn(startOrientation, turningangle);
    //---------------------finish-from-here---------------------------------------
    GoToCoordinate(lat4, long4);
    brake(1000);
    turningangle = anglecalc(currlat1, originLat, currlong1, originLong);
    while (key == 1) {
      euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      startOrientation = euler.x();
      if(startOrientation != 0.0) {
        key = 0;
      }
    }
    key = 1;
    if (startOrientation > turningangle) {
        leftTurn(turningangle);
    }
    else if (startOrientation < turningangle) {
        rightTurn(turningangle);
    }
    TargetSystem();
    GoToCoordinate(originLat, originLong);
    brake(1000);
}
