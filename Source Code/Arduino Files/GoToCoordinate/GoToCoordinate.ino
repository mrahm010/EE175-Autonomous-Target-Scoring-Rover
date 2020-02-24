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
Adafruit_GPS GPS(&Serial3);
uint32_t timer = millis();

#define GPSECHO  true
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55);
adafruit_bno055_offsets_t calib;
sensors_event_t event;
//Chung
float lat1prec = 0;
float long1prec = 0;
float lat1 = 0;
float long1 = 0;
// HUB
float lat2 = 33.9757;
float long2 = -117.3294;

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
int vibration_pin = 39;
const int minAngle = 0;
const int maxAngle = 106;
int targetStart = 1;
int targetHIT = 0;
int pos = 0;
int servoInit = 130;
int detecting = 1;
Servo myservo;
/* 
L298N H-Bridge driving DC motor on Arduino
*/
int ENA = 10; // MCU PWM Pin 10 to ENA on L298n Board
int IN1 = 13;  // MCU Digital Pin 9 to IN1 on L298n Board 
int IN2 = 12;  // MCU Digital Pin 8 to IN2 on L298n Board

int ENB = 6;  // MCU PWM Pin 5 to ENB on L298n Board
int IN3 = 11;  // MCU Digital pin 7 to IN3 on L298n Board
int IN4 = 10;  // MCU Digital pin 6 to IN4 on L298n Board

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
      Serial.print("Location: ");
      Serial.print(GPS.latitudeDegrees, 6); 
      Serial.print(", ");
      Serial.print(GPS.longitudeDegrees, 6);
      Serial.print("\n");
//      lat1 = (round(GPS.latitudeDegrees * 10000)) /10000;
//      long1 = (round(GPS.longitudeDegrees * 10000)) / 10000;
      counter++;
//      if (Serial.available() > 0) {
//        confirm = Serial.read();
//      }
      if (counter > 5) {
        lat1prec = GPS.latitudeDegrees;
        long1prec = GPS.longitudeDegrees;
        lat1 = floor(GPS.latitudeDegrees*10000 + 0.5)/10000;
        long1 = floor(GPS.longitudeDegrees*10000 + 0.5)/10000;
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
    currentLatprec = GPS.latitudeDegrees;
    currentLongprec = GPS.longitudeDegrees;
    currentLat = floor(GPS.latitudeDegrees*10000 + 0.5)/10000;
    currentLong = floor(GPS.longitudeDegrees*10000 + 0.5)/10000;
    Serial.print("Precise Lat: ");
    Serial.println(currentLatprec, 6);
    Serial.print("Precise Long: ");
    Serial.println(currentLongprec, 6);
    Serial.print("currentLat: ");
    Serial.println(currentLat, 4);
    Serial.print("currentLong: ");
    Serial.println(currentLong, 4);
    Serial.print("angle:");
    functionAngle = anglecalc(currentLatprec, latitude, currentLongprec, longitude);
    Serial.println(functionAngle);
    if((currentLat == latitude) && (currentLong == longitude)) {
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
      if(duration >= 20000) {
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
  analogWrite(ENA, 200);
  analogWrite(ENB, 200);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);  
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(time);
}

void backward(int time)
{
  analogWrite(ENA, 200);
  analogWrite(ENB, 200);
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
  //analogWrite(ENA, speed);
  //analogWrite(ENB, speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(time); 
}

void left(int time)
{
  //analogWrite(ENA, speed);
  //analogWrite(ENB, speed);
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
  Serial.println("Right turn function");
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  while(euler.x() < turnangle){
    euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    Serial.print("eulerx: ");
    Serial.println(euler.x());
    Serial.print("angle: ");
    Serial.println(turnangle);
    right(100);
    brake(100);
  }
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  if (euler.x() >= turnangle)
  {
    brake(100);
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
  float xeuler = euler.x();
  while(xeuler > turnangle){
    euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    Serial.print("eulerx: ");
    Serial.println(xeuler);
    Serial.print("angle: ");
    Serial.println(turnangle);
    xeuler = euler.x();
    left(100);
    brake(100);
  }
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  xeuler = euler.x();
  if (xeuler <= turnangle)
  {
    brake(100);
  }
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

void loop()
{
    float turningangle = 0.0;
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    while(!calibrate()) {
    }
    Serial.println("Calibrated");
    while(!GetCoordinates()) {
    }
    Serial.println("Fix!");
    Serial.println("Quality at least 2");
    delay(1000);
    Serial.print("starting...");
    delay(3000);
    while (key == 1) {
      euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      startOrientation = euler.x();
      if(startOrientation != 0.0) {
        key = 0;
      }
    }
    key = 1;
    Serial.print("lat1: ");
    Serial.println(lat1, 4);
    Serial.print("long1: ");
    Serial.println(long1, 4);
    
    turningangle = anglecalc(lat1prec, lat2, long1prec, long2);
    Serial.print("startOrientation: ");
    Serial.println(startOrientation);
    delay(1000);
    if (startOrientation > turningangle) {
        leftTurn(turningangle);
    }
    else if (startOrientation < turningangle) {
        rightTurn(turningangle);
    }
    GoToCoordinate(lat2, long2); 
    brake(1000);
    turningangle = anglecalc(lat2, lat1, long2, long1);
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
  
}
