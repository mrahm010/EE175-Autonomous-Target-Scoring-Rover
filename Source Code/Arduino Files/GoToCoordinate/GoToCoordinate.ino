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
<<<<<<< HEAD

=======
>>>>>>> 1a57abd75e1089637e672bcee53ba0119e7e37e0
SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);
uint32_t timer = millis();

#define GPSECHO  true
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55);
adafruit_bno055_offsets_t calib;
sensors_event_t event;
//Chung
float lat1 = 33.9754;
<<<<<<< HEAD
float long1 = -117.3257;
=======
float long1 = -117.3254;
>>>>>>> 1a57abd75e1089637e672bcee53ba0119e7e37e0
// HUB
float lat2 = 33.9753;
float long2 = -117.3254;

/////////////////////////////////////////
//Global Variables
float magnetichead;
double bearing;
double distance;
double startOrientation = 0;
int key = 1;
//////////////
//servo
int myservoPIN  = 5;
////////
// Servo code & vibration
int angle = 105;    // initial angle  for servo
int angleStep = 10;
int vibration_pin = 11;
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
//  ///Servo
//  myservo.attach(myservoPIN);
//  pinMode(vibration_pin, INPUT);
//  if(targetStart) { // Initializes target to zero
//    for(pos = 180; pos >= 0; pos -= 1) {
//      myservo.write(pos);
//      delay(15);
//    }
//  }
//  Serial.println("Adafruit GPS library basic test!");
//  GPS.begin(9600);
//  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
//  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
//  GPS.sendCommand(PGCMD_ANTENNA);
//  mySerial.println(PMTK_Q_RELEASE);
//
//  delay(1000);
//  //Serial.begin(9600);
//  Serial.println("Orientation Sensor Test"); Serial.println("");
  
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
int GetCoordinates() {
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
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitudeDegrees, 6); 
      Serial.print(", ");
      Serial.print(GPS.longitudeDegrees, 6);
      Serial.print("\n");
      lat1 = round(GPS.latitudeDegrees * 10000) /10000;
      long1 = round(GPS.latitudeDegrees * 10000) / 10000;
      return 1;
    }
  }
  return 0;
}
void GoToCoordinate(float latitude, float longitude) {
  
  float currentLat = 0;
  float currentLong = 0;
  int coordinateReached = 0;
  float angle;

  while (coordinateReached == 0) {
    currentLat = round(GPS.latitudeDegrees * 10000) / 10000; //stores in 4 decimal places
<<<<<<< HEAD
    currentLong = round(GPS.longitudeDegrees * 10000) /10000; //stores in 4 decimal places
=======
    currentLong = round(GPS.longitudeDegrees * 10000) / 10000; //stores in 4 decimal places
>>>>>>> 1a57abd75e1089637e672bcee53ba0119e7e37e0
    angle = anglecalc(currentLat, latitude, currentLong, longitude);
    if((currentLat == latitude) && (currentLong == longitude)) {
      coordinateReached = 1;
    }
    else {
      forward(1000);
//      if (event.orientation.x > angle) {
//        leftTurn(startOrientation - angle);
//      }
//      else if (event.orientation.x < angle) {
//        rightTurn(angle - startOrientation);
//      }
    }
  }
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

<<<<<<< HEAD
void magHead() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  magnetichead = euler.x();
//  Serial.println(magnetichead);
//  Serial.print("\n");
//  Serial.print("Heading: ");
//  Serial.print(magnetichead);
//  Serial.print("\n");
}

=======
>>>>>>> 1a57abd75e1089637e672bcee53ba0119e7e37e0
void TargetSystem() {
  detecting = 1;
  targetStart = 1;
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
      targetHIT = 1; 
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

<<<<<<< HEAD
double course_to(long lat1, long lon1, long lat2, long lon2) {
  double dlam,dphi,radius;
  
  double deg2rad = 3.141592654/180.0;
  double rad2deg = 180.0/3.141592654;
  
  radius = 6371000.0; //approximate Earth radius in meters.
  
  dphi = deg2rad*(lat1+lat2)*0.5e-6; //average latitude in radians
  //Serial.println(dphi);
  double cphi=cos(dphi);
  
  dphi = deg2rad*(lat2-lat1)*1.0e-6; //differences in degrees
  dlam = deg2rad*(lon2-lon1)*1.0e-6;
  
  dlam *= cphi;  //correct for latitude
  
  bearing=rad2deg*atan2(dlam,dphi);
  if(bearing<0) {bearing=bearing+360.0 ;}
  
  distance = radius * sqrt(dphi*dphi + dlam*dlam);
//  Serial.println("Bearing");
//  Serial.println(bearing, 8);
//  Serial.println("Distance");
//  Serial.println(distance, 8);
  return bearing;
}

=======
>>>>>>> 1a57abd75e1089637e672bcee53ba0119e7e37e0
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
void rightTurn(float angle)
{
  float targetAngle;
  sensors_event_t event;
  bno.getEvent(&event); 
  startOrientation = event.orientation.x;
  float currOrientation = event.orientation.x;
  targetAngle = startOrientation + angle;
  Serial.print("targetAngle: ");
  Serial.println(targetAngle);
  Serial.print("curr: ");
  Serial.println(currOrientation);
  while(currOrientation < angle){
    right(100);
    brake(1000);
    bno.getEvent(&event);
    currOrientation = event.orientation.x;
    Serial.print("curr: ");
    Serial.println(currOrientation);
  }
  if (currOrientation >= angle)
  {
    brake(100);
  }
//  else if (event.orientation.x - startOrientation > angle)
//  {
//    left(100);
//    brake(100);
//  }
}
void leftTurn(float angle) {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  while(euler.x() > angle){
    euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    Serial.println(euler.x());
    left(100);
    brake(1000);
  }
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  if (euler.x() <= angle)
  {
    brake(100);
  }
}

void testMotor()
{
  forward(3000);
  brake(1000);
  rightTurn(450);
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
<<<<<<< HEAD
  char c = GPS.read();
  if ((c) && (GPSECHO))
  if (GPS.newNMEAreceived()) {
  if (!GPS.parse(GPS.lastNMEA()))
    return;
  }
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
  }
  bno.getEvent(&event);
  if (key == 1) {
    startOrientation = event.orientation.x;
    key = 0;
  }
//  if (GPS.fix) {
//    float angle = anglecalc(lat1, lat2, long1, long2);
//    if (event.orientation.x > angle) {
//    leftTurn(startOrientation - angle);
//    }
//    if (event.orientation.x < angle) {
//      rightTurn(angle - startOrientation);
//    }
//    Serial.print("Fix: "); Serial.print((int)GPS.fix);
//    lat1 = round(GPS.latitudeDegrees * 10000) / 10000;
//    long1 = round(GPS.longitudeDegrees * 10000) / 10000;
//    Serial.print("Lat: ");
//    Serial.println(lat1);
//    Serial.print("Long: ");
    Serial.println(long1);
    float angle = anglecalc(lat1, lat2, long1, long2);
    if (event.orientation.x > angle) {
    leftTurn(startOrientation - angle);
    }
    if (event.orientation.x < angle) {
      rightTurn(angle - startOrientation);
    }
    //GoToCoordinate(lat2, long2);
    //angle = anglecalc(lat2, lat1, long2, long1);
    //if (event.orientation.x > angle) {
      //leftTurn(startOrientation - angle);
   // }
    //else if (event.orientation.x < angle) {
     // rightTurn(angle - startOrientation);
   // }
}
  //TargetSystem();
  //ReturnToHome();
=======
    //testMotor();
//    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
//    while(!calibrate()) {
//    }
//    Serial.println("Calibrated");
//    while(!GetCoordinates()) {
//    }
//    Serial.println("Fix!");
//    delay(1000);
//    if (key == 1) {
//      startOrientation = euler.x();
//      key = 0;
//    }
//    angle = anglecalc(lat1, lat2, long1, long2);
//    if (startOrientation > angle) {
//      leftTurn(angle);
//    }
//    else if (startOrientation < angle) {
//      rightTurn(angle);
//    }
    //GoToCoordinate(lat2, long2); comment back in when turning works
>>>>>>> 1a57abd75e1089637e672bcee53ba0119e7e37e0
  
