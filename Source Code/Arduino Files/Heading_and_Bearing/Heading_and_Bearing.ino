#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_GPS.h>
#include <utility/imumaths.h>
#include "Arduino.h"
#include <limits.h>
#include <math.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
adafruit_bno055_offsets_t calib;
sensors_event_t event;

//GPS Init
SoftwareSerial myserial(1, 0);
Adafruit_GPS GPS

//Chung
float lat1 = 0;
float long1 = 0;
// HUB
float lat2 = 0;
float long2 = 0;

/////////////////////////////////////////
//Global Variables
float magnetichead;
double bearing;
double distance;
double startOrientation = 0;
int key = 1;

/* 
L298N H-Bridge driving DC motor on Arduino
*/
int ENA = 10; // MCU PWM Pin 10 to ENA on L298n Board
int IN1 = 13;  // MCU Digital Pin 9 to IN1 on L298n Board 
int IN2 = 12;  // MCU Digital Pin 8 to IN2 on L298n Board

int ENB = 6;  // MCU PWM Pin 5 to ENB on L298n Board
int IN3 = 7;  // MCU Digital pin 7 to IN3 on L298n Board
int IN4 = 8;  // MCU Digital pin 6 to IN4 on L298n Board

void setup()
{
  Serial.begin(9600);
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
  bno.setExtCrystalUse(true);

  delay(1000);
  
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

void magHead() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  magnetichead = euler.x();
  Serial.println(magnetichead);
  Serial.print("\n");
  Serial.print("Heading: ");
  Serial.print(magnetichead);
  Serial.print("\n");
}

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
  Serial.println("Bearing");
  Serial.println(bearing, 8);
  Serial.println("Distance");
  Serial.println(distance, 8);
  return bearing;
}




void forward()
{
  analogWrite(ENA, 230);
  analogWrite(ENB, 230);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);  
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW); 
}

void backward()
{
  analogWrite(ENA, 230);
  analogWrite(ENB, 230);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH); 
}

void right(int speed)
{
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH); 
}

void left(int speed)
{
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);  
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW); 
}
void brake()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
void rightTurn(float angle)
{
 
  if(event.orientation.x < angle){
    right(250);
    delay(100);
    brake();
  }
  else if (event.orientation.x = angle)
  {
    brake();
  }
  else if (event.orientation.x > angle)
  {
    left(250);
    delay(100);
    brake();
  }
}
void leftTurn(float angle) {
  if(event.orientation.x > angle){
    left(250);
    delay(100);
    brake();
  }
  else if (event.orientation.x = angle)
  {
    brake();
  }
  else if (event.orientation.x < angle)
  {
    right(250);
    delay(100);
    brake();
  }
}


void testMotor()
{
  forward();
  delay(3000);
  brake();
  delay(1000);
  left(230);
  delay(750);
  brake();
  delay(1000);
  forward();
  delay(3000);
  left(230);
  delay(750);
}
void loop()
{
  
  bno.getEvent(&event);
  //magHead();
  if (key == 1) {
    startOrientation = event.orientation.x;
    key = 0;
  }
  float angle = anglecalc(lat1, lat2, long1, long2);
  //Serial.print(angle);
  //Serial.println(" ");
  Serial.println(event.orientation.x);
  //magHead();
  //Serial.println(magnetichead);
  //Serial.print("\n");
  //double b = course_to(lat1, lat2, long1, long2);
  //testMotor();
  //rightTurn(angle);

  if (event.orientation.x > angle) {
    leftTurn(startOrientation - angle);
  }
  if (event.orientation.x < angle) {
    rightTurn(angle - startOrientation);
  }
  GoToWaypoint();
  RaiseTarget();
  ReturnHome;
  delay(200);
}
