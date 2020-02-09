#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Arduino.h"
#include <limits.h>
#include <math.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

adafruit_bno055_offsets_t calib;
sensors_event_t event;

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
  Serial.begin(115200);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  pinMode(ENA, OUTPUT); //Set all the L298n Pin to output
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

   if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);
  bno.setExtCrystalUse(true);
  
}

void compass()
{
  
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  
}

void rightTurn(float angle)
{
 
  if(event.orientation.x < angle){
    right(250);
  }
  else if (event.orientation.x = angle)
  {
    brake();
  }
  else if (event.orientation.x > angle)
  {
    left(250);
  }
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

void left(int speed)
{
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH); 
}

void right(int speed)
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
  delay(100);
  bno.getEvent(&event);
  //testMotor();
  rightTurn(90);
}
