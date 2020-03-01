/*******************************************************************************************************
* Test Servo Motor Functionality
* Created 7/11/16
* 
*
*
*
*******************************************************************************************************/
#include <Servo.h>
Servo servo;
// Servo controls motor
const int servo_motor = 6; // pin for servo
const int output = 7;
const int trigger = 41; // 10 ms pulse
const int echo = 40; // input for pulse



Servo myServo;


#define ANGLES 7
unsigned char sensorAngle[ANGLES] = { 60, 70, 80, 90, 100, 110, 120 };  
unsigned int distance[ANGLES];

// Scans area by moving servo left to right
// Records angle and moves servo to the next angle
// Repeats every 50ms

// Read dist from ultrasonic sensor
unsigned int findDistance() {
  digitalWrite(trigger ,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger,LOW);
  unsigned long period = pulseIn(echo,HIGH); 
  return period * 343 / 2000; // in inches
}

void angleServo() {
  static unsigned char angleIndex = 0;
  for(int i = 0; i < ANGLES; i++){
    angleIndex = sensorAngle[i];
    myServo.write(angleIndex);
    delay(500);
    }
}

void calcNextDistance() 
{
  static unsigned char savedAngle = 0;
  static signed char step = 1;
  distance[savedAngle] = findDistance (); 
  savedAngle += step;
  if(savedAngle == ANGLES - 1) step = -1;
  else if (savedAngle == 0) step = 1;
  servo.write( sensorAngle[savedAngle] );
}

void obstacle(){
  calcNextDistance();
  unsigned char tooClose = 0;
  for (unsigned char i = 0 ; i < ANGLES ; i++) 
     if ( distance [i] < 10)
        tooClose = 1;

  if ( tooClose ) {
  // Something's nearby: back up left
  Serial.print("Object is too close");
  //output pin will be connected to Arduino Mega for ISR
  digitalWrite(output, HIGH); //Sets output pin to HIGH
  delay(50);
  digitalWrite(output, LOW); //Sets output pin to LOW
  }
  else {
  // Nothing in our way: go forward
  Serial.print("Go!");
} 
//To be put in Arduino Mega

  
}
void setup() {
  Serial.begin(9600);
  pinMode(trigger, OUTPUT); 
  pinMode(echo, INPUT);
  pinMode(output, OUTPUT);
  digitalWrite(trigger , LOW);
  servo.attach(servo_motor); 
  servo.write(90);
// Initial scan
  servo.write(sensorAngle[0] );
  delay (200);
  for (unsigned char i = 0 ; i <ANGLES;i++)
    calcNextDistance();
}

void loop(){
  
  obstacle();
  
}
