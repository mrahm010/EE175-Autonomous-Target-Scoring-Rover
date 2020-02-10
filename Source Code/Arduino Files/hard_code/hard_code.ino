/*******************************************************************************************************
* Test Servo Motor Functionality
* Created 7/11/16
* 
*
* //Change so servo doesnt move
*
*******************************************************************************************************/
#include <Servo.h>
Servo servo;
#define pinServo 4
#define trig 3
#define echo 2
long duration, distance_from_rover;

int ENA = 10; // MCU PWM Pin 10 to ENA on L298n Board
int IN1 = 13;  // MCU Digital Pin 9 to IN1 on L298n Board 
int IN2 = 12;  // MCU Digital Pin 8 to IN2 on L298n Board

int ENB = 6;  // MCU PWM Pin 5 to ENB on L298n Board
int IN3 = 7;  // MCU Digital pin 7 to IN3 on L298n Board
int IN4 = 8;  // MCU Digital pin 6 to IN4 on L298n Board


Servo myServo;

void setup() {
  Serial.begin(9600);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  myServo.attach(pinServo);

  pinMode(ENA, OUTPUT); //Set all the L298n Pin to output
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

#define NUM_ANGLES 7
unsigned char sensorAngle[NUM_ANGLES] = { 60, 70, 80, 90, 100, 110, 120 };  
unsigned int distance[NUM_ANGLES];

unsigned int readDistance(){
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  duration = pulseIn(echo, HIGH);
  //distance = duration / 58;
  distance_from_rover = duration / 148; // in inches


  delay(100);
  Serial.print("Distance: ");
  Serial.println(distance_from_rover);
  return distance_from_rover;
}

void angleServo() {
  static unsigned char angleIndex = 0;
  for(int i = 0; i < NUM_ANGLES; i++){
    angleIndex = sensorAngle[i];
    myServo.write(angleIndex);
    delay(500);
    }
}

void readNextDistance(){
  static unsigned char angleIndex = 0;
  static signed char next = 1;
  distance[angleIndex] = readDistance();
  angleIndex += next;
  if(angleIndex == NUM_ANGLES - 1) next = -1;
  else if (angleIndex == 0) next = 1;
  myServo.write( sensorAngle[angleIndex] );

}

void obstacle(){
  readNextDistance();
  unsigned char tooClose = 0;
  for (unsigned char i = 0 ; i < NUM_ANGLES ; i++) 
     if ( distance [i] < 30)
        tooClose = 1;

  if ( tooClose ) {
  // Something's nearby: back up left
  Serial.print("Object is too close");
  avoid();
  delay(250);
  tooClose = 0;
  }
  else {
  // Nothing in our way: go forward
  Serial.print("Go!");
  go();
  }  
}

void go()
{
  analogWrite(ENA, 200);
  analogWrite(ENB, 200);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);  
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}
void avoid()
{
  backward(750);
  left(750);
  forward(1000);
  right(750);
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

void left(int time)
{
  //analogWrite(ENA, speed);
  //analogWrite(ENB, speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(time); 
}

void right(int time)
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
void testMotor()
{
  forward(3000);
  brake(1000);
  left(550);
  brake(1000);
  forward(3000);
  brake(750);
  left(550);
}

void loop(){
  testMotor();
  //obstacle();
  //avoid();
  
}
