/*******************************************************************************************************
* Test Servo Motor Functionality
* Created 7/11/16
* 
*
* //Change so servo doesnt move
*
*******************************************************************************************************/
#include <Servo.h>
int myservoPIN  = 5;
long duration, distance_from_rover;
////////////////////////////////Motor Driver Pins
int ENA = 10; // MCU PWM Pin 10 to ENA on L298n Board
int IN1 = 13;  // MCU Digital Pin 9 to IN1 on L298n Board 
int IN2 = 12;  // MCU Digital Pin 8 to IN2 on L298n Board
int ENB = 6;  // MCU PWM Pin 5 to ENB on L298n Board
int IN3 = 7;  // MCU Digital pin 7 to IN3 on L298n Board
int IN4 = 8;  // MCU Digital pin 6 to IN4 on L298n Board
////////////////////////////////

////////////////////////////////Servo code & vibration
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
////////////////////////////////

////////////////////////////////GPS Fetch
#include <SoftwareSerial.h>
int counter = 1; //tells which coordinate we are readingg
int longitude = 0; //tells whether it is reeading lat or long
String inStringLatitude = "";    // string to hold input
String inStringLongitude = "";
char inputLatitude[11];
char inputLongitude[11];
  
struct GPSCoor {
  double latitude = 0.0;
  double longitude = 0.0;
};
struct GPSPack {
  GPSCoor CoordinateOrigin;
  GPSCoor Coordinate1;
  GPSCoor Coordinate2;
  GPSCoor Coordinate3;
};
struct GPSPack Route;
int coordinatesRecieved = 0;
////////////////////////////////

Servo myservo;

////////////////////////////////
void fetchCoordinates(GPSPack &Route) {
  //GPSCoor Coor1;
  //GPSCoor Coor2;
  //GPSCoor Coor3;
  while (Serial.available() > 0) {
    int inChar = Serial.read();
    if (inChar == ',') {    //tells us to switch from reading latitude to reading longitude
      longitude = 1;
    }
    else if (inChar == '.') {
      if (longitude == 0) {
        inStringLatitude += '.';
      }
      else {
        inStringLongitude += '.';
      }
    }
    else if (inChar == '-') {  //negative?
      if (longitude == 0) {
        inStringLatitude += '-';
      }
      else {
        inStringLongitude += '-';
      }
    }
    else if (isDigit(inChar)) {
      // convert the incoming byte to a char and add it to the string:
      if (longitude == 0) {
        inStringLatitude += (char)inChar;
      }
      else {
        inStringLongitude += (char)inChar;
      }
    }
    // if you get a newline, print the string, then the string's value:
    else if (inChar == '|') {
      longitude = 0;
      inStringLatitude.toCharArray(inputLatitude, sizeof(inputLatitude));
      inStringLongitude.toCharArray(inputLongitude, sizeof(inputLongitude));
      if(counter == 1) {
        Route.Coordinate1.latitude = atof(inputLatitude);
        Serial.print("Latitude1:");
        Serial.println(Route.Coordinate1.latitude);
        Route.Coordinate1.longitude = atof(inputLongitude);
        Serial.print("Longitude1:");
        Serial.println(Route.Coordinate1.longitude);
      }
      else if (counter == 2) {
        Route.Coordinate2.latitude = atof(inputLatitude);
        Serial.print("Latitude2:");
        Serial.println(Route.Coordinate2.latitude);
        Route.Coordinate2.longitude = atof(inputLongitude);
        Serial.print("Longitude2:");
        Serial.println(Route.Coordinate2.longitude);
      }
      else if (counter == 3) {
        Route.Coordinate3.latitude = atof(inputLatitude);
        Serial.print("Latitude3:");
        Serial.println(Route.Coordinate3.latitude);
        Route.Coordinate3.longitude = atof(inputLongitude);
        Serial.print("Longitude3:");
        Serial.println(Route.Coordinate3.longitude);
      }
      inStringLatitude = "";
      inStringLongitude = "";
      counter++;
    }
  }
  if (counter >= 3) {  //are the coordinates recieved and ready to use?
    coordinatesRecieved = 1;
  }
}
////////////////////////////////

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
  left(450);
  brake(1000);
  TargetSystem();
  delay(1000);
  forward(1400);
  brake(750);
  left(450);
  brake(500);
  TargetSystem();
  delay(1000);
}

void setup() {
  Serial.begin(9600);
  myservo.attach(myservoPIN);
  pinMode(vibration_pin, INPUT);
  if(targetStart) { // Initializes target to zero
    for(pos = 180; pos >= 0; pos -= 1) {
      myservo.write(pos);
      delay(15);
    }
  }

  pinMode(ENA, OUTPUT); //Set all the L298n Pin to output
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop(){
////////////////////////////////GPS Fetch Init
  while (coordinatesRecieved == 0) {
    fetchCoordinates(Route);
  }

  Serial.print(Route.Coordinate1.latitude);
////////////////////////////////

  //testMotor();
  //obstacle();
  //avoid();
  //TargetSystem();
  
}
