#include <Servo.h>

Servo myservo; 
int myservoPIN = 3; // Servo on Pin 3
int vibration_pin = 7; // Vibration Sensor on Pin 7

int angle = 105;
int angleStep = 10;
const int minAngle = 0;
const int maxAngle = 106;

int targetStart = 1; // Lock and Key for Target to Raise when the rover stops
int targetHIT = 0;
int pos = 0;

int detecting = 1;

void setup() {
  Serial.begin(9600);      
  myservo.attach(myservoPIN);  
  // Serial.println("Servo Button");
  pinMode(vibration_pin, INPUT);
  if(targetStart) { // Initializes target to zero
    for(pos = 180; pos >= 0; pos -= 1) {
      myservo.write(pos);
      delay(15);
    }
  }
}

void TargetSystem() {
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
      if(angle <= minAngle || angle >= maxAngle) { 
         targetHIT = 0;
      } 
     myservo.write(angle); 
     Serial.print("Moved to: ");
     Serial.print(angle);   
     Serial.println(" degree");    
     delay(100); 
    }
  } 
  detecting = 0; 
}

void loop() {
  TargetSystem();
}
