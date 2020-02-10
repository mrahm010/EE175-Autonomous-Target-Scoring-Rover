#include <Servo.h>

Servo myservo;  // create servo object to control a servo
int myservoPIN = 3;

int angle = 105;    // initial angle  for servo
int angleStep = 10;
int vibration_pin = 7;

const int minAngle = 0;
const int maxAngle = 106;

int targetStart = 1;
int targetHIT = 0;


int pos = 0;

//int SenseForHit(int &counter) {
//  int val;
//  val = digitalRead(vibration_pin);
//  if(val == 1) {
//    counter++;
//  }
//  else {
//    if(counter > 0) {
//      counter--;
//    }
//  }
//  if (counter >= 1000) {
//    return 1;
//  }
//  else {
//    return 0;
//  }
//}

void setup() {
  Serial.begin(9600);      
  myservo.attach(myservoPIN);  // attaches the servo on pin 3 to the servo object
  Serial.println("Servo Button ");
  pinMode(vibration_pin, INPUT);
  
}

void TargetSystem() {
 int val;
 val = digitalRead(vibration_pin);
 if (targetStart) {
  for(pos = 0; pos <= 107; pos += 1) {
    myservo.write(pos);
    delay(15);
  }
  targetStart = 0;
 }
    if(val == 1 ){
        targetHIT = 1; 
    }    
    if(targetHIT){ //changes the angle for the next time through the loop
       angle = angle + angleStep;
        if (angle <= minAngle || angle >= maxAngle) { // reveres the direction of the moving at the ends of the angle
          angleStep = -angleStep;
          targetHIT = 0;
        } 
      myservo.write(angle); // move the servo to desired angle
      Serial.print("Moved to: ");
      Serial.print(angle);   // print the angle
      Serial.println(" degree");    
      delay(100); // waits for the servo to get there
   }  
}

void loop() {
  TargetSystem();
}
