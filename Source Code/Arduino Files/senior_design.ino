/* 
L298N H-Bridge driving DC motor on Arduino
*/

int ENA = 10; // MCU PWM Pin 10 to ENA on L298n Board
int IN1 = 9;  // MCU Digital Pin 9 to IN1 on L298n Board 
int IN2 = 8;  // MCU Digital Pin 8 to IN2 on L298n Board

int ENB = 5;  // MCU PWM Pin 5 to ENB on L298n Board
int IN3 = 7;  // MCU Digital pin 7 to IN3 on L298n Board
int IN4 = 6;  // MCU Digital pin 6 to IN4 on L298n Board

void setup()
{

  pinMode(ENA, OUTPUT); //Set all the L298n Pin to output
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}
void DRIVEONE()
{
  // Run the motors on both direction at fixed speed
  digitalWrite(IN1, HIGH); // Turn HIGH motor A
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 200); // TO set the turning speed to 200 out of possible range 0 to 255
 
  digitalWrite(IN3, HIGH); // turn HIGH motor B
  digitalWrite(IN4, LOW);  // To set the turning speed to 200 out of possible range 0 to 255 
  analogWrite(ENB, 200);
  delay(2000);  // Delay to 2 seconds
  
  // Changing the direction of the motor
  
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, HIGH);  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH); 
  delay(2000); // Delay to 2 seconds
 
  digitalWrite(IN1, LOW); // Turn the motor off
  digitalWrite(IN2, LOW); // Turn the motor off  
  digitalWrite(IN3, LOW); // Turn the motor off
  digitalWrite(IN4, LOW); // Turn the motor off
}
void DRIVETWO()
{
/*
  These function will turn the motors on the possible speeds, the maximum speed turns is determined
  by the motor specs and the operating voltage. The PWM(Pulse with modulation values will sent
  by analogWrite() function to drive the maxim speed. 
*/
  
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH); 

  for (int x = 0; x < 256; x++)   // Motor will accelerate from 0 to maximum speed
  {
    analogWrite(ENA, x);
    analogWrite(ENB, x);
    delay(20);
  } 
 
  for (int y = 255; y >= 0; --y)  // Motor will decelerate from maximum speed to 0
  {
    analogWrite(ENA, y);
    analogWrite(ENB, y);
    delay(20);
  } 
 
  digitalWrite(IN1, LOW); //  Tun Off All the Motors 
  digitalWrite(IN2, LOW);  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);  
}

void forward()
{
  analogWrite(ENA, 200);
  analogWrite(ENB, 200);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);  
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW); 
}

void backward()
{
  analogWrite(ENA, 200);
  analogWrite(ENB, 200);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH); 
}

void left()
{
  analogWrite(ENA, 170);
  analogWrite(ENB, 170);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH); 
}

void right()
{
  analogWrite(ENA, 170);
  analogWrite(ENB, 170);
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

testMotor()
{
  forward();
  delay(3000);
  //brake();
  //delay(1000);
  left();
  delay(750);
  forward();
  delay(1000);
  left();
  delay(750);
}
void loop()
{
  
  testMotor();
}
