#include <IRremote.h>

// pins for the motors

const int leftFw = 3;
const int leftBk = 4;

const int rightFw = 7;
const int rightBk = 8;

const int enablePinRight = 5;
const int enablePinLeft = 6;

// IR remote pin
const int receiverPin = A2;

// pins for the ultrasonics
const int trigPinLeft = 11;
const int echoPinLeft = A1;

const int trigPinRight = 10;
const int echoPinRight = A0;

const int trigPinCenter = 9;
const int echoPinCenter = A3;

// speeds when making a turn
const int speedTurnLeft = 200;
const int speedTurnRight = 200;


// thresholds for the sensors
const int distanceRightThreshold = 25;
const int distanceCenterThreshold = 13;
const int distanceLeftThreshold = 25;

// speeds while going straight forward(for a few seconds, after making a turn)
const int speedLeftForward = 120;
const int speedRightForward = 115;

const int leftSpeed = 130; // default speed of the left DC motor
const int rightSpeed = 125; // default speed of the right DC motor

unsigned long lastTimeTurn = 0; // last time a turn happened
const int noTurn = 2700; // minimum time needed between turns
const int timeRight = 850; // time for turning right
const int timeLeft = 800 ; // time for turning left
const int timeUturn = 1630; // time for 180 turn

// pins for the leds
const int pinBlue = 12;
const int pinGreen = A4;

// variable describing motors state
bool carStop = true; 

// disances for left, right and center sensor
float distanceLeft;
float distanceRight;
float distanceCenter;

// variables for the PID control
float error , lastError, derivative;
const int offset = 8; // distance for the rightWall
int integral;

// constans for the PID control
int turn , Kp = 6, Ki = 0, Kd = 4;
float speedCorrection;

// the spped of each wheel
int leftWheelSpeed , rightWheelSpeed;

// IR remote result
IRrecv irrecv(receiverPin);
decode_results results;

unsigned long keyValue = 0;

void setup()
{
  // motors
  pinMode(leftFw, OUTPUT);
  pinMode(leftBk , OUTPUT);
  pinMode(enablePinLeft , OUTPUT);

  pinMode(rightFw, OUTPUT);
  pinMode(rightFw, OUTPUT);
  pinMode(enablePinRight , OUTPUT);

  // IR
  irrecv.enableIRIn();
  irrecv.blink13(true);

  // ultrasonic sensors
  pinMode(trigPinCenter , OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPinCenter , INPUT);

  pinMode(trigPinLeft , OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPinLeft , INPUT);

  pinMode(trigPinRight , OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPinRight , INPUT);
  
  Serial.begin(9600);
}

void readIR()
{
  /* function that decodes the IR remote signal */
  
  if (irrecv.decode(&results)){
    if (results.value == 0XFFFFFFFF)
      results.value = keyValue;
   
    switch(results.value)
    {   
    case 0xFFC23D:
      stopCar();
      Serial.println(">|"); //stop
      break;
   
    case 0xFF02FD:
      carStop = false;
      Serial.println(">>|"); // start
    break ;
    }
   
    keyValue = results.value;
    irrecv.resume();
    }
}

void stopCar()
{
   /* function that stops the motors of the car(when pressing the remote stop button) */
   
   carStop = true;

   analogWrite(enablePinRight , 0 );
   digitalWrite(rightFw , LOW);
   digitalWrite(rightBk , LOW);

   analogWrite(enablePinLeft , 0);
   digitalWrite(leftFw , LOW);
   digitalWrite(leftBk , LOW);
}

void readSensor(const int trigPin , const int echoPin , float &distance)
{
  /* function that calculates the distance measured by a sensor */
  
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  float duration = pulseIn(echoPin, HIGH);
  
  // Sound wave reflects from the obstacle, so to calculate the distance we
  // consider half of the distance traveled.
  distance = duration * 0.034 / 2;
}


void forward()
{ 
   /* function that determinates the speed of each wheel, by using the PID control */
   
   // calculates the variables for the PID control  
   error = distanceRight  - offset;
   integral = integral * 2 / 3 + error;
   derivative = error - lastError;
   lastError = error;

   // calculates the speed correction 
   speedCorrection = Kp * error + Ki * integral + Kd * derivative;

   leftWheelSpeed = leftSpeed + speedCorrection;
   rightWheelSpeed = rightSpeed - speedCorrection;

   /*
   Serial.print("distanta = ");
   Serial.print(distanceRight);

   Serial.print(" eroare = ");
   Serial.print(error);
   
   Serial.print(" turn = ");
   Serial.print(turn);
  */

   // sets the new speed of each motor 
   analogWrite(enablePinLeft , leftWheelSpeed);
   digitalWrite(leftFw , HIGH);
   digitalWrite(leftBk , LOW); 
  
   analogWrite(enablePinRight , rightWheelSpeed);
   digitalWrite(rightFw , HIGH);
   digitalWrite(rightBk , LOW);

  /*
   Serial.print(" leftWheelSpeed = ");
   Serial.print(leftWheelSpeed);
   
   Serial.print(" rightWheelSpeed = ");
   Serial.println(rightWheelSpeed);

   Serial.println("FORWARD\n");
  */
}

void turnRight()
{
  /* function that makes the car do a 90° right turn */

  // the new speed of each motor 
  leftWheelSpeed = speedTurnLeft;
  rightWheelSpeed = speedTurnRight;

  // powers the motors, both on the opposite side
  analogWrite(enablePinLeft , leftWheelSpeed);
  digitalWrite(leftFw , HIGH);
  digitalWrite(leftBk , LOW); 
  
  analogWrite(enablePinRight , rightWheelSpeed );
  digitalWrite(rightFw , LOW);
  digitalWrite(rightBk , HIGH);

  unsigned long time = millis();

  // for <<timeRight> ms, the blue led will blink and the car is turning right
  while(millis() - time < timeRight)
  {
    digitalWrite(pinBlue, HIGH);
    readIR();
  }

  // stop the led
  digitalWrite(pinBlue , LOW);
  
  // stop the motors
  analogWrite(enablePinLeft , 0);
  digitalWrite(leftFw , LOW);
  digitalWrite(leftBk , HIGH); 
  
  analogWrite(enablePinRight , 0 );
  digitalWrite(rightFw , HIGH);
  digitalWrite(rightBk , LOW);

  
  lastTimeTurn = millis();
}

void turn180()
{
  /* function that makes the car do a 180° turn */

  // the new speed of each motor 
  leftWheelSpeed = speedTurnLeft;
  rightWheelSpeed = speedTurnRight;

  // powers the dc motors, both on the opposite side
  analogWrite(enablePinLeft , leftWheelSpeed);
  digitalWrite(leftFw , LOW);
  digitalWrite(leftBk , HIGH); 
  
  analogWrite(enablePinRight , rightWheelSpeed );
  digitalWrite(rightFw , HIGH);
  digitalWrite(rightBk , LOW);

  // for <<timeUturn>> ms, the car will make a U-turn
  unsigned long time = millis();
  while(millis() - time < timeUturn)
  {
    readIR();
  }
  
  // stop the motors
  analogWrite(enablePinLeft , 0);
  digitalWrite(leftFw , LOW);
  digitalWrite(leftBk , HIGH); 
  
  analogWrite(enablePinRight , 0 );
  digitalWrite(rightFw , HIGH);
  digitalWrite(rightBk , LOW);
  
  lastTimeTurn = millis();
}

void turnLeft()
{
  /* function that makes the car do a 90° left turn */

  // new speed of each motor
  leftWheelSpeed = speedTurnLeft;
  rightWheelSpeed = speedTurnRight;

  //powers the dc motors, both on the opposite side
  analogWrite(enablePinLeft , leftWheelSpeed);
  digitalWrite(leftFw , LOW);
  digitalWrite(leftBk , HIGH); 
  
  analogWrite(enablePinRight , rightWheelSpeed );
  digitalWrite(rightFw , HIGH);
  digitalWrite(rightBk , LOW);

  unsigned long time = millis();

  // for <<timeLeft>> ms, the green led will blink and the car will turn left
  while(millis() - time < timeLeft)
  {
    digitalWrite(pinGreen, HIGH);
    readIR();
  }

  // stop the green led
  digitalWrite(pinGreen , LOW);

  // stop the motors;
  analogWrite(enablePinLeft , 0);
  digitalWrite(leftFw , LOW);
  digitalWrite(leftBk , HIGH); 
  
  analogWrite(enablePinRight , 0 );
  digitalWrite(rightFw , HIGH);
  digitalWrite(rightBk , LOW);
  
  lastTimeTurn = millis();
}

void maze()
{
  
  if(distanceRight > distanceRightThreshold && millis() - lastTimeTurn > noTurn) // the car goes to the right if it can
        turnRight();
  else if(millis() - lastTimeTurn < noTurn && distanceCenter > distanceCenterThreshold) // if the car just made a turn, then it has to wait until the next turn, so it just goes forward ( if it can)
  {
    analogWrite(enablePinLeft , speedLeftForward);
    digitalWrite(leftFw , HIGH);
    digitalWrite(leftBk , LOW); 
    
    analogWrite(enablePinRight , speedRightForward);
    digitalWrite(rightFw , HIGH);
    digitalWrite(rightBk , LOW);
  }
  else if(distanceCenter > distanceCenterThreshold) // if there is no space on the right, it goes forward, using the PID
        forward();
  else if(distanceLeft > distanceLeftThreshold && millis() - lastTimeTurn > noTurn) // goes to the left if it is the only option
        turnLeft();
  else if(distanceRight < distanceRightThreshold && distanceLeft < distanceLeftThreshold && distanceCenter < distanceCenterThreshold) // if it is blocked, then it makes a U-turn
        turn180();
}

void loop()
{
  // the car works as long as it is not stopped by the remote, when it finishes

  readIR();
  
  if(carStop == false)
  {
    readSensor(trigPinCenter , echoPinCenter , distanceCenter);
    readSensor(trigPinLeft , echoPinLeft , distanceLeft);
    readSensor(trigPinRight , echoPinRight , distanceRight); 
    maze();

    /*
    Serial.print("Left: ");
    Serial.print(distanceLeft);
  
    Serial.print("     Center: ");
    Serial.print(distanceCenter);
  
    Serial.print("      Right: ");
    Serial.println(distanceRight);
    */
  }
  else 
  {
    // the motors are stopped
    analogWrite(enablePinLeft , 0);
    digitalWrite(leftFw , HIGH);
    digitalWrite(leftBk , LOW); 
    
    analogWrite(enablePinRight , 0);
    digitalWrite(rightFw , HIGH);
    digitalWrite(rightBk , LOW);
  }
}
