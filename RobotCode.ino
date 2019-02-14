
#include <Servo.h>

//Digital port variables:
int LEDs = 2;
int buzzer = 3;
int servo = 4;
int echo = 5;
int ping = 6;
int switchPin = 7;

int AIN1 = 11;
int AIN2 = 12;
int PWMA = 13;

int PWMB = 8;
int BIN2 = 9;
int BIN1 = 10;


//Servo code
Servo theServo;

//Global variables

int darkestCorner;
int darkestServoPosition;
float distanceToNextWall;

void setup() {
  // put your setup code here, to run once:

  pinMode(LEDs, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(servo, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(ping, OUTPUT);
  pinMode(switchPin, INPUT_PULLUP);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  theServo.attach(servo);
  theServo.write(20);

  Serial.begin(9600); //for testing / monitoring
}

void loop() {
  // put your main code here, to run repeatedly:

scan();

//the goal is to pass through the dark zone, not go directly to it.
//if the dark zone is to the left, the bot will go straight.
//if the dark zone is to the right, the bot will turn 90 degrees first
if(darkestServoPosition > 90){
 turnRight(20);
 stop(500);
}

findForwardWall();
delay(500);
flashLights();

turnLeft(65);
stop(500);
flashLights();


bugHunt();
stop(500);
flashLights();

//turnLeft(90);
//stop(500);

//bugHunt();
//stop(500);

//turnLeft(135);
//stop(500);

//goForward(getDistance());

//tone(3, 440, 2000);
//
}


//use the motors to turn the robot to the right
void turnRight(int turnDegrees){
  
  rightMotor(-150);
  leftMotor(150);
  delay(turnDegrees * 5);
}

//use the motors to turn the robot to the left
void turnLeft(int turnDegrees){

  rightMotor(150);
  leftMotor(-150);
  delay(turnDegrees * 5);
}

void goForward(int distance){
  rightMotor(150);
  leftMotor(150);
  delay(distance);
}

void goForwardSlowly(int distance){
  rightMotor(130);
  leftMotor(130);
  delay(distance);
}

void goBackward(int distance){
  rightMotor(-150);
  leftMotor(-150);
  delay(distance);
}

void bugHunt(){

  while(getDistance() > 15){
    flashLights();
    goForwardSlowly(50);
    flashLights();
    beep();
    flashLights();
    }
  stop(2000);
 }



/*
 * Method to make the beeper beep for each loop
 */
void beep(){
int note = random(80, 550);
tone(3, note, random(500, 1000));  
}
/*
 * Method to make the lights flicker for each loop
 */
void flashLights(){
  digitalWrite(2, HIGH);
  delay(50);
  digitalWrite(2, LOW);
  delay(50);
  digitalWrite(2, HIGH);
  delay(50);
  digitalWrite(2, LOW);
  delay(50);
  digitalWrite(2, HIGH);
  delay(50);
  digitalWrite(2, LOW);
  delay(50);
  digitalWrite(2, HIGH);
  delay(50);
  digitalWrite(2, LOW); 
}

//This is my method for the robot to orient itself with the wall. It relies
//on the sonar readings increasing or decreasing after each turn
void findForwardWall(){

  distanceToNextWall = getDistance();
  Serial.println(distanceToNextWall);

  while(distanceToNextWall > 15){
    goForward(50);
    distanceToNextWall = getDistance();
    Serial.println(distanceToNextWall);
    stop(0);
  }

  stop(5);
  delay(1000);

  goBackward(200);
  stop(5);
  delay(1000);
  
  float distA = getDistance();

  turnLeft(20);
  stop(20);

  float distB = getDistance();

  //we turned towards the wall, wall is to the left
  if(distA > distB){
    while(distA > distB){
      distA = getDistance();
      turnLeft(20);
      stop(20);
      distB = getDistance();
      delay(500);
    }
  }
  else if(distA < distB){
    while(distA < distB){
      distA = getDistance();
      turnRight(20);
      stop(20);
      distB = getDistance();
      delay(500);
    }
  }
}

//This method is taken from SIK Circuit 5C Autonomous Robot
//it is used to operate the sonar and calculate the distance to an object in front of the
//robot. It is very important for orienting with the wall
float getDistance(){
  
  float echoTime;
  float calculatedDistance;

  delay(50);  //the constant-time pings were interfering with each other! I added
  //this delay to fix that.

  
  //10ms ultrasonic pulse
  digitalWrite(ping, HIGH);
  delayMicroseconds(10);
  digitalWrite(ping, LOW);

  echoTime = pulseIn(echo, HIGH);

  calculatedDistance = echoTime / 148;

  return calculatedDistance;
}

//This method stops the robot
void stop(int pause){

  rightMotor(0);
  leftMotor(0);
  delay(pause);
}


//These two methods are borrowed from SIK Circuit 5-C - Autonomous Robot
//they activate the motor based on the integer passed to them
void rightMotor(int power){

  if (power > 0){
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }
  else if(power < 0){
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }
  else if(power = 0){
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
  }
  
  analogWrite(PWMA, abs(power)); 
}

void leftMotor(int power){

  if (power > 0){
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  }
  else if(power < 0){
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }
  else{
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
  }
  
  analogWrite(PWMB, abs(power)); 
}

//Method for scanning the area to find the darkest spot
void scan(){
   
  int photoresistor;

  int servoPosition = 20; 

  while(servoPosition < 160){
    servoPosition++;
    theServo.write(servoPosition);
    photoresistor = analogRead(A0);
    Serial.println(photoresistor);
    Serial.println(servoPosition);
    delay(10);
    
    if(photoresistor > 25){
      if(photoresistor < darkestCorner){
        darkestCorner = photoresistor;
        darkestServoPosition = servoPosition;
      }
    }
  }
  theServo.write(20);
}







