#include "esp32-hal-gpio.h"


#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Arduino.h>
#include <vector>
#include <utility>  // For std::pair
#include "defs.h"
#include "sensors.h"

#define MotFwdA  12  // Motor A Forward pin - left motor
#define MotRevA  14  // Motor A Reverse pin - left motor
#define MotFwdB  25  // Motor B Forward pin - right motor
#define MotRevB  33  // Motor B Reverse pin - right motor
#define STBY     27  // Standby pin
#define PWMA     13  // PWM control for Motor A speed
#define PWMB     26  // PWM control for Motor B speed

// Encoder pins for Motor A
int encoderPinA1 = 35;
int encoderPinA2 = 34;
volatile int lastEncodedA = 0;
volatile long encoderValueA = 0;

// Encoder pins for Motor B
int encoderPinB1 = 15;
int encoderPinB2 = 4;
volatile int lastEncodedB = 0;
volatile long encoderValueB = 0;

// -------------------------------
// PID Parameters
// -------------------------------
// PID Parameters
float KpA = 1.1;  // Motor A Proportional
float KiA = 0.01; 
float KdA = 0.2;

// ערכים חדשים עבור מנוע B (ימני) כדי לשפר את הביצועים
float KpB = 1.3;  // תגדיל את Kp למנוע B כדי להגביר את התגובה
float KiB = 0.015; // התאם את Ki למנוע B
float KdB = 0.25;  // תוכל לנסות להגדיל את Kd כדי לשפר את היציבות


float balanceFactor = 1.0;  // תוודא שהמאזן מתאים, תוכל לנסות גם 1.0

// Adjust PID for better balance and less drift




// State tracking for PID
float errorSumA = 0.0;
float lastErrorA = 0.0;
float errorSumB = 0.0;
float lastErrorB = 0.0;

// Speed tracking
int motorSpeedA = 0;
int motorSpeedB = 0;

// Motor PWM commands
int pwmA = 0;
int pwmB = 0;
int targetSpeed =100;           // [ticks per sample]
unsigned long pidInterval = 20; // ms between PID checks
unsigned long lastPIDTime = 0;

void move_forward();
void stop_moving();
void rotate_right(int duration);
void rotate_left(int duration);

void moveRight(int duration);
void move_left(int duration);
void turnRight();
void move_backward(int duration);

void move_pid();

void setMotorForward(int fwdPin, int revPin, int pwmPin, int pwmVal) {
  if (pwmVal < 0) {
    digitalWrite(fwdPin, LOW);
    digitalWrite(revPin, HIGH);
    analogWrite(pwmPin, -pwmVal);
  } else {
    digitalWrite(fwdPin, HIGH);
    digitalWrite(revPin, LOW);
    analogWrite(pwmPin, pwmVal);
  }
}

// -------------------------------
// Encoder Update Functions
// -------------------------------
void updateEncoderA() {
  int MSB = digitalRead(encoderPinA1);
  int LSB = digitalRead(encoderPinA2);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncodedA << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValueA--;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValueA++;
  lastEncodedA = encoded;
}

void updateEncoderB() {
  int MSB = digitalRead(encoderPinB1);
  int LSB = digitalRead(encoderPinB2);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncodedB << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValueB--;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValueB++;
  lastEncodedB = encoded;
}

void move_pid() {
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(MotFwdA, OUTPUT);
  pinMode(MotRevA, OUTPUT);
  pinMode(MotFwdB, OUTPUT);
  pinMode(MotRevB, OUTPUT);


  pinMode(encoderPinA1, INPUT_PULLUP);
  pinMode(encoderPinA2, INPUT_PULLUP);
  pinMode(encoderPinB1, INPUT_PULLUP);
  pinMode(encoderPinB2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderPinA1), updateEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinA2), updateEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB1), updateEncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB2), updateEncoderB, CHANGE);

  pwmA = 0;
  pwmB = 0;
  setMotorForward(MotFwdA, MotRevA, PWMA, pwmA);
  setMotorForward(MotFwdB, MotRevB, PWMB, pwmB);
}

void move_forward() {
   unsigned long now = millis();

  if (now - lastPIDTime >= pidInterval) {
    lastPIDTime = now;

    // Calculate speed from encoders
    int deltaA = encoderValueA;
    int deltaB = encoderValueB;

    encoderValueA = 0;
    encoderValueB = 0;

    motorSpeedA = deltaA;
    motorSpeedB = deltaB;

    // Calculate errors
    float errorA = targetSpeed - motorSpeedA;
    float errorB = targetSpeed - motorSpeedB;

    // PID עבור מנוע A
errorSumA += errorA;
errorSumA = constrain(errorSumA, -1000, 1000); // מניעת wind-up

float dErrorA = errorA - lastErrorA;
lastErrorA = errorA;

float adjustA = (KpA * errorA) + (KiA * errorSumA) + (KdA * dErrorA);
pwmA += (int)adjustA;
pwmA = constrain(pwmA, 0, 255);

// PID עבור מנוע B
errorSumB += errorB;
errorSumB = constrain(errorSumB, -1000, 1000); // מניעת wind-up

float dErrorB = errorB - lastErrorB;
lastErrorB = errorB;

float adjustB = (KpB * errorB) + (KiB * errorSumB) + (KdB * dErrorB);
pwmB += (int)adjustB * balanceFactor;  // שימוש במאזן (balanceFactor)
pwmB = constrain(pwmB, 0, 255);


    // Set motors
    setMotorForward(MotFwdA, MotRevA, PWMA, pwmA);
    setMotorForward(MotFwdB, MotRevB, PWMB, pwmB);
  }
}
//////////////////////////////////////////////
void stop_moving() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}
///////////////////////////////////////////

int correctSpeed(int speed) {
  if (speed <= 0)
    return 0;
  if (speed >= 255)
    return 255;
  return speed;
}


void adjust_course() {

  // If there's an obstacle in front
  while (measureForward.RangeStatus != 4 && distance_Forward < 80) {
    getMeasurments();
    stop_moving();
    if (distance_Right > distance_Left+40) {
      digitalWrite(RIGHT_LED_PIN, HIGH);
      rotate_right(200);
      digitalWrite(RIGHT_LED_PIN, LOW);
      continue;
    }
    if ( distance_Left > distance_Right+40) {
      digitalWrite(LEFT_LED_PIN, HIGH);
      rotate_left(200);
      digitalWrite(LEFT_LED_PIN, LOW);
      continue;
    }
    delay(500);  // Short pause before turning
  }
 

  right_speed_adjustment = RIGHT_SPEED;
  left_speed_adjustment = LEFT_SPEED;

  double diff_left_distaces = distance_Left - prev_distance_Left;
  double diff_right_distaces = distance_Right - prev_distance_Right;
  bool closing_to_left = diff_left_distaces <= 0;
  bool closing_to_right = diff_right_distaces <= 0;
  double lamda = 0.7;

  // If the robot is too close to the right wall, steer left
  switch (walls_current) {
    case both_walls:
      if (distance_Right < 180 && closing_to_right) {
        if (distance_Right < 95) {
          left_speed_adjustment = distance_Right*0.6;
        } else {
          left_speed_adjustment = LEFT_SPEED - lamda * (180 - distance_Right);
        }
      }
      if (distance_Left < 180 && closing_to_left) {
        if (distance_Left < 95) {
          right_speed_adjustment = distance_Left*0.6;

        } else {
          right_speed_adjustment = RIGHT_SPEED - lamda * (180 - distance_Left);
        }
      }
      break;

    case left_wall:
      if (closing_to_left && distance_Left < 120) {
        right_speed_adjustment = RIGHT_SPEED - lamda * (120 - distance_Left);
      } else if (!closing_to_left && distance_Left > 120) {
        left_speed_adjustment = LEFT_SPEED - lamda * (120 - distance_Left / 2);
      }
      break;

    case right_wall:

      if (closing_to_right && distance_Right < 120) {
        left_speed_adjustment = LEFT_SPEED - lamda * (120 - distance_Right);
      } else if (!closing_to_right && distance_Right > 120) {
        right_speed_adjustment = RIGHT_SPEED - lamda * (120 - distance_Right / 2);
      }
      break;

    default:
      break;
  }
}

///////////////////////////////////////////
void rotate_left(int duration) {

  unsigned long startTime = millis();
  double turn_speed = (0.2 * FULLSPEED);
  while (millis() - startTime < duration) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, turn_speed);

    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, turn_speed);
  }
  stop_moving();
  
}


void rotate_right(int duration) {
 
  unsigned long startTime = millis();
  double turn_speed = (0.2 * FULLSPEED);
  while (millis() - startTime < duration) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, turn_speed);

    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, turn_speed);
  }
  stop_moving();
 
}

// Function to gradually turn right by making the left motor move faster than the right motor
void move_right() {
  // Define the speed difference between the motors
  const int leftMotorSpeed = 200;          // Speed for the left motor
  const int rightMotorSpeed = 80;  // Speed for the right motor, slower to create a right turn

  // Continue turning right for the specified duration
  // Move the left motor forward at full speed
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, leftMotorSpeed);

  // Move the right motor forward at reduced speed
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, rightMotorSpeed);
}

void move_left() {
  // Define the speed difference between the motors
  const int leftMotorSpeed = FULLSPEED * 0.35;  // Speed for the left motor, slower to create a left turn
  const int rightMotorSpeed = 220;        // Speed for the right motor

  // Move the left motor forward at reduced speed
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, leftMotorSpeed);

  // Move the right motor forward at full speed
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, rightMotorSpeed);
}
  void move_forward_helper()
  {
      digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, left_speed_adjustment);

  // Move Motor B forward at defined speed
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, right_speed_adjustment);

  }


void move_left(int duration) {
  double turn_speed = (0.2 * FULLSPEED);
  move_forward_helper();
  delay(790);
  unsigned long startTime = millis();
  while (millis() - startTime < duration) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, turn_speed);

    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, turn_speed);
  }
move_forward_helper();
  delay(580);
  stop_moving();
}

void turnLeft() {
  detachInterrupt(digitalPinToInterrupt(encoderPinA1));
  detachInterrupt(digitalPinToInterrupt(encoderPinA2));
  detachInterrupt(digitalPinToInterrupt(encoderPinB1));
  detachInterrupt(digitalPinToInterrupt(encoderPinB2));
  

  // Constants for determining when to stop turning
   // Constants for determining when to stop turning
  const int wallDetectionThreshold = 150;  // Distance to consider a sensor has detected a wall
  const int frontClearThreshold = 250;     // Distance to consider the front is clear
  const int closeToCornerThreshold = 100;  // Distance to consider the robot too close to the corner

  double frontDistanceChange;

  while (true) {
    getMeasurments();
    TransmitValues();
    adjust_course();
    frontDistanceChange = distance_Forward - prev_distance_Forward;
    bool leftWallDetected = (distance_Left < wallDetectionThreshold);
    bool rightWallDetected = (distance_Right < wallDetectionThreshold);
    bool frontClear = (distance_Forward > frontClearThreshold);

    // Finished the turn
    if (leftWallDetected && rightWallDetected) {
      stop_moving();  // Stop the motors
      break;          // Exit the function
    }

   
    // If the robot is too close to the left corner side, perform a right rotation
    if (distance_Forward < closeToCornerThreshold && distance_Left < 70) {
      stop_moving();
      delay(300);
      getMeasurments();
      if (distance_Forward < closeToCornerThreshold)
        rotate_right(210);  // Perform a full right rotation
      continue;
    }


  move_left();
    delay(10);


    
  }
   attachInterrupt(digitalPinToInterrupt(encoderPinA1), updateEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinA2), updateEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB1), updateEncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB2), updateEncoderB, CHANGE);

}


void turnRight() {
 detachInterrupt(digitalPinToInterrupt(encoderPinA1));
  detachInterrupt(digitalPinToInterrupt(encoderPinA2));
  detachInterrupt(digitalPinToInterrupt(encoderPinB1));
  detachInterrupt(digitalPinToInterrupt(encoderPinB2));

  // Constants for determining when to stop turning
  const int wallDetectionThreshold = 150;  // Distance to consider a sensor has detected a wall
  const int frontClearThreshold = 250;     // Distance to consider the front is clear
  const int closeToCornerThreshold = 100;  // Distance to consider the robot too close to the corner

  double currentFrontDistance;
  double frontDistanceChange;

  while (true) {
    getMeasurments();
    TransmitValues();
    adjust_course();
    frontDistanceChange = distance_Forward - prev_distance_Forward;
    bool leftWallDetected = (distance_Left < wallDetectionThreshold);
    bool rightWallDetected = (distance_Right < wallDetectionThreshold);
    bool frontClear = (distance_Forward > frontClearThreshold);

    // finished the turn
    if (leftWallDetected && rightWallDetected) {
      stop_moving();  // Stop the motors
      break;          // Exit the function
    }

    /*
    if (distance_Forward < closeToCornerThreshold && distance_Right < 70) {
      stop_moving();
      delay(300);
      getMeasurments();
      if (distance_Forward < closeToCornerThreshold)
        rotate_left(210);  // Perform a full left rotation
      continue;
    }*/

    move_right();
    delay(10);
  }
 attachInterrupt(digitalPinToInterrupt(encoderPinA1), updateEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinA2), updateEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB1), updateEncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB2), updateEncoderB, CHANGE);
}





