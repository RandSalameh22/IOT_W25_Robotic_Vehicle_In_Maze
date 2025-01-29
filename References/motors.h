#include "esp32-hal-gpio.h"


#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Arduino.h>
#include <vector>
#include <utility>  // For std::pair
#include "defs.h"
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

  Serial.begin(115200);

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

void stop_moving() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}


void rotate_left(int duration) {
  // פסק זמן קריאת אינקודר
  detachInterrupt(digitalPinToInterrupt(encoderPinA1));
  detachInterrupt(digitalPinToInterrupt(encoderPinA2));
  detachInterrupt(digitalPinToInterrupt(encoderPinB1));
  detachInterrupt(digitalPinToInterrupt(encoderPinB2));

  unsigned long startTime = millis();
  double turn_speed = (0.2 * FULLSPEED); // מהירות סיבוב
  while (millis() - startTime < duration) {
    // סיבוב שמאלה (מנוע A אחורה, מנוע B קדימה)
    digitalWrite(MotFwdA, LOW);
    digitalWrite(MotRevA, HIGH);
    analogWrite(PWMA, turn_speed);

    digitalWrite(MotFwdB, HIGH);
    digitalWrite(MotRevB, LOW);
    analogWrite(PWMB, turn_speed);
  }
  
  // סיים את הפנייה והחזיר את קריאות האינקודר
  attachInterrupt(digitalPinToInterrupt(encoderPinA1), updateEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinA2), updateEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB1), updateEncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB2), updateEncoderB, CHANGE);

  stop_moving();
}

void rotate_right(int duration) {
  // פסק זמן קריאת אינקודר
  detachInterrupt(digitalPinToInterrupt(encoderPinA1));
  detachInterrupt(digitalPinToInterrupt(encoderPinA2));
  detachInterrupt(digitalPinToInterrupt(encoderPinB1));
  detachInterrupt(digitalPinToInterrupt(encoderPinB2));

  unsigned long startTime = millis();
  double turn_speed = (0.2 * FULLSPEED); // מהירות סיבוב
  while (millis() - startTime < duration) {
    // סיבוב ימינה (מנוע A קדימה, מנוע B אחורה)
    digitalWrite(MotFwdA, HIGH);
    digitalWrite(MotRevA, LOW);
    analogWrite(PWMA, turn_speed);

    digitalWrite(MotFwdB, LOW);
    digitalWrite(MotRevB, HIGH);
    analogWrite(PWMB, turn_speed);
  }

  // סיים את הפנייה והחזיר את קריאות האינקודר
  attachInterrupt(digitalPinToInterrupt(encoderPinA1), updateEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinA2), updateEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB1), updateEncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB2), updateEncoderB, CHANGE);

  stop_moving();
}


// Main turning functions
void turnLeft() {
  digitalWrite(LEFT_LED_PIN, HIGH);
  // Similar to rotate_left but handles conditions and sensors
  rotate_left(450); // Rotate left for 1 second as an example
  digitalWrite(LEFT_LED_PIN, LOW);
}

void turnRight() {
  digitalWrite(RIGHT_LED_PIN, HIGH);
  // Similar to rotate_right but handles conditions and sensors
  rotate_right(425); // Rotate right for 1 second as an example
  digitalWrite(RIGHT_LED_PIN, LOW);
}
