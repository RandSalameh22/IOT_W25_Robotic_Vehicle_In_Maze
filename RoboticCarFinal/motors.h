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
#define WHEEL_DIAMETER 6.5  // in cm (example, adjust as per your robot)
#define ENCODER_TICKS_PER_REV 712  // Number of encoder ticks per full wheel rotation


void stop_moving();
void rotate_right(int duration);
void rotate_full(long targetSteps);
void rotate_180();

void rotate_left(int duration);

void moveRight(int duration);
void move_left(int duration);
void turnRight();
void move_backward(int duration);




// Encoder pins for Motor A 
int encoderPinA1 = 35;
int encoderPinA2 = 34;
volatile int lastEncodedA = 0;
volatile long encoderValueA = 0;
volatile unsigned long lastTickTimeA = 0;
volatile float RPM_A = 0;

// Encoder pins for Motor B
int encoderPinB1 = 15;
int encoderPinB2 = 4;
volatile int lastEncodedB = 0;
volatile long encoderValueB = 0;
volatile unsigned long lastTickTimeB = 0;
volatile float RPM_B = 0;

// PID Parameters
float KpA = 1.0;  // Left motor
float KiA = 0.05;
float KdA = 0.1;

float KpB = 1.0;  // Right motor
float KiB = 0.05;
float KdB = 0.1;
float leftMotorCorrection = 1.0;  // Adjust this value (e.g., 1.05 or 0.95) based on motor performance
float rightMotorCorrection = 1.0; // Default is 1.0

float balanceFactor = 1.0;  

// PID state variables
float errorSumA = 0.0;
float lastErrorA = 0.0;
float errorSumB = 0.0;
float lastErrorB = 0.0;
int pwmA = 0;
int pwmB = 0;
int targetSpeed = 150;           
unsigned long pidInterval = 20; // ms between PID checks
unsigned long lastPIDTime = 0;
int correctSpeed(int speed) {
  return constrain(speed, 0, 255);
}
//****************************************************************************************************************************************
// Change function signature to accept 'volatile' for all related variables
void calculateRPM(volatile unsigned long &lastTickTime, volatile long &encoderValue, volatile float &RPM, int encoderTicksPerRev) {
  unsigned long currentTime = millis();
  unsigned long timeElapsed = currentTime - lastTickTime;

  if (timeElapsed >= 1000) {  // Calculate RPM every second
    RPM = (encoderValue / (float)encoderTicksPerRev) * 60.0;  // Convert ticks to RPM
    lastTickTime = currentTime;
    encoderValue = 0;  // Reset encoder count after RPM calculation
  }
}
//****************************************************************************************************************************************
void synchronizeMotors(int& leftSpeed, int& rightSpeed, float leftRPM, float rightRPM) {
  float rpmDifference = leftRPM - rightRPM;
  if (rpmDifference > 0) {
    rightSpeed += static_cast<int>(rpmDifference * 0.1); // Adjust multiplier for sensitivity
  } else {
    leftSpeed -= static_cast<int>(rpmDifference * 0.1);
  }
}

void stop_moving() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}
//****************************************************************************************************************************************
int calculatePIDMotorWithCorrection(int currentTicks, float Kp, float Ki, float Kd, float& errorSum, float& lastError, float correctionFactor) {
  float error = targetSpeed - currentTicks;
  errorSum += error;
  float derivative = error - lastError;
  lastError = error;

  int pidOutput = static_cast<int>(Kp * error + Ki * errorSum + Kd * derivative);
  return correctSpeed(static_cast<int>(pidOutput * correctionFactor));
}

// PID motor control logic for each motor
int calculatePIDMotor(int current_ticks, float Kp, float Ki, float Kd, float& errorSum, float& lastError) {
  float error = targetSpeed - current_ticks;
  errorSum += error;
  float derivative = error - lastError;
  lastError = error;
  return correctSpeed(static_cast<int>(Kp * error + Ki * errorSum + Kd * derivative));
}

// void rotate_full(long targetSteps) {
//     while (abs(encoderValueA) < targetSteps) {
      
//         setMotorDirection(MotFwdA, MotRevA, PWMA, 75);
//         setMotorDirection(MotFwdB, MotRevB, PWMB, -75);

//     }
//     stop_moving();
// }


//****************************************************************************************************************************************

// Function to set motor direction and speed
void setMotorDirection(int fwdPin, int revPin, int pwmPin, int pwmVal) {
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
//****************************************************************************************************************************************

// Update encoder values based on pin states
void updateEncoder(int pinA1, int pinA2, volatile long& encoderValue, volatile int& lastEncoded) {
  int MSB = digitalRead(pinA1);
  int LSB = digitalRead(pinA2);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;
  
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue--;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue++;
  
  lastEncoded = encoded;
}
//****************************************************************************************************************************************

// Update all encoders
void updateEncoderB() {
  updateEncoder(encoderPinB1, encoderPinB2, encoderValueB, lastEncodedB);
}
//****************************************************************************************************************************************

void updateEncoderA() {

    updateEncoder(encoderPinA1, encoderPinA2, encoderValueA, lastEncodedA);

}
//****************************************************************************************************************************************

void updateEncoders()
{
  updateEncoderB();
  updateEncoderA();
}

//****************************************************************************************************************************************

// Move motors with PID control
void move_pid() {
    Serial.begin(115200);

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

  // Calculate RPM for each motor
  calculateRPM(lastTickTimeA, encoderValueA, RPM_A, ENCODER_TICKS_PER_REV);
  calculateRPM(lastTickTimeB, encoderValueB, RPM_B, ENCODER_TICKS_PER_REV);
Serial.print("RPM_A: ");
Serial.println(RPM_A);
Serial.print("RPM_B: ");
Serial.println(RPM_B);

  pwmA = calculatePIDMotorWithCorrection(encoderValueA, KpA, KiA, KdA, errorSumA, lastErrorA, leftMotorCorrection);
  pwmB = calculatePIDMotorWithCorrection(encoderValueB, KpB, KiB, KdB, errorSumB, lastErrorB, rightMotorCorrection);

  // Synchronize motor speeds for straight movement
  synchronizeMotors(pwmA, pwmB, RPM_A, RPM_B);

  // Set motor directions and speeds
  setMotorDirection(MotFwdA, MotRevA, PWMA, pwmA);
  setMotorDirection(MotFwdB, MotRevB, PWMB, pwmB);
}
//****************************************************************************************************************************************

// Adjust motor speed to smooth movement
void smoothSpeedAdjustment(int& currentSpeed, int targetSpeed, int maxStep) {
  if (abs(currentSpeed - targetSpeed) > maxStep) {
    currentSpeed += (currentSpeed < targetSpeed ? maxStep : -maxStep);
  } else {
    currentSpeed = targetSpeed;
  }
}
void rotate_180() {
  // Define the turn duration in milliseconds (adjust based on your robot's speed and desired turn angle)
  unsigned long turnDuration = 1000;  // Example duration for a 180-degree turn, adjust as needed
    getMeasurments();
while(distance_Forward<500)
{


  // Rotate the robot right (clockwise)
  digitalWrite(MotFwdA, HIGH);
  digitalWrite(MotRevA, LOW);
  analogWrite(PWMA, 200);  // Set motor speed (0-255)

  digitalWrite(MotFwdB, LOW);
  digitalWrite(MotRevB, HIGH);
  analogWrite(PWMB, 200);  // Set motor speed (0-255)

  // Wait for the turn to complete
   getMeasurments();
  delay(10);
   getMeasurments();
}
  // Stop the robot after the turn
       

  stop_moving();

}




//****************************************************************************************************************************************
/*
void adjust_course(int distance_Left, int distance_Right) {
  updateEncoders();
  updateState();

  static int prevDistanceLeft = 0;
  static int prevDistanceRight = 0;

  const int CLOSE_WALL_THRESHOLD = 120;
  const int EXTREME_CLOSE_THRESHOLD = 90; // ערך נמוך יותר לתגובה מהירה יותר
  const double lambda = 0.8;              // ערך חזק יותר לתיקון
  const double extremeLambda = 1.2;       // תיקון אגרסיבי לקרבה גבוהה
  const int DEAD_ZONE = 5;                // הקטנת אזור מת

  // סינון קריאות סנסור מהיר יותר
  distance_Left = 0.3 * prevDistanceLeft + 0.7 * distance_Left;
  distance_Right = 0.3 * prevDistanceRight + 0.7 * distance_Right;
  prevDistanceLeft = distance_Left;
  prevDistanceRight = distance_Right;

  int left_speed_adjustment = targetSpeed;
  int right_speed_adjustment = targetSpeed;

  // עצירה במקרה של כל הקירות
  if (walls_current == all_walls) {
    stop_moving();
    delay(100);
    return;
  }

  switch (walls_current) {
   

    case both_walls:
      if (distance_Right < CLOSE_WALL_THRESHOLD) {
        left_speed_adjustment -= lambda * (CLOSE_WALL_THRESHOLD - distance_Right);
      }
      if (distance_Right < EXTREME_CLOSE_THRESHOLD) {
       // left_speed_adjustment -= extremeLambda * (EXTREME_CLOSE_THRESHOLD - distance_Right);
        // right_speed_adjustment += 0.3 * (CLOSE_WALL_THRESHOLD - distance_Left);
           left_speed_adjustment = distance_Right*0.6;


      }
      if (distance_Left < CLOSE_WALL_THRESHOLD) {
        right_speed_adjustment -= lambda * (CLOSE_WALL_THRESHOLD - distance_Left);
      }
      if (distance_Left < EXTREME_CLOSE_THRESHOLD) {
        //right_speed_adjustment -= extremeLambda * (EXTREME_CLOSE_THRESHOLD - distance_Left);
          //left_speed_adjustment += 0.3 * (CLOSE_WALL_THRESHOLD - distance_Left);
                    right_speed_adjustment = distance_Left*0.6;


        
      }
      break;

    case left_wall:
      if (distance_Left < CLOSE_WALL_THRESHOLD) {
        right_speed_adjustment -= lambda * (CLOSE_WALL_THRESHOLD - distance_Left);
      }
      if (distance_Left < EXTREME_CLOSE_THRESHOLD) {
        right_speed_adjustment -= extremeLambda * (EXTREME_CLOSE_THRESHOLD - distance_Left);
      }
      
      break;

    case right_wall:
      if (distance_Right < CLOSE_WALL_THRESHOLD) {
        left_speed_adjustment -= lambda * (CLOSE_WALL_THRESHOLD - distance_Right);
      }
      if (distance_Right < EXTREME_CLOSE_THRESHOLD) {
        left_speed_adjustment -= extremeLambda * (EXTREME_CLOSE_THRESHOLD - distance_Right);
      }
      
      break;

    default:
      break;
  }

  int left_pid_speed = calculatePIDMotor(encoderValueA, KpA, KiA, KdA, errorSumA, lastErrorA);
  int right_pid_speed = calculatePIDMotor(encoderValueB, KpB, KiB, KdB, errorSumB, lastErrorB);

  int left_final_speed = left_speed_adjustment + left_pid_speed;
  int right_final_speed = (right_speed_adjustment + right_pid_speed) * balanceFactor;

  // הקטנת אזור המת
  if (abs(left_final_speed - right_final_speed) < DEAD_ZONE) {
    left_final_speed = right_final_speed = targetSpeed;
  }

  left_final_speed = correctSpeed(left_final_speed);
  right_final_speed = correctSpeed(right_final_speed);

  // הבטחה למהירות מינימלית
  if (left_final_speed < 70) left_final_speed = 70; // מהירות בסיסית גבוהה יותר
  if (right_final_speed < 70) right_final_speed = 70;

  Serial.print(" Left Speed: ");
  Serial.print(left_final_speed);
  Serial.print(" Right Speed: ");
  Serial.println(right_final_speed);

  setMotorDirection(MotFwdA, MotRevA, PWMA, left_final_speed);
  setMotorDirection(MotFwdB, MotRevB, PWMB, right_final_speed);
}
*/

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
 
 // עצירה במקרה של כל הקירות
  if (walls_current == all_walls) {
    stop_moving();
    delay(100);
    return;
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

//****************************************************************************************************************************************

void move_forward() {
  getMeasurments();
//adjust_course(distance_Left, distance_Right);
 if (walls_current == all_walls) {
    stop_moving();
    delay(100);
    return;
  }
  adjust_course();

 setMotorDirection(MotFwdA, MotRevA, PWMA, left_speed_adjustment);
  setMotorDirection(MotFwdB, MotRevB, PWMB, right_speed_adjustment);
  delay(50);  // Add a small delay for better stability
}

//****************************************************************************************************************************************


void resetEncoders() {
    encoderValueA = 0;
    encoderValueB = 0;
}

//****************************************************************************************************************************************


/*

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
//****************************************************************************************************************************************


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
//****************************************************************************************************************************************


// Function to gradually turn right by making the left motor move faster than the right motor
void move_right() {
  // Define the speed difference between the motors
  const int leftMotorSpeed = 180;          // Speed for the left motor
  const int rightMotorSpeed = 70;  // Speed for the right motor, slower to create a right turn

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
//****************************************************************************************************************************************

void move_left() {
  // Define the speed difference between the motors
  const int leftMotorSpeed = 70;  // Speed for the left motor, slower to create a left turn
  const int rightMotorSpeed = 170;        // Speed for the right motor

  // Move the left motor forward at reduced speed
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, leftMotorSpeed);

  // Move the right motor forward at full speed
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, rightMotorSpeed);
}
  
//****************************************************************************************************************************************

void move_left(int duration) {
  double turn_speed = (0.2 * FULLSPEED);
 // move_forward_helper();
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
//move_forward_helper();
  delay(580);
  stop_moving();
}
//****************************************************************************************************************************************

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
   // adjust_course();
    frontDistanceChange = distance_Forward - prev_distance_Forward;
    bool leftWallDetected = (distance_Left < wallDetectionThreshold);
    bool rightWallDetected = (distance_Right < wallDetectionThreshold);
    bool frontClear = (distance_Forward > frontClearThreshold);
    Serial.print("Left Distance: ");
  Serial.print(distance_Left);
  Serial.print(" mm, Right Distance: ");
  Serial.print(distance_Right);
  Serial.print(" mm, Forward Distance: ");
  Serial.println(distance_Forward);

    // Finished the turn
    if ((leftWallDetected && rightWallDetected )) {
      stop_moving();  // Stop the motors
      break;          // Exit the function
    }
    delay(10);
    if(distance_Left<100)
    {
       rotate_right(300);
       stop_moving();  // Stop the motors
      //break; 
    }

   
    // If the robot is too close to the left corner side, perform a right rotation
    if (distance_Forward < closeToCornerThreshold && distance_Left < 70) {
      stop_moving();
      delay(300);
      getMeasurments();
      if (distance_Forward < closeToCornerThreshold)
        rotate_right(450);  // Perform a full right rotation
      continue;
    }
    


  move_left();
    delay(2000);


    
  }
   attachInterrupt(digitalPinToInterrupt(encoderPinA1), updateEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinA2), updateEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB1), updateEncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB2), updateEncoderB, CHANGE);

}
//****************************************************************************************************************************************


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
    //adjust_course();
    frontDistanceChange = distance_Forward - prev_distance_Forward;
    bool leftWallDetected = (distance_Left < wallDetectionThreshold);
    bool rightWallDetected = (distance_Right < wallDetectionThreshold);
    bool frontClear = (distance_Forward > frontClearThreshold);
    Serial.print("Left Distance: ");
  Serial.print(distance_Left);
  Serial.print(" mm, Right Distance: ");
  Serial.print(distance_Right);
  Serial.print(" mm, Forward Distance: ");
  Serial.println(distance_Forward);

    // finished the turn
    if (leftWallDetected && rightWallDetected) {
      stop_moving();  // Stop the motors
      break;          // Exit the function
    }
    
    if (distance_Forward < 100 && distance_Right < 70) {
      stop_moving();
      delay(300);
      getMeasurments();
      if (distance_Forward < 100)
        rotate_left(100);  // Perform a full left rotation
      continue;
    }
    if(rightWallDetected)
    {
       rotate_left(103);
       stop_moving();  // Stop the motors
      //break; 
    }

    move_right();
    delay(500);
  }
 attachInterrupt(digitalPinToInterrupt(encoderPinA1), updateEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinA2), updateEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB1), updateEncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB2), updateEncoderB, CHANGE);
}

*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\/
void rotate_left(long targetSteps) {
    encoderValueA = 0;
    encoderValueB = 0;
    long initialEncoderValueB = 0;
    long currentSteps = 0;

    while (currentSteps < targetSteps) {
        digitalWrite(MotFwdA, LOW);
        digitalWrite(MotRevA, HIGH);
        analogWrite(PWMA, 90);

        digitalWrite(MotFwdB, HIGH);
        digitalWrite(MotRevB, LOW);
        analogWrite(PWMB, 90);

        currentSteps = encoderValueB;

    }
    stop_moving();
    encoderValueA = 0;
    encoderValueB = 0;
    resetEncoders();

}

void rotate_right(long targetSteps) {
    resetEncoders();
    while (abs(encoderValueA) < targetSteps) {
      
        setMotorDirection(MotFwdA, MotRevA, PWMA, 75);
        setMotorDirection(MotFwdB, MotRevB, PWMB, -75);

    }
    stop_moving();
    resetEncoders();
}

void correctAfterTurn() {
    float correctionFactor = 0.5;
    long remainingSteps = abs(encoderValueA - encoderValueB);

    if (remainingSteps > 0) {
        if (encoderValueA > encoderValueB) {
            setMotorDirection(MotFwdA, MotRevA, PWMA, 100 * correctionFactor);
        }
        else {
            setMotorDirection(MotFwdB, MotRevB, PWMB, 100 * correctionFactor);
        }
    }
    stop_moving();
    resetEncoders();
}
void move_forward_helper()
  {
      digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 120);

  // Move Motor B forward at defined speed
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, 90);

  }
  long calculateStepsForTurn(float angle) {
  const float wheel_distance = 135.0;  // Distance between wheels in mm
  const float wheel_diameter = 65.0;   // Diameter in mm
  const long steps_per_revolution = 720;
  const float pi = 3.14159265358979323846;
  float calibration_factor = 1.2;  // Adjust for better accuracy

  float turn_circumference = pi * wheel_distance * (angle / 360.0);
  float wheel_circumference = pi * wheel_diameter;

  long steps = (long)((steps_per_revolution * turn_circumference) / wheel_circumference * calibration_factor);

 

  return steps;
}

void turnLeft() {
  getMeasurments();
while(distance_Forward>165)
{
  move_forward_helper();
    getMeasurments();
    delay(10);

}
    rotate_left(calculateStepsForTurn(180));
    getMeasurments();
while(distance_Left>200)
{
  move_forward_helper();
    getMeasurments();
    delay(10);

}

    resetEncoders();

    // correctAfterTurn();
}

void turnRight() {
    getMeasurments();
while(distance_Forward>165)
{
  move_forward_helper();
    getMeasurments();
    delay(10);

}
    rotate_right(calculateStepsForTurn(160));
      getMeasurments();
while(distance_Right>200)
{
  move_forward_helper();
    getMeasurments();
    delay(10);

}
    resetEncoders();

    
    //correctAfterTurn();
}
*/




void rotate_left(int duration) {
  // digitalWrite(LEFT_LED_PIN, HIGH);
  // digitalWrite(RIGHT_LED_PIN, HIGH);

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
  resetEncoders();

  // digitalWrite(LEFT_LED_PIN, LOW);
  // digitalWrite(RIGHT_LED_PIN, LOW);
}


void rotate_right(int duration) {
  // digitalWrite(LEFT_LED_PIN, HIGH);
  // digitalWrite(RIGHT_LED_PIN, HIGH);
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
      resetEncoders();

  // digitalWrite(LEFT_LED_PIN, LOW);
  // digitalWrite(RIGHT_LED_PIN, LOW);
}

// Function to gradually turn right by making the left motor move faster than the right motor
void move_right() {
  // Define the speed difference between the motors
  const int leftMotorSpeed = FULLSPEED;          // Speed for the left motor
  const int rightMotorSpeed = FULLSPEED * 0.35;  // Speed for the right motor, slower to create a right turn

  // Continue turning right for the specified duration
  // Move the left motor forward at full speed
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, leftMotorSpeed);

  // Move the right motor forward at reduced speed
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, rightMotorSpeed);
    resetEncoders();

}

void move_left() {
  // Define the speed difference between the motors
  const int leftMotorSpeed = FULLSPEED * 0.40;  // Speed for the left motor, slower to create a left turn
  const int rightMotorSpeed = FULLSPEED ;        // Speed for the right motor

  // Move the left motor forward at reduced speed
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, leftMotorSpeed);

  // Move the right motor forward at full speed
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, rightMotorSpeed);
      resetEncoders();

}



void turnLeft() {
  // Constants for determining when to stop turning
  const int wallDetectionThreshold = 170;  // Distance to consider a sensor has detected a wall
  const int frontClearThreshold = 250;     // Distance to consider the front is clear
  const int closeToCornerThreshold = 110;  // Distance to consider the robot too close to the corner

  double frontDistanceChange;

  while (true) {
    getMeasurments();
    TransmitValues();
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
    if (distance_Forward < closeToCornerThreshold && distance_Left < 75) {
      stop_moving();
      delay(300);
      getMeasurments();
      if (distance_Forward < closeToCornerThreshold)
        rotate_right(200);  // Perform a full right rotation
      continue;
    }
    if(distance_Left<75)
    {
      
      rotate_right(200);
      break;
    }


    move_left();
    delay(200);

  }
      resetEncoders();
      errorSumA = 0;
   lastErrorA = 0;
   errorSumB = 0;
   lastErrorB = 0;
   stop_moving();
  
}


void turnRight() {
  // Constants for determining when to stop turning
  const int wallDetectionThreshold = 170;  // Distance to consider a sensor has detected a wall
  const int frontClearThreshold = 250;     // Distance to consider the front is clear
  const int closeToCornerThreshold = 110;  // Distance to consider the robot too close to the corner

  double currentFrontDistance;
  double frontDistanceChange;

  while (true) {
    getMeasurments();
    TransmitValues();
    frontDistanceChange = distance_Forward - prev_distance_Forward;
    bool leftWallDetected = (distance_Left < wallDetectionThreshold);
    bool rightWallDetected = (distance_Right < wallDetectionThreshold);
    bool frontClear = (distance_Forward > frontClearThreshold);

    // finished the turn
    if (leftWallDetected && rightWallDetected) {
      stop_moving();  // Stop the motors
      break;       
     resetEncoders();
   // Exit the function
    }


    if (distance_Forward < closeToCornerThreshold && distance_Right < 70) {
      stop_moving();
      delay(300);
      getMeasurments();
      if (distance_Forward < closeToCornerThreshold)
        rotate_left(200);  // Perform a full left rotation
      continue;
    }
    if(distance_Right < 70)
    {

      rotate_right(200);
            break;

    }

    move_right();
    delay(200);
  }
      resetEncoders();
      errorSumA = 0;
lastErrorA = 0;
errorSumB = 0;
lastErrorB = 0;
stop_moving();
  int leftSpeed = calculatePIDMotor(encoderValueA, KpA, KiA, KdA, errorSumA, lastErrorA);
  int rightSpeed = calculatePIDMotor(encoderValueB, KpB, KiB, KdB, errorSumB, lastErrorB);

  // Set the PID-controlled motor speeds
  analogWrite(PWMA, constrain(leftSpeed, 0, 80));   // Limit speed between 0-255
  analogWrite(PWMB, constrain(rightSpeed, 0, 80)); // Limit speed between 0-255


}



void move_backward(int duration) {

  unsigned long startTime = millis();
  while (millis() - startTime < duration) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, FULLSPEED);

    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, FULLSPEED);
  }
  stop_moving();
}


 long calculateStepsForTurn(float angle) {
  const float wheel_distance = 135.0;  // Distance between wheels in mm
  const float wheel_diameter = 65.0;   // Diameter in mm
  const long steps_per_revolution = 720;
  const float pi = 3.14159265358979323846;
  float calibration_factor = 1.2;  // Adjust for better accuracy

  float turn_circumference = pi * wheel_distance * (angle / 360.0);
  float wheel_circumference = pi * wheel_diameter;

  long steps = (long)((steps_per_revolution * turn_circumference) / wheel_circumference * calibration_factor);

 

  return steps;
}


