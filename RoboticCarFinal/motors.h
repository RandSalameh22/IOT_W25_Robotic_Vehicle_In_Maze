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

float KpA = 0.7;  //left
float KiA = 0.005; 
float KdA = 0.25;

float KpB = 0.7;  //right
float KiB = 0.005; 
float KdB = 0.25;


float leftMotorCorrection = 1.0;  
float rightMotorCorrection = 1.0; // Default is 1.0

float balanceFactor = 1.0;  

// PID state variables
float errorSumA = 0.0;
float lastErrorA = 0.0;
float errorSumB = 0.0;
float lastErrorB = 0.0;
int pwmA = 0;
int pwmB = 0;
int targetSpeed = 120;           
unsigned long pidInterval = 20; // ms between PID checks
unsigned long lastPIDTime = 0;
int correctSpeed(int speed) {
  return constrain(speed, 0, 255);
}
//****************************************************************************************************************************************


void resetEncoders() {
    encoderValueA = 0;
    encoderValueB = 0;
}
//****************************************************************************************************************************************

// Change function signature to accept 'volatile' for all related variables
void calculateRPM(volatile unsigned long &lastTickTime, volatile long &encoderValue, volatile float &RPM, int encoderTicksPerRev) {
  unsigned long currentTime = millis();
  unsigned long timeElapsed = currentTime - lastTickTime;

  // Ensure the time interval is sufficient to calculate RPM
  if (timeElapsed >= 100) {  // Calculate every 100 ms (adjust as needed)
    // Convert ticks to RPM
    RPM = (encoderValue / (float)encoderTicksPerRev) * (60000.0 / timeElapsed);  

    // Update the last tick time and reset encoder count
    lastTickTime = currentTime;
    encoderValue = 0;
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
//****************************************************************************************************************************************
void stop_moving() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}
//****************************************************************************************************************************************
int calculatePIDMotor(float currentRPM, float targetSpeed, float Kp, float Ki, float Kd, float& errorSum, float& lastError) {
  // Calculate the error
  float error = targetSpeed - currentRPM;

  // Accumulate the integral (error sum)
  errorSum += error;

  // Calculate the derivative (change in error)
  float derivative = error - lastError;

  // Update the last error
  lastError = error;

  // Compute PID output
  float output = (Kp * error) + (Ki * errorSum) + (Kd * derivative);

  // Constrain the output to valid PWM range (0-255)
  return correctSpeed(static_cast<int>(output));
}



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

  int pwmA = calculatePIDMotor(RPM_A, targetSpeed, KpA, KiA, KdA, errorSumA, lastErrorA);
  int pwmB = calculatePIDMotor(RPM_B, targetSpeed, KpB, KiB, KdB, errorSumB, lastErrorB);
  // Synchronize motor speeds for straight movement
  synchronizeMotors(pwmA, pwmB, RPM_A, RPM_B);

  // Set motor directions and speeds
  setMotorDirection(MotFwdA, MotRevA, PWMA, pwmA);
  setMotorDirection(MotFwdB, MotRevB, PWMB, pwmB);
}
//****************************************************************************************************************************************

void rotate_180() {
  // Define the turn duration in milliseconds (adjust based on your robot's speed and desired turn angle)
  move_backward(200);;
  unsigned long turnDuration = 1000;  // Example duration for a 180-degree turn, adjust as needed
    getMeasurments();
   while(distance_Forward<500)
   {


  // Rotate the robot right (clockwise)
   digitalWrite(MotFwdA, HIGH);
  digitalWrite(MotRevA, LOW);
  analogWrite(PWMA, 120);  // Set motor speed (0-255)

  digitalWrite(MotFwdB, LOW);
  digitalWrite(MotRevB, HIGH);
  analogWrite(PWMB, 120);  // Set motor speed (0-255)

  // Wait for the turn to complete
   getMeasurments();
  delay(300);
   getMeasurments();
}
  // Stop the robot after the turn
       

      resetEncoders();
      errorSumA = 0;
lastErrorA = 0;
errorSumB = 0;
lastErrorB = 0;
  stop_moving();
  
      resetEncoders();
      errorSumA = 0;
lastErrorA = 0;
errorSumB = 0;
lastErrorB = 0;
  

}

/*
void rotate_180() {
  // Reset encoders before starting the turn
  resetEncoders();

  // Define necessary parameters
  float wheelDiameter = 6.5;  // Wheel diameter in cm
  float trackWidth = 13.5;     // Distance between the two wheels in cm
  int countsPerRevolution = 720; // Adjust this based on your encoder specs

  // Calculate the target encoder counts for a 180-degree turn
  float halfTurnDistance = (PI * trackWidth) ;  // 180-degree arc length
  int targetCounts = (halfTurnDistance * countsPerRevolution) / (PI * wheelDiameter);

  // Start rotating the robot in place
  while (abs(encoderValueA) < targetCounts || abs(encoderValueB) < targetCounts) {
    digitalWrite(MotFwdA, HIGH);
    digitalWrite(MotRevA, LOW);
    analogWrite(PWMA, 180);

    digitalWrite(MotFwdB, LOW);
    digitalWrite(MotRevB, HIGH);
    analogWrite(PWMB, 180);

    // Continuously check encoder values
   
  }

  // Stop the motors after reaching target counts
  
  stop_moving();

  // Reset encoders and PID errors
  resetEncoders();
  errorSumA = 0;
  lastErrorA = 0;
  errorSumB = 0;
  lastErrorB = 0;
}
*/



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
calculateRPM(lastTickTimeA, encoderValueA, RPM_A, ENCODER_TICKS_PER_REV);
  calculateRPM(lastTickTimeB, encoderValueB, RPM_B, ENCODER_TICKS_PER_REV);

  // If there's an obstacle in front
  while (measureForward.RangeStatus != 4 && distance_Forward < 80) {
    getMeasurments();
    stop_moving();
    if (distance_Right > distance_Left+40) {
      rotate_right(200);
      continue;
    }
    if ( distance_Left > distance_Right+40) {
      rotate_left(200);
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
 int pidSpeedA = calculatePIDMotor(RPM_A, targetSpeed, KpA, KiA, KdA, errorSumA, lastErrorA);
int pidSpeedB = calculatePIDMotor(RPM_B, targetSpeed, KpB, KiB, KdB, errorSumB, lastErrorB);


  left_speed_adjustment = constrain(left_speed_adjustment + pidSpeedA, 70, 255);
  right_speed_adjustment = constrain(right_speed_adjustment + pidSpeedB, 70, 255);
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
  const int rightMotorSpeed = FULLSPEED * 0.35;
   // int leftSpeed, rightSpeed;

 // leftSpeed = calculatePIDMotor(encoderValueA, 255, KpA, KiA, KdA, errorSumA, lastErrorA);
  //rightSpeed = calculatePIDMotor(encoderValueB, 75, KpB, KiB, KdB, errorSumB, lastErrorB);  // Speed for the right motor, slower to create a right turn

  // Continue turning right for the specified duration
  // Move the left motor forward at full speed
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
 analogWrite(PWMA, leftMotorSpeed);
   //analogWrite(PWMA, constrain(leftSpeed, 0, 255));  // Apply PID speed control to the left motor


  // Move the right motor forward at reduced speed
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, rightMotorSpeed);
  //  analogWrite(PWMB, constrain(rightSpeed, 0, 255)); // Apply PID speed control to the right motor

    resetEncoders();

}

void move_left() {
  // Define the speed difference between the motors
    const int leftMotorSpeed = FULLSPEED * 0.35;
  //calculatePIDMotor(encoderValueA, FULLSPEED * 0.35, KpA, KiA, KdA, errorSumA, lastErrorA);;  // Speed for the left motor, slower to create a left turn
  const int rightMotorSpeed = FULLSPEED;
  // calculatePIDMotor(encoderValueB, FULLSPEED, KpB, KiB, KdB, errorSumB, lastErrorB); ;        // Speed for the right motor

  // Move the left motor forward at reduced speed
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
 // analogWrite(PWMA, constrain(leftMotorSpeed, 70, 255));
  analogWrite(PWMA, leftMotorSpeed);


  // Move the right motor forward at full speed
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, rightMotorSpeed); // Apply PID speed control to the right motor
    resetEncoders();

}



void turnLeft() {
  // Constants for determining when to stop turning
  const int wallDetectionThreshold = 180;  // Distance to consider a sensor has detected a wall
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
  const int wallDetectionThreshold = 180;  // Distance to consider a sensor has detected a wall
  const int frontClearThreshold = 250;     // Distance to consider the front is clear
  const int closeToCornerThreshold = 110;  // Distance to consider the robot too close to the corner

  double currentFrontDistance;
  double frontDistanceChange;
   errorSumA = 0;
   lastErrorA = 0;
   errorSumB = 0;
   lastErrorB = 0;
   //resetEncoders();


  while (true) {
    getMeasurments();
    TransmitValues();
    frontDistanceChange = distance_Forward - prev_distance_Forward;
    bool leftWallDetected = (distance_Left < wallDetectionThreshold);
    bool rightWallDetected = (distance_Right < wallDetectionThreshold);
    bool frontClear = (distance_Forward > frontClearThreshold);

    // finished the turn
    if (leftWallDetected && rightWallDetected) {
      stop_moving(); 
           resetEncoders();
 // Stop the motors
      break;       
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
  stop_moving();

      resetEncoders();
      errorSumA = 0;
lastErrorA = 0;
errorSumB = 0;
lastErrorB = 0;

 int leftSpeed = calculatePIDMotor(encoderValueA, targetSpeed, KpA, KiA, KdA, errorSumA, lastErrorA);
int rightSpeed = calculatePIDMotor(encoderValueB, targetSpeed, KpB, KiB, KdB, errorSumB, lastErrorB);


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

void move_forwardTime(int duration) {

  unsigned long startTime = millis();
  while (millis() - startTime < duration) {
    digitalWrite(AIN2, LOW);
    digitalWrite(AIN1, HIGH);
    analogWrite(PWMA, FULLSPEED);

    digitalWrite(BIN2, LOW);
    digitalWrite(BIN1, HIGH);
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


