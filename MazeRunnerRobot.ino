#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Arduino.h>
#include <vector>
#include <utility>  // For std::pair
#include <WiFi.h>
#include "defs.h"
#include "sensors.h"
#include "adjust.h"
#include "motors.h"
#include "comm.h"
#define WALL_DETECTION_THRESHOLD 120 // Adjust the value based on your sensor's range (in mm)

void setup() {
  Serial.begin(115200);
  while (!Serial) ;  // Wait for serial port to initialize
  pinMode(LEFT_LED_PIN, OUTPUT);
  pinMode(RIGHT_LED_PIN, OUTPUT);

  commSetup();
  move_pid();
  commReadData();

  setupSensors();  // Setup the sensors (defined in sensors.h)
// motorsSetup();  // Uncomment if you have a motors setup function
}

void loop() {
  // Get measurements from the sensors (right, forward, left sensors)
  getMeasurments();  // This will update the distances for Right, Forward, and Left

  // Print out sensor readings for debugging
  Serial.print("Left Distance: ");
  Serial.print(distance_Left);
  Serial.print(" mm, Right Distance: ");
  Serial.print(distance_Right);
  Serial.print(" mm, Forward Distance: ");
  Serial.println(distance_Forward);

  // Check if any wall is detected on either side
  if (distance_Forward < WALL_DETECTION_THRESHOLD) {
    // Decide whether to turn left or right based on which side has more space
    if (distance_Left < distance_Right) {
      Serial.println("Wall detected on the left, turning right...");
      turnRight();  // Replace with your actual motor control code for turning right
    } else {
      Serial.println("Wall detected on the right, turning left...");
      turnLeft();  // Replace with your actual motor control code for turning left
    }
  } else {
    // If no wall is detected, continue moving forward
    move_forward();  // Replace with your actual motor control code for moving forward
  }

  // Optionally, transmit sensor values for external monitoring (e.g., via Serial Monitor)
 // TransmitValues();  // This will print wall status and sensor distances, as defined in sensors.h
}
