#include <Arduino.h>
#include <vector>
#include <queue>
#include <tuple>
#include <utility>
#include <WiFi.h>
#include "defs.h"
#include "sensors.h"
#include "adjust.h"
#include "motors.h"
#include "comm.h"

// Directions enumeration
enum Direction { UP, DOWN, LEFT, RIGHT };

// Define maze dimensions
#define LEFT_DIR 0
#define RIGHT_DIR 1
#define UP_DIR 2
#define DOWN_DIR 3

// Movement directions and their deltas
const int dx[] = {-1, 1, 0, 0}; // UP, DOWN, LEFT, RIGHT
const int dy[] = {0, 0, -1, 1};

// Global variables
int startX, startY;
int endX, endY;
bool foundPath = false;

// Define constants
// const int ROWS = 6;
// const int COLS = 6;
#define MAX_PATH_LENGTH 36

// Struct to hold coordinate pairs
struct Pair {
  int first;
  int second;
};

// Struct to hold instructions
struct Instruction {
  String direction;
  int count;
};

// Global arrays for visited cells, path, and instructions
bool visitedCells[ROWS][COLS];
Pair path[MAX_PATH_LENGTH];
int pathSize = 0;

Instruction instructionsList[MAX_PATH_LENGTH];
int instructionsSize = 0;

// Function Prototypes
bool isValid(int row, int col);
bool dfs(int row, int col);
Pair countRightLeft(Pair old_element, Pair new_element);
void fillPathInstructions();
void executeInstructions();

// Function to check if a cell is valid
bool isValid(int row, int col) {
  if (row >= 0 && row < ROWS && col >= 0 && col < COLS &&
      maze[row][col] != "W" && !visitedCells[row][col]) {
    return true;
  }
  return false;
}

// Depth-First Search to find the goal
bool dfs(int row, int col) {
  if (maze[row][col] == "G") {
    path[pathSize].first = row;
    path[pathSize].second = col;
    pathSize++;
    Serial.print("Goal reached at: ");
    Serial.print(row);
    Serial.print(", ");
    Serial.println(col);
    return true;
  }

  visitedCells[row][col] = true;
  path[pathSize].first = row;
  path[pathSize].second = col;
  pathSize++;
  Serial.print("Visiting: ");
  Serial.print(row);
  Serial.print(", ");
  Serial.println(col);

  for (int i = 0; i < 4; i++) { // 4 Directions
    int newRow = row + dx[i];
    int newCol = col + dy[i];

    if (isValid(newRow, newCol)) {
      if (dfs(newRow, newCol)) {
        return true;
      }
    }
  }

  // Backtrack
  pathSize--;
  visitedCells[row][col] = false;
  return false;
}

// Function to count right and left turns based on movement
// Pair countRightLeft(Pair old_element, Pair new_element) {
//   Pair result = {0, 0};

//   // Determine movement direction
//   if (new_element.second == old_element.second + 1) { // Right
//     if (new_element.first != 0 && (maze[new_element.first - 1][new_element.second] == "F" || maze[new_element.first - 1][new_element.second] == "G")) {
//       result.second++; // Left
//     }
//     if (new_element.first != 3 && (maze[new_element.first + 1][new_element.second] == "F" || maze[new_element.first + 1][new_element.second] == "G")) {
//       result.first++; // Right
//     }
//   }
//   else if (new_element.second == old_element.second - 1) { // Left
//     if (new_element.first != 0 && (maze[new_element.first - 1][new_element.second] == "F" || maze[new_element.first - 1][new_element.second] == "G")) {
//       result.first++; // Right
//     }
//     if (new_element.first != 3 && (maze[new_element.first + 1][new_element.second] == "F" || maze[new_element.first + 1][new_element.second] == "G")) {
//       result.second++; // Left
//     }
//   }
//   else if (new_element.first == old_element.first + 1) { // Down
//     if (new_element.second != 0 && (maze[new_element.first][new_element.second - 1] == "F" || maze[new_element.first][new_element.second - 1] == "G")) {
//       result.second++; // Left
//     }
//     if (new_element.second != 3 && (maze[new_element.first][new_element.second + 1] == "F" || maze[new_element.first][new_element.second + 1] == "G")) {
//       result.first++; // Right
//     }
//   }
//   else if (new_element.first == old_element.first - 1) { // Up
//     if (new_element.second != 0 && (maze[new_element.first][new_element.second - 1] == "F" || maze[new_element.first][new_element.second - 1] == "G")) {
//       result.first++; // Right
//     }
//     if (new_element.second != 3 && (maze[new_element.first][new_element.second + 1] == "F" || maze[new_element.first][new_element.second + 1] == "G")) {
//       result.second++; // Left
//     }
//   }

//   return result;
// }

Pair countRightLeft(Pair old_element, Pair new_element) {
  int left = 0, right = 0;

  // Went right
  if (new_element.second == old_element.second + 1) {
    if (new_element.first != 0 && 
        (maze[new_element.first - 1][new_element.second] == "F" || maze[new_element.first - 1][new_element.second] == "G")) {
      left++;
    }
    if (new_element.first != ROWS - 1 && 
        (maze[new_element.first + 1][new_element.second] == "F" || maze[new_element.first + 1][new_element.second] == "G")) {
      right++;
    }
    return {right, left};
  }

  // Went left
  if (new_element.second == old_element.second - 1) {
    if (new_element.first != 0 && 
        (maze[new_element.first - 1][new_element.second] == "F" || maze[new_element.first - 1][new_element.second] == "G")) {
      right++;
    }
    if (new_element.first != ROWS - 1 && 
        (maze[new_element.first + 1][new_element.second] == "F" || maze[new_element.first + 1][new_element.second] == "G")) {
      left++;
    }
    return {right, left};
  }

  // Went down
  if (new_element.first == old_element.first - 1) {
    if (new_element.second != 0 && 
        (maze[new_element.first][new_element.second - 1] == "F" || maze[new_element.first][new_element.second - 1] == "G")) {
      left++;
    }
    if (new_element.second != COLS - 1 && 
        (maze[new_element.first][new_element.second + 1] == "F" || maze[new_element.first][new_element.second + 1] == "G")) {
      right++;
    }
    return {right, left};
  }

  // The last case (went up)
  if (new_element.second != 0 && 
      (maze[new_element.first][new_element.second - 1] =="F" || maze[new_element.first][new_element.second - 1] == "G")) {
    right++;
  }
  if (new_element.second != COLS - 1 && 
      (maze[new_element.first][new_element.second + 1] == "F" || maze[new_element.first][new_element.second + 1] == "G")) {
    left++;
  }

  return {right, left};
}


void fillPathInstructions() {
  if (pathSize == 0) return;

  int right_count = 0, left_count = 0;
  int left = 0, right = 0, up = 0, down = 0;
  Pair old = path[0];

  for (int i = 1; i < pathSize; i++) {
    Pair current = path[i];

    if (current.first == old.first + 1) { // Moving down
      down++;
      if ((left + up + right) > 0) {
        left = up = right = 0;
        instructionsList[instructionsSize].direction = (right_count > left_count) ? "right" : "left";
        instructionsList[instructionsSize].count = (right_count > left_count) ? right_count : left_count;
        instructionsSize++;
        right_count = left_count = 0;
      }
    } else if (current.first == old.first - 1) { // Moving up
      up++;
      if ((left + right + down) > 0) {
        left = right = down = 0;
        instructionsList[instructionsSize].direction = (right_count > left_count) ? "right" : "left";
        instructionsList[instructionsSize].count = (right_count > left_count) ? right_count : left_count;
        instructionsSize++;
        right_count = left_count = 0;
      }
    } else if (current.second == old.second + 1) { // Moving right
      right++;
      if ((left + up + down) > 0) {
        left = up = down = 0;
        instructionsList[instructionsSize].direction = (right_count > left_count) ? "right" : "left";
        instructionsList[instructionsSize].count = (right_count > left_count) ? right_count : left_count;
        instructionsSize++;
        right_count = left_count = 0;
      }
    } else { // Moving left
      left++;
      if ((right + up + down) > 0) {
        right = up = down = 0;
        instructionsList[instructionsSize].direction = (right_count > left_count) ? "right" : "left";
        instructionsList[instructionsSize].count = (right_count > left_count) ? right_count : left_count;
        instructionsSize++;
        right_count = left_count = 0;
      }
    }

    Pair result = countRightLeft(old, current);
    right_count += result.first;
    left_count += result.second;
    old = current;
  }

  // Handle any remaining instructions
  if (instructionsSize == 0) {
    instructionsList[instructionsSize].direction = "right";
    instructionsList[instructionsSize].count = 4;
    instructionsSize++;
    instructionsList[instructionsSize].direction = "left";
    instructionsList[instructionsSize].count = 4;
    instructionsSize++;


  }
}
// Function to execute movement based on instructions
// Function to execute movement based on instructions
void executeInstructions() {
  for (int i = 0; i < instructionsSize; i++) {
    Serial.print("Executing instruction ");
    Serial.print(i);
    Serial.print(": Move ");
    Serial.print(instructionsList[i].direction);
    Serial.print(" ");
    Serial.print(instructionsList[i].count);
    Serial.println(" steps.");

    for (int j = 0; j < instructionsList[i].count; j++) {
      Serial.print("Step ");
      Serial.print(j + 1);
      Serial.print(" of ");
      Serial.print(instructionsList[i].count);
      Serial.print(" in direction: ");
      Serial.println(instructionsList[i].direction);

      // if (instructionsList[i].direction == "right") {
      //   // moveRobot(RIGHT);
      // } else if (instructionsList[i].direction == "left") {
      //   // moveRobot(LEFT);
      // }
      // Add additional directions if needed
      delay(500); // Delay between movements for safety
    }
  }
  Serial.println("All instructions executed.");
}

void setup() {
  // Initialize Serial communication
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for Serial to be ready
  }

  // Initialize LEDs
  pinMode(LEFT_LED_PIN, OUTPUT);
  pinMode(RIGHT_LED_PIN, OUTPUT);

  // Setup communication and read maze data
  commSetup();
  commReadData(); // Read the maze data from the server into CommData

  // Initialize visited cells to false
  for (int i = 0; i < ROWS; i++) {
    for (int j = 0; j < COLS; j++) {
      visitedCells[i][j] = false;
    }
  }

  // Setup sensors
  setupSensors();
  // move_pid(); // Uncomment if PID movement is required

  // Find the start position 'S' in the maze
  bool foundStart = false;
  for (int i = 0; i < ROWS && !foundStart; i++) {
    for (int j = 0; j < COLS && !foundStart; j++) {
      if (maze[i][j] == "S") {
        startX = i;
        startY = j;
        foundStart = true;
        Serial.print("Start position found at: ");
        Serial.print(startX);
        Serial.print(", ");
        Serial.println(startY);
      }
    }
  }

  // Perform DFS to find the path to the goal
  if (!dfs(startX, startY)) {
    Serial.println("No path to the goal found.");
  } else {
    Serial.println("Path to the goal found:");
    // Print the path
    for (int i = 0; i < pathSize; i++) {
      Serial.print(i);
      Serial.print("'th element is: ");
      Serial.print(path[i].first);
      Serial.print(", ");
      Serial.println(path[i].second);
    }

    // Fill path instructions
    fillPathInstructions();

    // Print instructions
    Serial.println("Instructions:");
    Serial.println(instructionsSize);

    for (int i = 0; i < instructionsSize; i++) {
      Serial.print(i);
      Serial.print("'th instruction: ");
      Serial.print(instructionsList[i].direction);
      Serial.print(", ");
      Serial.println(instructionsList[i].count);
    }

    // Optionally, execute the instructions to move the robot
    executeInstructions();
  }
}

void loop() {
  if (vec_index > instructionsSize) {
    stop_moving();
    return;
  }

  getMeasurments();
  updateState();
  countJunctions();

    // TO DO : The situation where the robot is detect 3 walls and the still instructions remaining ----> Dynamic Maze Detect
    //  if (distance_Left <= WALL_DIST && distance_Right <= WALL_DIST && distance_Forward <= WALL_DIST) {
    //     Serial.println("Blocked: No way to proceed!");

    //     if (vec_index < instructionsList.size()) {
    //         Serial.println("Still instructions remaining, stopping for now.");
    //         stop_moving();
    //     } else {
    //         Serial.println("All instructions completed, stopping.");
    //     }
    //     return;
    // }


  if (vec_index < instructionsSize) {
    auto current_instruction = instructionsList[vec_index];
    if (current_instruction.direction == "right" && right_turns_counter >= current_instruction.count) {
      turnRight();
      right_turns_counter = 0; // Reset the counter after turning
      left_turns_counter = 0;
      vec_index++;  
    } 
    else if (current_instruction.direction == "left" && left_turns_counter >= current_instruction.count) {
      turnLeft();
      left_turns_counter = 0; // Reset the counter after turning
      right_turns_counter = 0;
      vec_index++;
    }
  }

  move_forward();

  if (vec_index == instructionsSize) {
    if (distance_Forward < 180) {
      if (distance_Left > distance_Right) {
        for (int i = 1; i < 6; i++) {
          digitalWrite(LEFT_LED_PIN, HIGH);
          digitalWrite(RIGHT_LED_PIN, LOW);
          rotate_left(500);
          digitalWrite(LEFT_LED_PIN, LOW);
          digitalWrite(RIGHT_LED_PIN, HIGH);
          rotate_right(500);
        }
      } else {
        for (int i = 1; i < 6; i++) {
          digitalWrite(LEFT_LED_PIN, LOW);
          digitalWrite(RIGHT_LED_PIN, HIGH);
          rotate_right(500);
          digitalWrite(LEFT_LED_PIN, HIGH);
          digitalWrite(RIGHT_LED_PIN, LOW);
          rotate_left(500);
        }
      }
      digitalWrite(LEFT_LED_PIN, LOW);
      digitalWrite(RIGHT_LED_PIN, LOW);
      stop_moving();
      vec_index++;
    }
  }

  adjust_course();

  delay(30);
} 

  // Main loop can remain empty or handle other tasks
  // delay(1000);
// #include <Wire.h>
// #include <Adafruit_VL53L0X.h>
// #include <Arduino.h>
// #include <vector>
// #include <utility>  // For std::pair
// #include <WiFi.h>
// #include "defs.h"
// #include "sensors.h"
// #include "adjust.h"
// #include "motors.h"
// #include "comm.h"
// #define WALL_DETECTION_THRESHOLD 120 // Adjust the value based on your sensor's range (in mm)

// void setup() {
//   Serial.begin(115200);
//   while (!Serial) ;  // Wait for serial port to initialize
//   pinMode(LEFT_LED_PIN, OUTPUT);
//   pinMode(RIGHT_LED_PIN, OUTPUT);

//   commSetup();
//   move_pid();
//   commReadData();

//   setupSensors();  // Setup the sensors (defined in sensors.h)
// // motorsSetup();  // Uncomment if you have a motors setup function
// }

// void loop() {
//   // Get measurements from the sensors (right, forward, left sensors)
//   getMeasurments();  // This will update the distances for Right, Forward, and Left

//   // Print out sensor readings for debugging
//   Serial.print("Left Distance: ");
//   Serial.print(distance_Left);
//   Serial.print(" mm, Right Distance: ");
//   Serial.print(distance_Right);
//   Serial.print(" mm, Forward Distance: ");
//   Serial.println(distance_Forward);

//   // Check if any wall is detected on either side
//   if (distance_Forward < WALL_DETECTION_THRESHOLD) {
//     // Decide whether to turn left or right based on which side has more space
//     if (distance_Left < distance_Right) {
//       Serial.println("Wall detected on the left, turning right...");
//       turnRight();  // Replace with your actual motor control code for turning right
//     } else {
//       Serial.println("Wall detected on the right, turning left...");
//       turnLeft();  // Replace with your actual motor control code for turning left
//     }
//   } else {
//     // If no wall is detected, continue moving forward
//     move_forward();  // Replace with your actual motor control code for moving forward
//   }

//   // Optionally, transmit sensor values for external monitoring (e.g., via Serial Monitor)
//  // TransmitValues();  // This will print wall status and sensor distances, as defined in sensors.h
// }
