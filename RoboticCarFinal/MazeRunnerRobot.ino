#include <Arduino.h>
#include <vector>
#include <queue>
#include <tuple>
#include <utility>
#include <WiFi.h>
#include "defs.h"
#include "sensors.h"
#include "motors.h"
#include "comm.h"


#include <queue>
#include <cmath> // For abs()

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
#define MAX_PATH_LENGTH 100

// Struct to hold coordinate pairs
struct Pair {
  int first;
  int second;
};

struct Start{
  int x;
  int y;
};
// Struct to hold instructions
struct Instruction {
  String direction;
  int count;
};
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////

// Global arrays for visited cells, path, and instructions
bool visitedCells[ROWS][COLS];
Pair path[MAX_PATH_LENGTH];
int pathSize = 0;

Start StartLocAfterTurn[MAX_PATH_LENGTH];
Instruction instructionsList[MAX_PATH_LENGTH];
int instructionsSize = 0;

// Function Prototypes
bool isValid(int row, int col);
bool dfs(int row, int col);
Pair countRightLeft(Pair old_element, Pair new_element);
void fillPathInstructions();
void executeInstructions();
void resetForNewDFS();

// Function to check if a cell is valid
bool isValid(int row, int col) {
  if (row >= 0 && row < ROWS && col >= 0 && col < COLS &&
      maze[row][col] != "W" && !visitedCells[row][col]) {
    return true;
  }
  return false;
}

// Global variables to store the shortest path and its length
// Global variables
int shortestPathLength = MAX_PATH_LENGTH;  // Initialize with maximum path length
Pair finalPath[MAX_PATH_LENGTH];  // To store the shortest path at the end
/*
// DFS Helper function
bool dfsHelper(int row, int col, int pathLength) {
  if (maze[row][col] == "G") { // Goal reached
    // Store the path to the goal in the finalPath array
    path[pathLength].first = row;
    path[pathLength].second = col;
    pathSize = pathLength + 1; // Set pathSize to the length of the path
    Serial.print("Goal reached at: ");
    Serial.print(row);
    Serial.print(", ");
    Serial.println(col);

    // Store the current path in finalPath[] to preserve the full path
    for (int i = 0; i <= pathLength; i++) {
      finalPath[i] = path[i];
    }

    // Print the path found
    Serial.println("Path to the goal found:");
    for (int i = 0; i <= pathLength; i++) {
      Serial.print("Step ");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(finalPath[i].first);
      Serial.print(", ");
      Serial.println(finalPath[i].second);
    }
    return true; // Goal reached, stop further recursion
  }

  // Mark the current cell as visited
  visitedCells[row][col] = true;

  // Save current cell in the path
  path[pathLength].first = row;
  path[pathLength].second = col;

  // Explore all 4 possible directions (up, down, left, right)
  for (int i = 0; i < 4; i++) {
    int newRow = row + dx[i];
    int newCol = col + dy[i];

    if (isValid(newRow, newCol) && !visitedCells[newRow][newCol]) {
      // Recurse with pathLength incremented
      if (dfsHelper(newRow, newCol, pathLength + 1)) {
        return true; // Return true if the goal is found in the recursive call
      }
    }
  }

  // Backtrack: Mark the cell as not visited
  visitedCells[row][col] = false;
  return false; // No valid path found
}

// Main DFS function
bool dfs(int row, int col) {
  return dfsHelper(row, col, 0); // Start DFS with pathLength = 0
   for (int i = 0; i <= pathSize; i++) {
      path[i] = finalPath[i];
    }

}
*/
// Struct for each node in the A* algorithm
struct Node {
  int row, col;         // Current cell coordinates
  int costSoFar;        // Cost to reach this cell from the start
  int estimatedCost;    // Total estimated cost (costSoFar + heuristic)
  int parentRow, parentCol; // Parent cell for path reconstruction

  // Custom comparator for the priority queue (min-heap)
  bool operator<(const Node &other) const {
    return estimatedCost > other.estimatedCost; // Lowest cost has highest priority
  }
};

struct Parent {
    int row;
    int col;
};

bool AStar(int startRow, int startCol) {
    std::priority_queue<Node> pq; // Min-heap based on estimatedCost
    bool visited[ROWS][COLS] = {false}; // To track visited cells
    Parent parent[ROWS][COLS]; // To store parent nodes for path reconstruction

    // Initialize parent array
    for (int i = 0; i < ROWS; i++) {
        for (int j = 0; j < COLS; j++) {
            parent[i][j] = {-1, -1};
        }
    }

    // Heuristic function to calculate Manhattan distance
    auto heuristic = [](int row, int col, int goalRow, int goalCol) -> int {
        return abs(row - goalRow) + abs(col - goalCol); // Manhattan distance
    };

    // Locate the goal ('G')
    int goalRow = -1, goalCol = -1;
    for (int i = 0; i < ROWS; i++) {
        for (int j = 0; j < COLS; j++) {
            if (maze[i][j] == "G") {
                goalRow = i;
                goalCol = j;
                break;
            }
        }
    }
    if (goalRow == -1 || goalCol == -1) {
        Serial.println("Goal 'G' not found in the maze!");
        return false; // Goal not found
    }

    // Initialize the start node
    Node startNode = {startRow, startCol, 0, heuristic(startRow, startCol, goalRow, goalCol), -1, -1};
    pq.push(startNode);

    while (!pq.empty()) {
        Node current = pq.top();
        pq.pop();

        int row = current.row;
        int col = current.col;

        // Check if the goal is reached
        if (row == goalRow && col == goalCol) {
            // Reconstruct the path by tracing parents
            int pathIndex = 0;
            while (row != -1 && col != -1) {
                path[pathIndex].first = row;
                path[pathIndex].second = col;
                Parent p = parent[row][col];
                row = p.row;
                col = p.col;
                pathIndex++;
            }
            pathSize = pathIndex;

            // Reverse the path to get it from start to goal
            for (int i = 0; i < pathSize / 2; i++) {
                std::swap(path[i], path[pathSize - i - 1]);
            }

            Serial.println("Path to the goal found:");
            for (int i = 0; i < pathSize; i++) {
                Serial.print(path[i].first);
                Serial.print(", ");
                Serial.println(path[i].second);
            }

            return true; // Path successfully found
        }

        // Skip if the cell is already visited
        if (visited[row][col]) continue;
        visited[row][col] = true;

        // Explore neighbors in all four directions
        for (int i = 0; i < 4; i++) {
            int newRow = row + dx[i];
            int newCol = col + dy[i];

            if (isValid(newRow, newCol) && !visited[newRow][newCol]) {
                int newCost = current.costSoFar + 1; // Cost of 1 to move to a neighbor
                int estimatedCost = newCost + heuristic(newRow, newCol, goalRow, goalCol);  // Corrected call

                // Add the neighbor to the priority queue
                Node neighbor = {newRow, newCol, newCost, estimatedCost, row, col};
                pq.push(neighbor);

                // Update parent for path reconstruction
                parent[newRow][newCol] = {row, col};
            }
        }
    }

    Serial.println("No path to the goal found.");
    return false; // No path to the goal found
}

// Call this function to find the shortest path using A*
bool dfs(int startRow, int startCol) {
  return AStar(startRow, startCol);
}



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
//                                                                                                      
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
  Pair older = path[0];

  for (int i = 1; i < pathSize; i++) {
    Pair current = path[i];

    // Moving down
    if (current.first == old.first + 1) {
      down++;
      if ((left + up + right) > 0) {
        left = up = right = 0;
        Serial.print("older: ");
        Serial.print(older.first);
        Serial.print(",");
         Serial.print(older.second);
        Serial.print("old: ");
        Serial.print(old.first);
        Serial.print(",");
         Serial.print(old.second);
        Serial.print("current");
        Serial.print(current.first);
        Serial.print(",");
         Serial.println(current.second);
        if (older.second + 1 == old.second) {
          instructionsList[instructionsSize].direction = "right";
          instructionsList[instructionsSize].count = right_count;
        } else  {
          instructionsList[instructionsSize].direction = "left";
          instructionsList[instructionsSize].count = left_count;
        }
        StartLocAfterTurn[instructionsSize].x = current.first;
        StartLocAfterTurn[instructionsSize].y = current.second;
        instructionsSize++;
        right_count = left_count = 0;
      }
    }
    // Moving up
    else if (current.first == old.first - 1) {
      up++;
      if ((left + right + down) > 0) {
        left = right = down = 0;
         Serial.print("older: ");
        Serial.print(older.first);
        Serial.print(",");
         Serial.print(older.second);
        Serial.print("old: ");
        Serial.print(old.first);
        Serial.print(",");
         Serial.print(old.second);
        Serial.print("current");
        Serial.print(current.first);
        Serial.print(",");
         Serial.println(current.second);

        if (older.second + 1 == old.second) {
          instructionsList[instructionsSize].direction = "left";
          instructionsList[instructionsSize].count = left_count;
        } else {
          instructionsList[instructionsSize].direction = "right";
          instructionsList[instructionsSize].count = right_count;
        }
        StartLocAfterTurn[instructionsSize].x = current.first;
        StartLocAfterTurn[instructionsSize].y = current.second;
        instructionsSize++;
        right_count = left_count = 0;
      }
    }
    // Moving right
    else if (current.second == old.second + 1) {
      right++;
      if ((left + up + down) > 0) {
        left = up = down = 0;
        Serial.print("right: ");

        //instructionsList[instructionsSize].direction = (right_count > left_count) ? "right" : "left";
        //instructionsList[instructionsSize].count = (right_count > left_count) ? right_count : left_count;
        Serial.print("older: ");
        Serial.print(older.first);
        Serial.print(",");
         Serial.print(older.second);
        Serial.print("old: ");
        Serial.print(old.first);
        Serial.print(",");
         Serial.print(old.second);
        Serial.print("current");
        Serial.print(current.first);
        Serial.print(",");
         Serial.println(current.second);

        if (older.first + 1 == old.first) {
          instructionsList[instructionsSize].direction = "left";
          instructionsList[instructionsSize].count = left_count;
        } else {
          instructionsList[instructionsSize].direction = "right";
          instructionsList[instructionsSize].count = right_count;
        }
        StartLocAfterTurn[instructionsSize].x = current.first;
        StartLocAfterTurn[instructionsSize].y = current.second;
        instructionsSize++;
        right_count = left_count = 0;
      }
    }
    // Moving left
    else {
      left++;
      if ((right + up + down) > 0) {
        right = up = down = 0;
                 Serial.print("left: ");

         Serial.print("older: ");
        Serial.print(older.first);
        Serial.print(",");
         Serial.print(older.second);
        Serial.print("old: ");
        Serial.print(old.first);
        Serial.print(",");
         Serial.print(old.second);
        Serial.print("current");
        Serial.print(current.first);
        Serial.print(",");
         Serial.println(current.second);

        if (older.first - 1 == old.first) {
          instructionsList[instructionsSize].direction = "left";
          instructionsList[instructionsSize].count = left_count;
        } else {
          instructionsList[instructionsSize].direction = "right";
          instructionsList[instructionsSize].count = right_count;
        }
        //instructionsList[instructionsSize].direction = (right_count >= left_count) ? "right" : "left";
        //instructionsList[instructionsSize].count = (right_count >= left_count) ? right_count : left_count;
        StartLocAfterTurn[instructionsSize].x = current.first;
        StartLocAfterTurn[instructionsSize].y = current.second;
        instructionsSize++;
        right_count = left_count = 0;
      }
    }

    // Count right and left turns
    Pair result = countRightLeft(old, current);
    right_count += result.first;
    left_count += result.second;

    Pair helper = old;
    old = current;
    older = helper;
  }

  // Handle any remaining instructions
  if ((right_count > 0 || left_count > 0) && instructionsSize == 0) {
    instructionsList[instructionsSize].direction = (right_count > left_count) ? "right" : "left";
    instructionsList[instructionsSize].count = (right_count > left_count) ? right_count : left_count;
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

      // Add additional directions if needed
      delay(500); // Delay between movements for safety
    }
  }
  Serial.println("All instructions executed.");
  move_pid();
}

void resetForNewDFS() {
  // Clear path
  pathSize = 0;
  for (int i = 0; i < MAX_PATH_LENGTH; i++) {
    path[i] = { -1, -1 };  // Reset path coordinates
  }
  // Clear instructions
  instructionsSize = 0;
  for (int i = 0; i < MAX_PATH_LENGTH; i++) {
    instructionsList[i].direction = "";
    instructionsList[i].count = 0;
    StartLocAfterTurn[i] = { -1, -1 };
  }

  // Clear visitedCells
  for (int i = 0; i < ROWS; i++) {
    for (int j = 0; j < COLS; j++) {
      visitedCells[i][j] = false;
    }
  }
  Serial.println("All DFS-related data has been reset.");
}

void setup() {
  // Initialize Serial communication
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for Serial to be ready
  }

  // Initialize LEDs

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
     Serial.print("<<<<<<<<< ");
      Serial.print(StartLocAfterTurn[i].x);
      Serial.print(" middle : ");
      Serial.println(StartLocAfterTurn[i].y);


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
   if (distance_Left <= 200 && distance_Right <= 200 && distance_Forward <= 180) {
      delay(500);  // להמתין כדי לוודא שהחיישנים התייצבו
      stop_moving();

      getMeasurments();  // קריאה נוספת של המדידות
      if (distance_Left <= 200 && distance_Right <= 200 && distance_Forward <= 180) {

    // לוגיקה אם התנאי עדיין נכון
 


        Serial.println("Blocked: No way to proceed!");
         
        if (vec_index < instructionsSize)
         {


           rotate_180();

          int xNewStart;
           int yNewStart;
         if(vec_index)
         {
             xNewStart = StartLocAfterTurn[vec_index-1].x;
             yNewStart = StartLocAfterTurn[vec_index-1].y;

         }
         else if(right_turns_counter==0 && left_turns_counter == 0)
         {
          Serial.println("No path to the goal found ");
            delay(10);
            stop_moving();
            for (int i = 1; i < 6; i++) {
         
               rotate_right(100);
          
               rotate_left(100);
            }

         }
         else {
           xNewStart = startX;
             yNewStart = startY;
             getMeasurments();
              while(distance_Forward>200)
           {
             move_forward();
             getMeasurments();

      
           }
            rotate_180();


         }
         
            
            


            maze[xNewStart][yNewStart] = "S";
            maze[startX][startY] = "F";
            maze[StartLocAfterTurn[vec_index].x][StartLocAfterTurn[vec_index].y] = "W";
            startX = xNewStart;
            startY = yNewStart;
            resetForNewDFS();
            pathSize=0;
            if (!dfs(startX, startY)) {
            Serial.println("No path to the goal found ");
            delay(10);
            stop_moving();
            for (int i = 1; i < 6; i++) {
         
               rotate_right(100);
          
               rotate_left(100);
              }
            return;
          }
            fillPathInstructions();
            Serial.println("New instructions generated:");
            for (int i = 0; i < instructionsSize; i++) {
              Serial.print(i);
              Serial.print(": ");
              Serial.print(instructionsList[i].direction);
              Serial.print(", ");
              Serial.println(instructionsList[i].count);
            }
            
             rotate_180();
             Serial.print("I'm out............. ");
             right_turns_counter =left_turns_counter=vec_index=0;
            
             


       
        return;  // Exit to allow the robot to execute the new path
        }
      }
     

     }
   

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
     getMeasurments();
    while(distance_Forward>100)
    {
        move_forward();
         getMeasurments();

      
    }
    for (int i = 1; i < 6; i++) {
         move_backward(200);
          rotate_right(100);
          rotate_left(100);

          move_forwardTime(150);
           rotate_right(100);
          rotate_left(100);
          
        }
    
    
      stop_moving();
      vec_index++;
  }
  delay(30);
} 
