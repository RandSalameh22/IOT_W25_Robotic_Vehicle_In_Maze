#ifndef COMM_H
#define COMM_H

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// Replace with your Google Apps Script web app URL
const char* scriptUrl = "https://script.google.com/macros/s/AKfycbwPG7saY-uFMCJCVG5xh1ruyzOdJOxz4VizYz-x2jLY5aiznPy9YLUdElHNham4P9pV/exec";

const int ROWS = 10; // Number of rows in your maze
const int COLS = 10; // Number of columns in your maze
String maze[ROWS][COLS]; // Maze array to hold the data

void commSetup() {
  pinMode(LEFT_LED_PIN, OUTPUT);
  pinMode(RIGHT_LED_PIN, OUTPUT);
  digitalWrite(LEFT_LED_PIN, HIGH);
  digitalWrite(RIGHT_LED_PIN, HIGH);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");
  digitalWrite(LEFT_LED_PIN, LOW);
  digitalWrite(RIGHT_LED_PIN, LOW);
}

void commReadData() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    http.begin(scriptUrl);

    int httpCode = http.GET();

    if (httpCode == 200) {
      String payload = http.getString();
      Serial.println("HTTP Code: 200");
      Serial.print("Payload: ");
      Serial.println(payload);

      // Deserialize the JSON payload
      DynamicJsonDocument doc(2048);
      DeserializationError error = deserializeJson(doc, payload);

      if (!error) {
        if (doc.is<JsonArray>()) {
          for (size_t i = 0; i < doc.size() && i < ROWS; i++) {
            JsonArray row = doc[i].as<JsonArray>();
            if (row) {
              for (size_t j = 0; j < row.size() && j < COLS; j++) {
                if (row[j].is<const char*>()) {
                  maze[i][j] = row[j].as<String>();
                  Serial.print(maze[i][j]);
                  Serial.print(" ");
                } else {
                  maze[i][j] = "";
                  Serial.print("Invalid cell at [");
                  Serial.print(i);
                  Serial.print("][");
                  Serial.print(j);
                  Serial.println("]");
                }
              }
              Serial.println();
            } else {
              Serial.print("Invalid row at index ");
              Serial.println(i);
            }
          }
          Serial.println("Maze matrix filled successfully.");
        } else {
          Serial.println("Invalid JSON format: Root is not an array.");
        }
      } else {
        Serial.print("deserializeJson() failed: ");
        Serial.println(error.c_str());
      }
    } else {
      Serial.print("HTTP request failed, code: ");
      Serial.println(httpCode);
    }

    http.end();
  } else {
    Serial.println("WiFi not connected.");
  }

  delay(2000); // Add a delay between requests
}

#endif // COMM_H
