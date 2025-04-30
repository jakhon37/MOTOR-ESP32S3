// MOTOR-ESP32S3/src/main.cpp

#include <Arduino.h>
#include "config.h"
#include "MotorControl.h"


void parseSerialCommand(String command);  // Declare the function here

void setup() {
  Serial.begin(115200);  // Match baud rate with Jetson Nano
  delay(1000);
  Serial.println("Starting motor control over serial...");

  // Set status LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // Initialize Motor Control module
  MotorControl::begin();

  Serial.println("Ready!");
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    parseSerialCommand(command);
  }
  delay(10);  // Small delay to prevent overloading
}

void parseSerialCommand(String command) {
  // Example command: "LEFT:0.5,RIGHT:0.7"
  int leftIndex = command.indexOf("LEFT:");
  int rightIndex = command.indexOf(",RIGHT:");
  
  if (leftIndex != -1 && rightIndex != -1) {
    String leftStr = command.substring(leftIndex + 5, rightIndex);
    String rightStr = command.substring(rightIndex + 7);
    
    float leftSpeed = leftStr.toFloat();
    float rightSpeed = rightStr.toFloat();
    
    // Constrain speeds to [-1.0, 1.0] as in your original code
    leftSpeed = constrain(leftSpeed, -1.0, 1.0);
    rightSpeed = constrain(rightSpeed, -1.0, 1.0);
    
    // Set motor speeds
    MotorControl::setMotorSpeeds(leftSpeed, rightSpeed);
    
    // Optional: Send feedback (e.g., RPM) back to Jetson
    Serial.print("RPM_LEFT:");
    Serial.print(MotorControl::getLeftRPM());
    Serial.print(",RPM_RIGHT:");
    Serial.println(MotorControl::getRightRPM());
  }
}