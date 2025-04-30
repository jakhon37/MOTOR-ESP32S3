// MOTOR-ESP32S3/src/main.cpp

#include <Arduino.h>
#include "config.h"
#include "MotorControl.h"

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting motor control over serial...");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  MotorControl::begin();

  Serial.println("Ready!");
}

void loop() {
  // Read velocity commands (two floats: left_vel, right_vel in rad/s)
  if (Serial.available() >= 8) {
    uint8_t buffer[8];
    Serial.readBytes(buffer, 8);
    float left_vel, right_vel;
    memcpy(&left_vel, buffer, 4);
    memcpy(&right_vel, buffer + 4, 4);

    // Convert rad/s to normalized [-1.0, 1.0] (assume max 14.286 rad/s)
    float max_angular = 0.5 / 0.035; // max_wheel_linear / wheel_radius
    float left_speed = constrain(left_vel / max_angular, -1.0, 1.0);
    float right_speed = constrain(right_vel / max_angular, -1.0, 1.0);

    MotorControl::setMotorSpeeds(left_speed, right_speed);
  }

  // Send encoder counts (two int32_t: left_count, right_count)
  int32_t left_count = MotorControl::getLeftEncoderCount();
  int32_t right_count = MotorControl::getRightEncoderCount();
  Serial.write((uint8_t*)&left_count, 4);
  Serial.write((uint8_t*)&right_count, 4);

  // Send RPM feedback for debugging (optional)
  Serial.print("RPM_LEFT:");
  Serial.print(MotorControl::getLeftRPM());
  Serial.print(",RPM_RIGHT:");
  Serial.println(MotorControl::getRightRPM());

  delay(33); // ~30 Hz
}