#include <Arduino.h>
#include "diff_drive_config.h"
#include "MotorControl.h"

// Robot parameters
float encoder_resolution = ENCODER_CPR;
float wheel_radius = WHEEL_RADIUS;  // meters
float wheel_separation = WHEEL_SEPARATION; // meters
float max_angular_velocity = MAX_WHEEL_ANGULAR; // rad/s

// Timing variables
uint32_t last_cmd_time = 0;
uint32_t last_encoder_read = 0;

void set_velocities(float left_vel, float right_vel);
float constrain_velocity(float velocity);

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial && millis() < 3000); // Wait up to 3 seconds for serial
  
  MotorControl::begin();
  
  pinMode(LED_PIN, OUTPUT);
  
  Serial.println("DiffDrive Arduino Controller Ready");
  digitalWrite(LED_PIN, HIGH);
}

void loop() {
  // Process serial commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\r');
    cmd.trim(); // Remove leading/trailing whitespace
    
    // Log raw input
    Serial.print("Raw input: '");
    Serial.print(cmd);
    Serial.println("'");
    
    // Ignore empty commands
    if (cmd.length() == 0) {
      Serial.println("Ignoring empty command");
      return;
    }
    
    Serial.print("Processing command: ");
    Serial.println(cmd);
    
    if (cmd == "e") {
      // Handle encoder request
      int32_t left_count = MotorControl::getLeftEncoderCount();
      int32_t right_count = MotorControl::getRightEncoderCount();
      float dt = (last_encoder_read > 0) ? (millis() - last_encoder_read) / 1000.0 : 0.0;
      last_encoder_read = millis();
      
      Serial.print("Sending encoder counts: left=");
      Serial.print(left_count);
      Serial.print(", right=");
      Serial.println(right_count);
      Serial.print(left_count);
      Serial.print(" ");
      Serial.println(right_count);
      
      last_cmd_time = millis();
    }
    else if (cmd.startsWith("m ")) {
      // Handle motor velocity command
      int space1 = cmd.indexOf(' ', 2);
      if (space1 != -1) {
        String val1_str = cmd.substring(2, space1);
        String val2_str = cmd.substring(space1 + 1);
        
        float left_vel = val1_str.toFloat() / 100.0 * max_angular_velocity; // Convert to rad/s
        float right_vel = val2_str.toFloat() / 100.0 * max_angular_velocity;
        
        Serial.print("Received velocities: left=");
        Serial.print(left_vel);
        Serial.print(" rad/s, right=");
        Serial.println(right_vel);
        
        set_velocities(left_vel, right_vel);
        last_cmd_time = millis();
      } else {
        Serial.println("Error: Invalid motor command format");
      }
    }
    else {
      Serial.println("Error: Unknown command");
    }
  }
  
  // Watchdog: stop motors if no commands received
  if (millis() - last_cmd_time > WATCHDOG_TIMEOUT) {
    Serial.println("Watchdog triggered: Stopping motors");
    set_velocities(0, 0);
  }
  
  // Blink LED
  if ((millis() / 500) % 2 == 0) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }
  
  delay(5);
}

void set_velocities(float left_vel, float right_vel) {
  float left_speed = constrain_velocity(left_vel);
  float right_speed = constrain_velocity(right_vel);
  
  Serial.print("Setting motor speeds: left=");
  Serial.print(left_speed);
  Serial.print(", right=");
  Serial.println(right_speed);
  
  MotorControl::setMotorSpeeds(left_speed, right_speed);
}

float constrain_velocity(float velocity) {
  float constrained = constrain(velocity / max_angular_velocity, -1.0, 1.0);
  return constrained;
}