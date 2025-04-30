// MOTOR-ESP32S3/lib/MotorControl/MotorControl.h

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include "config.h"

class MotorControl {
public:
  static void begin();
  static void setMotorSpeeds(float leftSpeed, float rightSpeed);
  static float getLeftRPM();
  static float getRightRPM();
  static int32_t getLeftEncoderCount();
  static int32_t getRightEncoderCount();
  static void IRAM_ATTR leftEncoderISR();
  static void IRAM_ATTR rightEncoderISR();

private:
  static void setMotorSpeed(uint8_t in1, uint8_t in2, uint8_t pwmPin, float speed);
  
  static volatile int32_t leftEncoderCount;
  static volatile int32_t rightEncoderCount;
  static int32_t leftLastCount;
  static int32_t rightLastCount;
  static uint32_t lastRPMTime;
};

#endif // MOTOR_CONTROL_H