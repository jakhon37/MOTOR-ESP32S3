// MOTOR-ESP32S3/lib/MotorControl/MotorControl.h
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include "config.h"

class MotorControl {
public:
  static void begin();
  static void setMotorSpeeds(float leftPWM, float rightPWM);
  
  // Feedback
  static int32_t getLeftTicks();
  static int32_t getRightTicks();
  static float getLeftVel(float dt);
  static float getRightVel(float dt);

  // Interrupts
  static void IRAM_ATTR leftEncoderISR();
  static void IRAM_ATTR rightEncoderISR();

private:
  static void setMotorSpeed(uint8_t in1, uint8_t in2, uint8_t channel, float pwm);
  
  static volatile int32_t leftEncoderCount;
  static volatile int32_t rightEncoderCount;
  static int32_t leftLastCount;
  static int32_t rightLastCount;
};

#endif
