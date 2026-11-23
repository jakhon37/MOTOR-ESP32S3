// MOTOR-ESP32S3/lib/MotorControl/MotorControl.h
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include "config.h"

class MotorControl {
public:
  static bool begin();  // false if the encoder counters could not be set up
  static void setMotorSpeeds(float leftPWM, float rightPWM);

  // Feedback. updateEncoders() must run at least every 500 ms; the control tick calls it.
  static void updateEncoders();
  static int32_t getLeftTicks();
  static int32_t getRightTicks();

  // Encoder pin activity, for the `d` command. Sampled at the control tick, so the transition
  // counts are not edge counts: they only show whether a pin is alive.
  static void sampleEncoderPins();
  static const uint8_t WATCHED_PIN_COUNT = 6;
  static uint8_t watchedPin(uint8_t i);
  static int watchedPinLevel(uint8_t i);
  static uint32_t watchedPinTransitions(uint8_t i);

private:
  static void setMotorSpeed(uint8_t in1, uint8_t in2, uint8_t channel, float pwm);

  static volatile int32_t leftEncoderCount;
  static volatile int32_t rightEncoderCount;
};

#endif
