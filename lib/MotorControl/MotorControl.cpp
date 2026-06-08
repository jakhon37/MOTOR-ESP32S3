// MOTOR-ESP32S3/lib/MotorControl/MotorControl.cpp
#include "MotorControl.h"

volatile int32_t MotorControl::leftEncoderCount = 0;
volatile int32_t MotorControl::rightEncoderCount = 0;
int32_t MotorControl::leftLastCount = 0;
int32_t MotorControl::rightLastCount = 0;

void MotorControl::begin() {
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);

  // Setup LEDC PWM
  ledcSetup(0, PWM_FREQ, PWM_RES);
  ledcAttachPin(LEFT_MOTOR_PWM, 0);
  ledcSetup(1, PWM_FREQ, PWM_RES);
  ledcAttachPin(RIGHT_MOTOR_PWM, 1);

  // Encoders
  pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightEncoderISR, RISING);
}

void MotorControl::setMotorSpeeds(float leftPWM, float rightPWM) {
  setMotorSpeed(LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, 0, leftPWM);
  setMotorSpeed(RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2, 1, rightPWM);
}

void MotorControl::setMotorSpeed(uint8_t in1, uint8_t in2, uint8_t channel, float pwm) {
  uint32_t duty = (uint32_t)(constrain(fabs(pwm), 0.0, (float)PWM_MAX));
  
  if (pwm > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  } else if (pwm < 0) {
    digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);  digitalWrite(in2, LOW);
  }
  ledcWrite(channel, duty);
}

void IRAM_ATTR MotorControl::leftEncoderISR() {
  if (digitalRead(LEFT_ENCODER_B)) leftEncoderCount--;
  else leftEncoderCount++;
}

void IRAM_ATTR MotorControl::rightEncoderISR() {
  if (digitalRead(RIGHT_ENCODER_B)) rightEncoderCount++; // R is mirrored usually
  else rightEncoderCount--;
}

int32_t MotorControl::getLeftTicks() { return leftEncoderCount; }
int32_t MotorControl::getRightTicks() { return rightEncoderCount; }

float MotorControl::getLeftVel(float dt) {
    int32_t current = leftEncoderCount;
    float d_ticks = current - leftLastCount;
    leftLastCount = current;
    // meters = (ticks / cpr) * (2 * PI * radius)
    return (d_ticks / (float)ENCODER_CPR) * (2.0 * PI * WHEEL_RADIUS) / dt;
}

float MotorControl::getRightVel(float dt) {
    int32_t current = rightEncoderCount;
    float d_ticks = current - rightLastCount;
    rightLastCount = current;
    return (d_ticks / (float)ENCODER_CPR) * (2.0 * PI * WHEEL_RADIUS) / dt;
}
