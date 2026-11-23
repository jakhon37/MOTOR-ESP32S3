// MOTOR-ESP32S3/lib/MotorControl/MotorControl.cpp
#include "MotorControl.h"

#include <driver/pcnt.h>
#include "SpeedControl.h"

// The counter resets to 0 at these limits. updateEncoders() reads it far more often than that.
static const int16_t COUNTER_LIMIT = 30000;

static const uint8_t WATCHED_PINS[MotorControl::WATCHED_PIN_COUNT] = {
  LEFT_ENCODER_A, LEFT_ENCODER_B, RIGHT_ENCODER_A, RIGHT_ENCODER_B, OLD_LEFT_ENCODER_A, OLD_LEFT_ENCODER_B};
static volatile uint8_t watchedLevels[MotorControl::WATCHED_PIN_COUNT];
static volatile uint32_t watchedTransitions[MotorControl::WATCHED_PIN_COUNT];

static int16_t leftLastRaw = 0;
static int16_t rightLastRaw = 0;

volatile int32_t MotorControl::leftEncoderCount = 0;
volatile int32_t MotorControl::rightEncoderCount = 0;

// Hardware quadrature decoding on all 4 edges: each pin is the other's direction control, so the
// count only moves through the proper A/B sequence. Noise on one pin steps it back and forth
// instead of adding up, and a dead pin stops it counting. The CPU sees no interrupts.
static bool beginQuadrature(pcnt_unit_t unit, int pinA, int pinB) {
  pcnt_config_t channelA = {};
  channelA.pulse_gpio_num = pinA;
  channelA.ctrl_gpio_num = pinB;
  channelA.lctrl_mode = PCNT_MODE_REVERSE;
  channelA.hctrl_mode = PCNT_MODE_KEEP;
  channelA.pos_mode = PCNT_COUNT_DEC;  // A rises while B is high: -1, while B is low: +1
  channelA.neg_mode = PCNT_COUNT_INC;
  channelA.counter_h_lim = COUNTER_LIMIT;
  channelA.counter_l_lim = -COUNTER_LIMIT;
  channelA.unit = unit;
  channelA.channel = PCNT_CHANNEL_0;
  esp_err_t err = pcnt_unit_config(&channelA);

  pcnt_config_t channelB = channelA;
  channelB.pulse_gpio_num = pinB;
  channelB.ctrl_gpio_num = pinA;
  channelB.pos_mode = PCNT_COUNT_INC;  // B rises while A is high: +1, while A is low: -1
  channelB.neg_mode = PCNT_COUNT_DEC;
  channelB.channel = PCNT_CHANNEL_1;
  err |= pcnt_unit_config(&channelB);

  err |= pcnt_set_filter_value(unit, ENCODER_FILTER_CYCLES);
  err |= pcnt_filter_enable(unit);
  // pcnt_unit_config turns these off. They make the counter reset at the limits.
  err |= pcnt_event_enable(unit, PCNT_EVT_H_LIM);
  err |= pcnt_event_enable(unit, PCNT_EVT_L_LIM);
  err |= pcnt_counter_pause(unit);
  err |= pcnt_counter_clear(unit);
  err |= pcnt_counter_resume(unit);
  return err == ESP_OK;
}

bool MotorControl::begin() {
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);

  // Setup LEDC PWM
  ledcSetup(0, PWM_FREQ, PWM_RES);
  ledcAttachPin(LEFT_MOTOR_PWM, 0);
  ledcSetup(1, PWM_FREQ, PWM_RES);
  ledcAttachPin(RIGHT_MOTOR_PWM, 1);

  // Encoders
  for (uint8_t i = 0; i < WATCHED_PIN_COUNT; i++) {
    pinMode(WATCHED_PINS[i], INPUT_PULLUP);
    watchedLevels[i] = digitalRead(WATCHED_PINS[i]);
  }
  const bool leftOk = beginQuadrature(PCNT_UNIT_0, LEFT_ENCODER_A, LEFT_ENCODER_B);
  const bool rightOk = beginQuadrature(PCNT_UNIT_1, RIGHT_ENCODER_A, RIGHT_ENCODER_B);
  return leftOk && rightOk;
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

void MotorControl::updateEncoders() {
  int16_t raw;
  pcnt_get_counter_value(PCNT_UNIT_0, &raw);
  leftEncoderCount = leftEncoderCount + LEFT_ENCODER_SIGN * counterDelta(raw, leftLastRaw, COUNTER_LIMIT);
  leftLastRaw = raw;
  pcnt_get_counter_value(PCNT_UNIT_1, &raw);
  rightEncoderCount = rightEncoderCount + RIGHT_ENCODER_SIGN * counterDelta(raw, rightLastRaw, COUNTER_LIMIT);
  rightLastRaw = raw;
}

int32_t MotorControl::getLeftTicks() { return leftEncoderCount; }
int32_t MotorControl::getRightTicks() { return rightEncoderCount; }

void MotorControl::sampleEncoderPins() {
  for (uint8_t i = 0; i < WATCHED_PIN_COUNT; i++) {
    const uint8_t level = digitalRead(WATCHED_PINS[i]);
    if (level != watchedLevels[i]) {
      watchedLevels[i] = level;
      watchedTransitions[i] = watchedTransitions[i] + 1;
    }
  }
}

uint8_t MotorControl::watchedPin(uint8_t i) { return WATCHED_PINS[i]; }
int MotorControl::watchedPinLevel(uint8_t i) { return watchedLevels[i]; }
uint32_t MotorControl::watchedPinTransitions(uint8_t i) { return watchedTransitions[i]; }
