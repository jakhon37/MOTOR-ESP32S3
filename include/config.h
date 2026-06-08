// MOTOR-ESP32S3/include/config.h
#ifndef CONFIG_H
#define CONFIG_H

// -----------------------------
// Motor Driver (TB6612FNG) Pin Definitions
// -----------------------------
#define LEFT_MOTOR_IN1    3
#define LEFT_MOTOR_IN2    10
#define LEFT_MOTOR_PWM    11

#define RIGHT_MOTOR_IN1   8
#define RIGHT_MOTOR_IN2   7
#define RIGHT_MOTOR_PWM   6

// -----------------------------
// Encoder Pin Definitions
// -----------------------------
#define LEFT_ENCODER_A    18
#define LEFT_ENCODER_B    17
#define RIGHT_ENCODER_A   4
#define RIGHT_ENCODER_B   5

// -----------------------------
// Robot Physical Specs
// -----------------------------
#define WHEEL_RADIUS      0.034   // meters
#define WHEEL_SEPARATION  0.212    // meters (Match robot.sh / launch)
#define ENCODER_CPR       3436    // counts per revolution (Standard Jetson Bot)

// -----------------------------
// Control Parameters
// -----------------------------
#define LOOP_FREQ         50      // Hz
#define LOOP_DT           0.02    // seconds (1/50)
#define CMD_TIMEOUT       500     // milliseconds to e-stop if no command

// PID Constants (Starting points, tune on blocks)
#define K_P               500.0
#define K_I               100.0
#define K_D               10.0

// -----------------------------
// PWM Parameters (using LEDC)
// -----------------------------
#define PWM_FREQ          20000   // 20 kHz (Silent mode)
#define PWM_RES           10      // 10-bit resolution (0-1023)
#define PWM_MAX           1023

#define RGB_PIN           48  // WS2812 RGB LED on ESP32-S3
#define LED_PIN           2   // Internal blue LED

#endif
