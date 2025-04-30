// include/pins.h

#pragma once


// Motor B (Right Motor)
// #define RIGHT_MOTOR_IN1   8    // TB6612FNG BIN1
// #define RIGHT_MOTOR_IN2   7    // TB6612FNG BIN2
// #define RIGHT_MOTOR_PWM   6    // TB6612FNG PWMB


// #define RIGHT_ENCODER_A   4    // Right encoder channel A
// #define RIGHT_ENCODER_B   5    // Right encoder channel B (for direction)

// TB6612FNG / Motor 1
#define MOTOR1_IN1   8
#define MOTOR1_IN2   7
#define MOTOR1_PWM   6
#define MOTOR1_ENCA  4
#define MOTOR1_ENCB  5


// Motor A (Left Motor)
#define LEFT_MOTOR_IN1    3    // TB6612FNG AIN1
#define LEFT_MOTOR_IN2    10   // TB6612FNG AIN2
#define LEFT_MOTOR_PWM    11   // TB6612FNG PWMA
// Encoder Pins
#define LEFT_ENCODER_A    13   // Left encoder channel A
#define LEFT_ENCODER_B    14   // Left encoder channel B (for direction)
// TB6612FNG / Motor 2
#define MOTOR2_IN1   3
#define MOTOR2_IN2   10
#define MOTOR2_PWM   11
#define MOTOR2_ENCA  13
#define MOTOR2_ENCB  14

// If you have a standby pin or other pins, define them here:
// #define MOTOR_STBY   12

// LED for status indication
// #define LED_PIN           2
// Optional status LED
#define STATUS_LED   2





