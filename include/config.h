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

// The left encoder was on these two pins before v1.1. Nothing uses them now. The `d` command
// reports them so a wire that was never moved shows up.
#define OLD_LEFT_ENCODER_A 13
#define OLD_LEFT_ENCODER_B 14

// A positive `m` request on a wheel must make its count go up. Raw counts go up when A rises
// while B is low. These signs match the old interrupts, but no motor has run on them yet.
// Flip one if the wheel test shows wrong_dir: the bridge expects this rule on both wheels.
#define LEFT_ENCODER_SIGN   1
#define RIGHT_ENCODER_SIGN  (-1)

// Edges shorter than this are ignored, in 80 MHz clock cycles (hardware limit 1023).
// 800 is 10 us, against about 140 us between real edges at full speed.
#define ENCODER_FILTER_CYCLES 800

// -----------------------------
// IMU (MPU6050) Pin Definitions
// -----------------------------
#define IMU_SDA           1
#define IMU_SCL           21
#define IMU_LPF_ALPHA     0.8f    // accelerometer filter (0.8 = sharp, 0.2 = smooth)

// -----------------------------
// Robot Physical Specs
// -----------------------------
#define WHEEL_RADIUS      0.034f  // meters
#define WHEEL_SEPARATION  0.212   // meters (Match robot.sh / launch)

// -----------------------------
// Control Parameters
// -----------------------------
#define CONTROL_PERIOD_MS 10      // motor control tick
#define CMD_TIMEOUT       500     // milliseconds to e-stop if no command
#define STOP_SPEED_MPS    0.01f   // a request below this means stop
#define FORCE_TEST_MS     2000    // the `f` test stops after this unless repeated

// =============================================================================================
// SET THESE FROM A MEASUREMENT. tools/wheel_test.py prints them, see docs/FIRST_DRIVE.md.
// The values below are estimates: none of them has been measured on the robot.
// All of them can also be tried without flashing, with the `g` command.
// =============================================================================================

// Counts per wheel turn, counting all 4 edges. Estimate: 4 x the 344 to 387 per turn counted on
// one edge in the 2026-11-06 hand test. The bridge's encoder_cpr must hold the same number.
#define ENCODER_CPR       1460

// PWM (0-1023) for a requested speed = DEADBAND + SLOPE * speed.
#define FF_DEADBAND_LEFT  200     // PWM at which the turning wheel stops
#define FF_DEADBAND_RIGHT 200
#define FF_SLOPE_LEFT     1100    // extra PWM per m/s
#define FF_SLOPE_RIGHT    1100

// Short push when a wheel starts from rest, because starting takes more PWM than keeping going.
// It must start the wheel on the floor. A wheel that a kick doesn't move stays stopped until its
// next start. The old firmware's 500 is the only level known to work, so this is that plus a margin.
#define KICK_PWM_LEFT     550
#define KICK_PWM_RIGHT    550
#define KICK_MS           40

// Closed loop on top of feed-forward: trim = K_P * error + K_I * integral(error), error in m/s.
#define CLOSED_LOOP       1       // 0 = feed-forward only
#define K_P               300.0f
#define K_I               2000.0f
#define TRIM_MAX_PWM      250.0f  // the most the closed loop may add or remove

// =============================================================================================

// Limits on when the encoders are believed. If one fails, that wheel runs on feed-forward alone
// until it is next stopped, and `d` and the LED say so.
#define VELOCITY_WINDOW_TICKS  10     // speed is measured over this many ticks
#define ENC_PROOF_COUNTS       6      // counts the right way before the encoder is believed
#define ENC_PROOF_TIMEOUT_MS   300    // no_counts: PWM applied, but no such counts in this time
#define ENC_WRONG_DIR_COUNTS   4      // wrong_dir: counts the wrong way in one window...
#define ENC_WRONG_DIR_SETTLE_MS 300   // ...this long after a start from rest...
#define ENC_REVERSAL_SETTLE_MS 800    // ...or after a start while still rolling the other way
#define ENC_STILL_COUNTS       2      // fewer counts than this in one window means not moving
#define ENC_STILL_MS           300    // not moving for this long: kick again and re-check
#define ENC_MAX_MPS            1.5f   // implausible: faster than these wheels can turn
#define TRIM_SATURATED_MS      1500   // trim_sat: trim at its limit, adding power, for this long
#define KICK_MIN_INTERVAL_MS   250    // a request flickering around the stop limit kicks no faster

// -----------------------------
// PWM Parameters (using LEDC)
// -----------------------------
#define PWM_FREQ          20000   // 20 kHz (Silent mode)
#define PWM_RES           10      // 10-bit resolution (0-1023)
#define PWM_MAX           1023

#define RGB_PIN           48  // WS2812 RGB LED on ESP32-S3
#define LED_PIN           2   // Internal blue LED

#endif
