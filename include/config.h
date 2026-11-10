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
#define LEFT_ENCODER_B    14  // the robot's green wire: the wiring diagram has it here, not on v1.1's 17
#define RIGHT_ENCODER_A   4
#define RIGHT_ENCODER_B   5

// Spare left pins: 13 was the left A before v1.1, and 17 is the B pin v1.1 planned. Nothing uses
// them now. The `d` command reports them so a wire on either shows up.
#define OLD_LEFT_ENCODER_A 13
#define OLD_LEFT_ENCODER_B 17

// A positive `m` request on a wheel must make its count go up. Raw counts go up when A rises
// while B is low. Set from the first drive on 2026-11-09: with the robot's wiring the left count
// ran backwards on a forward request, and the right one was right.
// Flip one if the wheel test shows wrong_dir: the bridge expects this rule on both wheels.
#define LEFT_ENCODER_SIGN   (-1)
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

// Gyro offsets at rest, in rad/s and the IMU's own frame (it's mounted upside down), subtracted
// before the `i` line is sent. Measured on 2026-11-10 over 60 s on the floor, 3,000 samples, with
// the same means in both halves. Uncorrected, Z alone turned the heading 0.7 deg/s. They're where
// the firmware starts: the learning below follows them as they drift with temperature.
#define GYRO_BIAS_X       -0.0092f
#define GYRO_BIAS_Y       -0.0569f
#define GYRO_BIAS_Z       -0.0122f

// On the robot z drifted by 0.003 rad/s within 20 minutes of the offsets being measured, and by
// 0.0017 after 8 hours powered, enough to turn the EKF's heading 4.8 deg a minute at rest. So the
// offsets are measured again whenever the robot stands still: no PWM on either motor, and neither
// wheel count moving by GYRO_LEARN_MAX_COUNTS, for GYRO_LEARN_SETTLE_MS. Each window of
// GYRO_LEARN_WINDOW readings (2 s at 50 Hz) whose standard deviation stays under
// GYRO_LEARN_MAX_SD on every axis, and whose mean is within GYRO_LEARN_MAX_SHIFT of the offsets
// above, moves the offsets GYRO_LEARN_RATE of the way to its mean. At rest on the floor the
// readings' standard deviation was 0.003 to 0.006 rad/s. `d` prints the offsets in use.
#define GYRO_LEARN_SETTLE_MS   1000
#define GYRO_LEARN_WINDOW      100
#define GYRO_LEARN_MAX_COUNTS  2
#define GYRO_LEARN_MAX_SD      0.015f  // rad/s
#define GYRO_LEARN_MAX_SHIFT   0.02f   // rad/s, about 7 times the largest drift seen
#define GYRO_LEARN_RATE        0.25f

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
// ENCODER_CPR was measured on 2026-11-10 by hand, and the feed-forward values on the floor the
// same day. The kick has started both wheels on the floor; the closed-loop gains are estimates.
// All of them can also be tried without flashing, with the `g` command.
// =============================================================================================

// Counts per wheel turn, counting all 4 edges. Measured in the 2026-11-10 hand test: 1,497, 1,496,
// 1,500 and 1,496 for the two wheels both ways (4 x 11 pulses x a 34:1 gearbox).
// The bridge's encoder_cpr must hold the same number.
#define ENCODER_CPR       1497

// PWM (0-1023) for a requested speed = DEADBAND + SLOPE * speed. Set for the floor on 2026-11-10
// from wheel_test.py hold at 0.03 and 0.15 m/s (docs/FIRST_DRIVE.md step 10): with these, the
// trims there stayed within 11. In the air the wheels need less: deadband 51 and 42, slope 1,743
// and 1,732, measured with wheel_test.py ff.
#define FF_DEADBAND_LEFT  58      // PWM at which the turning wheel stops
#define FF_DEADBAND_RIGHT 88
#define FF_SLOPE_LEFT     1893    // extra PWM per m/s
#define FF_SLOPE_RIGHT    1740

// Short push when a wheel starts from rest, because starting takes more PWM than keeping going.
// It must start the wheel on the floor. A wheel that a kick doesn't move stays stopped until its
// next start. This is the old firmware's 500 plus a margin, and on 2026-11-10 it started both
// wheels in five floor runs.
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
