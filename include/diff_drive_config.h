// /home/jakhon37/Documents/PlatformIO/Projects/MOTOR-ESP32S3/include/diff_drive_config.h

#ifndef DIFF_DRIVE_CONFIG_H
#define DIFF_DRIVE_CONFIG_H

// -----------------------------
// Hardware Pin Definitions
// -----------------------------

// Left Motor (Motor A)
#define LEFT_MOTOR_IN1    3
#define LEFT_MOTOR_IN2    10
#define LEFT_MOTOR_PWM    11

// Right Motor (Motor B)
#define RIGHT_MOTOR_IN1   8
#define RIGHT_MOTOR_IN2   7
#define RIGHT_MOTOR_PWM   6

// Encoder Pin Definitions
#define LEFT_ENCODER_A    13
#define LEFT_ENCODER_B    14
#define RIGHT_ENCODER_A   4
#define RIGHT_ENCODER_B   5

// Status LED Pin
#define LED_PIN           2

// -----------------------------
// Robot Parameters
// -----------------------------
#define WHEEL_RADIUS       0.035    // Wheel radius in meters
#define WHEEL_SEPARATION   0.2      // Distance between wheels in meters
#define ENCODER_CPR        360      // Encoder counts per revolution
#define MAX_WHEEL_ANGULAR  14.0     // Max wheel angular velocity in rad/s

// -----------------------------
// Communication Parameters
// -----------------------------
#define SERIAL_BAUD_RATE   115200   // Serial baud rate
#define WATCHDOG_TIMEOUT   10000     // Watchdog timeout in milliseconds

// -----------------------------
// Constants
// -----------------------------
#define ENCODER_CPR       360     // Encoder counts per revolution
#define RPM_TIMER_PERIOD  1000    // RPM calculation period in milliseconds


// -----------------------------
// PWM Parameters (LEDC)
// -----------------------------
#define PWM_FREQ          1000      // 1 kHz
#define PWM_RESOLUTION    8         // 8-bit resolution

// -----------------------------
// Protocol Constants
// -----------------------------
#define START_BYTE         0xFF     // Start byte for protocol packets
#define PID_VEL            0x01     // Velocity command packet ID
#define PID_ENCODER_COUNT  0x02     // Encoder count request packet ID
#define PID_ENCODER_DELTA  0x03     // Encoder delta request packet ID

// -----------------------------
// WiFi Constants (if needed)
// -----------------------------
#define WIFI_SSID       "U+NetEFDA"
#define WIFI_PASSWORD   "5D2A1A9EM#"
#define AGENT_IP        "192.168.219.107"
#define AGENT_PORT      8888

#endif // DIFF_DRIVE_CONFIG_H