# autoJetsonBot - Low Level Controller (LLC)

This repository contains the firmware for the ESP32-S3 motor controller used in the **autoJetsonBot** autonomous mobile robot. It controls wheel speed, reads the encoders and the IMU, and provides real-time telemetry to the ROS 2 environment.

> **Before flashing or driving, read [docs/FIRST_DRIVE.md](docs/FIRST_DRIVE.md).** This version hasn't run on the robot yet.

## 🚀 Features

-   **Wheel Speed Control**: A 100 Hz control task gives each wheel feed-forward PWM from the request, with a short kick to start it from rest and a closed-loop trim on top.
-   **Encoder Feedback**: Hardware quadrature decoding on all 4 edges of both channels, with a glitch filter.
-   **Runs Without Encoders**: The trim is only used while the encoder shows the wheel moving the requested way. If it doesn't, that wheel runs on feed-forward alone and the LED and `d` command say so.
-   **IMU**: MPU6050 at 50 Hz over I2C.
-   **RGB Status Indicators**: Visual feedback via onboard WS2812 LED.
-   **Human-Readable Serial Protocol**: Simple ASCII interface compatible with ROS 2 Python bridges.
-   **Failsafe Mode**: Automatic motor cutoff if communication with the Jetson/PC is lost (>500ms).

---

## 🛠️ Hardware Requirements

-   **Microcontroller**: ESP32-S3 (DevKitC-1 recommended).
-   **Motor Driver**: TB6612FNG Dual DC Motor Driver.
-   **Motors**: 2 × JGA25-370 DC 6V Geared Motors with Hall-Effect Encoders.
-   **Power**: 5.2V - 7.4V Battery (Separate from ESP32 power if possible).

---

## 📌 Pin Mapping (Standard Configuration)

### **Motor Driver (TB6612FNG)**
| Driver Pin | ESP32-S3 Pin | Note |
| :--- | :--- | :--- |
| **STBY** | **3.3V / 5V** | **Must be tied HIGH** to enable driver |
| **PWMA** | **GPIO 11** | Left Motor PWM |
| **AIN1** | **GPIO 3** | Left Motor Dir 1 |
| **AIN2** | **GPIO 10** | Left Motor Dir 2 |
| **PWMB** | **GPIO 6** | Right Motor PWM |
| **BIN1** | **GPIO 8** | Right Motor Dir 1 |
| **BIN2** | **GPIO 7** | Right Motor Dir 2 |

### **Encoders**
| Encoder Pin | ESP32-S3 Pin | Note |
| :--- | :--- | :--- |
| **Left Phase A** | **GPIO 18** | Hardware pulse counter |
| **Left Phase B** | **GPIO 14** | Hardware pulse counter |
| **Right Phase A**| **GPIO 4** | Hardware pulse counter |
| **Right Phase B**| **GPIO 5** | Hardware pulse counter |

### **IMU (MPU6050)**
| IMU Pin | ESP32-S3 Pin | Note |
| :--- | :--- | :--- |
| **SDA** | **GPIO 1** | I2C data |
| **SCL** | **GPIO 21** | I2C clock |
| **VCC** | **3.3V** | |

### **Internal / Visuals**
| Component | Pin | Logic |
| :--- | :--- | :--- |
| **WS2812 RGB** | **GPIO 48** | Status indicator |
| **Serial RX/TX**| **USB CDC** | Native S3 Serial over USB |

---

## 🛰️ Serial Communication Protocol

Baud Rate: **115200**

### **Downlink (ROS -> ESP32)**
Lines end in `\r` or `\n`. A malformed line is ignored.

-   **Motor Command**: `m <left_vel> <right_vel>` in m/s
    -   Example: `m 0.1 -0.1` (the bridge's forward: it negates the right wheel's command and count)
    -   A request under 0.01 m/s stops that wheel. With no `m` for 500 ms, both wheels stop.
-   **Force Test**: `f <pwm>` or `f <left_pwm> <right_pwm>`
    -   Example: `f 300` (raw PWM, no control, stops after 2 seconds unless repeated; `f 0` or any `m` ends it)
-   **Diagnostics**: `d` prints one line per wheel, the encoder pins' activity, and board state
-   **Tuning**: `g` prints the tuning values, and `g <name> <value>` changes one until the next reboot
    -   Names: `closed dead_l dead_r slope_l slope_r kick_l kick_r kick_ms kp ki trim cpr`

### **Uplink (ESP32 -> ROS)**
-   **Encoders**: `e <left_counts> <right_counts>` at 50 Hz
    -   Example: `e 14500 -14620`. A wheel's count goes up on a positive request for that wheel.
-   **IMU**: `i <ax> <ay> <az> <gx> <gy> <gz>` at 50 Hz, in m/s² and rad/s
    -   Example: `i 0.20 0.14 -9.94 -0.01 0.01 0.01`
    -   A failed read sends no line. After half a second of those, `i` lines stop until the IMU is found again.
-   **Boot**: `LLC_READY`, followed by ` ENCODER_ERROR` or ` IMU_ERROR` if one failed to start
-   **Fault**: `d fault <L|R> <reason>` when a wheel drops to feed-forward alone. Reasons:
    -   `no_counts`: driven, but the count didn't move the right way (dead wire, blocked wheel, or a kick too low to start it)
    -   `wrong_dir`: the count runs against the request
    -   `implausible`: faster than the wheel can turn (noise)
    -   `trim_sat`: the trim stayed at its limit adding power (bad feed-forward, or counts being lost)

---

## 🎛️ Setting the Gains

The values to set from a measurement are in one block in `include/config.h`. `tools/wheel_test.py` measures them and prints what to enter. [docs/FIRST_DRIVE.md](docs/FIRST_DRIVE.md) has the steps.

Run the control logic's tests on a PC, with no board: `pio test -e native`.

---

## 💡 Status LED Guide

-   **🟡 Yellow**: Booting.
-   **🟢 Dim Green**: Ready and Idle (Waiting for commands).
-   **🔵 Blue**: Receiving Movement Commands (Active).
-   **🟠 Orange**: A wheel is on feed-forward alone, until its next start shows the encoder working. `d` says why.
-   **🔴 Red**: The IMU wasn't found. Motors and encoders still work, and it retries every 2 s while idle.

---

## ⚡ Setup & Flashing

1.  Install **Visual Studio Code** and the **PlatformIO** extension.
2.  Clone this repository.
3.  Open the folder in PlatformIO.
4.  Connect your ESP32-S3 via the **USB** port.
5.  Click the **PlatformIO: Upload** button (Arrow icon at the bottom).

---

## ❓ Troubleshooting

### **Motors aren't moving?**
1.  Verify the **STBY** pin on the TB6612 is connected to **3.3V or 5V**.
2.  Ensure there is a **Common Ground** between the motor battery and the ESP32 GND pin.
3.  Check if the **VM** pin on the driver is receiving battery voltage (>5V).

### **Left encoder is 0?**
1.  Run `tools/wheel_test.py turns`. It reports which encoder pins change as you turn the wheel. Quadrature can't count with one of a wheel's two lines dead.
2.  Check the wiring on **Pin 18** and **Pin 14**. `d` also reports 13 and 17, the left pins of earlier versions, so a wire on either shows up.

### **Serial port not found on Mac?**
1.  Use `ls /dev/cu.usbmodem*` to find the port.
2.  Ensure no other Serial Monitor (like Arduino IDE) is open.
3.  Restart the `robotmac.sh` bridge if inside Docker.
