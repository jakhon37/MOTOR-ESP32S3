# autoJetsonBot - Low Level Controller (LLC)

This repository contains the firmware for the ESP32-S3 motor controller used in the **autoJetsonBot** autonomous mobile robot. It manages differential drive kinematics, PID speed control, encoder feedback, and provides real-time telemetry to the ROS 2 environment.

## 🚀 Features

-   **Deterministic Motor Control**: 50Hz control loop with PID regulation.
-   **Encoder Feedback**: Tracks dual-channel quadrature encoders for precise odometry.
-   **Aggressive Drive Logic**: Specialized kickstart PWM to overcome friction on low-voltage (5.2V - 6V) systems.
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
| **Left Phase A** | **GPIO 18** | Interrupt driven |
| **Left Phase B** | **GPIO 17** | Direction sensing |
| **Right Phase A**| **GPIO 4** | Interrupt driven |
| **Right Phase B**| **GPIO 5** | Direction sensing |

### **Internal / Visuals**
| Component | Pin | Logic |
| :--- | :--- | :--- |
| **WS2812 RGB** | **GPIO 48** | Status indicator |
| **Serial RX/TX**| **USB CDC** | Native S3 Serial over USB |

---

## 🛰️ Serial Communication Protocol

Baud Rate: **115200**

### **Downlink (ROS -> ESP32)**
-   **Motor Command**: `m <left_vel> <right_vel>`
    -   Example: `m 0.5 -0.5` (Spin clockwise)
-   **LED Control**: `l <state>`
    -   `l 1` (Bright Green), `l 0` (Dim Green)
-   **Force Test**: `f <pwm>`
    -   Example: `f 800` (Spin both wheels at 80% power for 2 seconds)

### **Uplink (ESP32 -> ROS)**
-   **Telemetry**: `e <left_ticks> <right_ticks>`
    -   Example: `e 14500 14620` (Sent at 20Hz)
-   **Heartbeat**: `STATUS: RGB_READY` (Sent on boot)

---

## 💡 Status LED Guide

-   **🟢 Dim Green**: Ready and Idle (Waiting for commands).
-   **🔵 Blue Pulse**: Receiving Movement Commands (Active).
-   **⚪ White**: Force/Diagnostic Mode.
-   **🔴 Red**: Booting / Initialization Error.

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
1.  Check the wiring on **Pin 18**.
2.  If using an older version of the firmware, ensure you aren't using Pin 13 (which conflicts with internal S3 functions).

### **Serial port not found on Mac?**
1.  Use `ls /dev/cu.usbmodem*` to find the port.
2.  Ensure no other Serial Monitor (like Arduino IDE) is open.
3.  Restart the `robotmac.sh` bridge if inside Docker.
