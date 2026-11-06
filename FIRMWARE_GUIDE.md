# autoJetsonBot — ESP32-S3 Motor Control Guide

This repository contains the **Low-Level Controller (LLC)** firmware for the autoJetsonBot. It is responsible for real-time motor control, encoder feedback, and maintaining precise velocity via PID loops.

---

## 🚀 Quick Start (PlatformIO)

1.  **Install PlatformIO:** Use the VS Code extension.
2.  **Open Project:** Open this folder (`MOTOR-ESP32S3`) in VS Code.
3.  **Build:** Press the checkmark icon in the status bar.
4.  **Upload:** Connect your ESP32-S3 via USB and press the arrow icon.

---

## ⚙️ Configuration (`include/config.h`)

The `config.h` file is the "brain" of the firmware. Always verify these values before deployment.

### 1. Physical Specs (The "Digital Twin")
These must match the Jetson Nano URDF and YAML configurations perfectly.
*   `WHEEL_RADIUS`: Currently **0.034m**.
*   `WHEEL_SEPARATION`: Currently **0.212m**.
*   `ENCODER_CPR`: counts per wheel turn on all 4 edges. Measure it: see [docs/FIRST_DRIVE.md](docs/FIRST_DRIVE.md).

### 2. Speed Control
PWM for a wheel = deadband + slope x request, with a short kick to start from rest, and a closed-loop trim on top. All of these are in one block in `config.h`, and `tools/wheel_test.py` measures them.
*   `FF_DEADBAND_*`: PWM at which the turning wheel stops.
*   `FF_SLOPE_*`: extra PWM per m/s.
*   `KICK_PWM_*`, `KICK_MS`: the push that starts a wheel from rest.
*   `K_P`, `K_I`, `TRIM_MAX_PWM`: the closed-loop trim and its limit. There's no derivative term, because speed from an encoder is too coarse for it. If a wheel hunts at low speed, halve both gains.
*   `CLOSED_LOOP`: 0 for feed-forward alone.
*   **Recommendation:** Tune these with the robot "on blocks" (wheels in the air) first.

---

## 📡 Communication Protocol (115200 Baud)

The ESP32 communicates with the Jetson Nano via a lightweight ASCII protocol.

### Downlink (Command)
**Format:** `m <left_v> <right_v>\r`
*   **Example:** `m 0.5 -0.5\r` (Spin in place).
*   **Safety:** If no `m` command is received for **500ms**, the robot will trigger an Emergency Stop (E-Stop).

### Uplink (Telemetry)
**Format:** `e <left_counts> <right_counts>\r\n`
*   **Example:** `e 1250 -1250\r\n`
*   **Frequency:** 50Hz.

**Format:** `i <ax> <ay> <az> <gx> <gy> <gz>\r\n`
*   **Units:** m/s² and rad/s, with 2 decimals.
*   **Frequency:** 50Hz.

---

## 🔌 Hardware Pin Mapping (ESP32-S3)

| Component | Function | Pin |
| :--- | :--- | :--- |
| **Left Motor** | IN1 / IN2 / PWM | 3 / 10 / 11 |
| **Right Motor**| IN1 / IN2 / PWM | 8 / 7 / 6 |
| **Left Encoder**| Phase A / B | 18 / 17 |
| **Right Encoder**| Phase A / B | 4 / 5 |
| **IMU (MPU6050)**| SDA / SCL | 1 / 21 |
| **Status LED** | WS2812 RGB | 48 |

---

## 🧠 Architectural Design

*   **Control task (High Priority):** A 100 Hz task reads the encoders, runs the speed control, and sets PWM. Nothing in it blocks.
*   **`loop()` (Low Priority):** Serial I/O, command parsing, telemetry, the IMU and the LED.
*   **Failsafe:** The control task enforces the command timeout itself. If the Jetson crashes, the USB cable is disconnected, or `loop()` stalls, the robot will stop within 500ms.

---

## 🛠️ Debugging

Use the **PlatformIO Serial Monitor** to watch the boot sequence. You should see `LLC_READY` when the system is initialized. 
*   A typed `m` command only lasts 500 ms. To drive by hand, use `tools/wheel_test.py`, with the wheels off the ground.

---
*Developed for the autoJetsonBot Project.*
