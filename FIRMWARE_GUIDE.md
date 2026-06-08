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
*   `ENCODER_CPR`: **3436** counts/rev.

### 2. PID Tuning
*   `K_P`: Proportional gain (Immediate response).
*   `K_I`: Integral gain (Eliminates steady-state error).
*   `K_D`: Derivative gain (Dampens oscillations).
*   **Recommendation:** Tune these with the robot "on blocks" (wheels in the air) first.

---

## 📡 Communication Protocol (115200 Baud)

The ESP32 communicates with the Jetson Nano via a lightweight ASCII protocol.

### Downlink (Command)
**Format:** `m <left_v> <right_v>\r`
*   **Example:** `m 0.5 -0.5\r` (Spin in place).
*   **Safety:** If no `m` command is received for **500ms**, the robot will trigger an Emergency Stop (E-Stop).

### Uplink (Telemetry)
**Format:** `e <left_ticks> <right_ticks>\n`
*   **Example:** `e 1250 -1250\n`
*   **Frequency:** 50Hz.

---

## 🔌 Hardware Pin Mapping (ESP32-S3)

| Component | Function | Pin |
| :--- | :--- | :--- |
| **Left Motor** | IN1 / IN2 / PWM | 3 / 10 / 11 |
| **Right Motor**| IN1 / IN2 / PWM | 8 / 7 / 6 |
| **Left Encoder**| Phase A / B | 13 / 14 |
| **Right Encoder**| Phase A / B | 4 / 5 |
| **Status LED** | Activity Blinker| 2 |

---

## 🧠 Architectural Design

*   **Core 1 (High Priority):** Handles the `controlLoop` task. This is a deterministic 50Hz loop that measures encoders, calculates PID, and sets PWM.
*   **Core 0 (Low Priority):** Handles the `loop()` function for Serial I/O and asynchronous command parsing.
*   **Failsafe:** The firmware includes a software watchdog. If the Jetson Nano crashes or the USB cable is disconnected, the robot will stop within 500ms.

---

## 🛠️ Debugging

Use the **PlatformIO Serial Monitor** to watch the boot sequence. You should see `LLC_READY` when the system is initialized. 
*   To test manually, type `m 0.2 0.2` and press Enter to move the wheels at 0.2 m/s.

---
*Developed for the autoJetsonBot Project.*
