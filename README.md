<!-- ````markdown -->
# ESP32-S3 Custom Firmware for autoJetsonBot

This repository contains the custom firmware for the ESP32‑S3 microcontroller used in the autoJetsonBot project. The firmware enables differential‑drive control and encoder reading for a two‑wheeled robot, interfacing with a ROS2 node via serial communication. It supports ASCII‑based commands for motor control (`m`) and encoder reading (`e`).

## Table of Contents

1. [Overview](#overview)  
2. [Hardware Requirements](#hardware-requirements)  
3. [Firmware Features](#firmware-features)  
4. [Installation](#installation)  
   1. [Prerequisites](#prerequisites)  
   2. [Steps](#steps)  
5. [Serial Communication Protocol](#serial-communication-protocol)  
6. [Testing with Serial Monitor](#testing-with-serial-monitor)  
7. [Integration with ROS2](#integration-with-ros2)  
8. [Troubleshooting](#troubleshooting)  
9. [Contributing](#contributing)  
10. [License](#license)  

---

## Overview

The firmware runs on an ESP32‑S3 microcontroller and controls a differential‑drive robot with two DC motors equipped with quadrature encoders. It processes serial commands to set motor speeds and report encoder counts, enabling integration with a ROS2 node for autonomous navigation. Before connecting to ROS2, the firmware can be tested using a serial monitor.

---

## Hardware Requirements

- **Microcontroller**: ESP32‑S3 (e.g., DevKitC‑1)  
- **Motors**: Two DC motors with quadrature encoders  
- **Motor Driver**: PWM‑compatible (e.g., L298N, TB6612FNG)  
- **Power Supply**: 3.3 V for ESP32, 6–12 V for motors  
- **USB‑to‑Serial Adapter**: For flashing and serial monitoring  
- **Jetson Nano**: Running ROS2 Foxy  
- **Serial Connection**: USB cable or UART (e.g., `/dev/ttyACM0`)  

---

## Firmware Features

- **Motor Control**  
  - `m <left> <right>`: Set PWM speeds or scaled velocity for left and right motors  
- **Encoder Reading**  
  - `e`: Return `<left_count> <right_count>`  
- **Serial Protocol**  
  - ASCII‑based, 115200 baud (8N1), commands end with `\r`  
- **Robustness**  
  - Handles invalid commands gracefully  
- **Configurability**  
  - Encoder CPR and motor pins defined in `diff_drive_config.h`  

---

## Installation

### Prerequisites

- Arduino IDE or PlatformIO with ESP32‑S3 support  
- ESP32 Arduino Core (install via Boards Manager)  
- Serial‑monitor tool (minicom, Arduino IDE, PlatformIO)  
- USB‑to‑Serial drivers (e.g., CP2102)  

### Steps

1. **Clone Repository**  
   ```bash
   git clone <repository_url>
   cd autoJetsonBot-firmware
<!-- ```` -->

2. **Configure Firmware**
   Edit `src/diff_drive_config.h`:

   ```cpp
   #define SERIAL_BAUD_RATE 115200
   #define ENCODER_CPR       3436
   #define LEFT_MOTOR_PIN1   12
   #define LEFT_MOTOR_PIN2   13
   #define RIGHT_MOTOR_PIN1  14
   #define RIGHT_MOTOR_PIN2  15
   #define LEFT_ENCODER_A    16
   #define LEFT_ENCODER_B    17
   #define RIGHT_ENCODER_A   18
   #define RIGHT_ENCODER_B   19
   ```

3. **Compile & Upload**

   * **Arduino IDE**

     1. Open `diff_drive.ino`
     2. Select **ESP32S3 Dev Module**, set port to `/dev/ttyACM0`
     3. Click **Upload**
   * **PlatformIO**

     ```bash
     pio run -t upload
     ```

4. **Verify Upload**

   ```bash
   minicom -D /dev/ttyACM0 -b 115200
   ```

   Look for a boot message, e.g., `ESP32-S3 DiffDrive Firmware Initialized`.

---

## Serial Communication Protocol

* **Baud Rate**: 115200, 8 data bits, no parity, 1 stop bit
* **Line Ending**: `\r` (carriage return)

### Commands

* **Motor Control**

  ```
  m <left> <right>\r
  ```

  * `left`, `right`: Integer in –255…255
  * Response: `OK` (or error message)

* **Encoder Reading**

  ```
  e\r
  ```

  * Response: `<left_count> <right_count>`

---

## Testing with Serial Monitor

1. **Connect & Identify Port**

   ```bash
   ls /dev/tty*
   ```

2. **Open Serial Monitor**

   * **minicom**:

     ```bash
     sudo apt-get install minicom
     minicom -D /dev/ttyACM0 -b 115200
     ```

     Enable CR ending: `Ctrl+A`, `Z`, `U`
   * **Arduino IDE** or **PlatformIO**: set baud to 115200, line ending to CR

3. **Test Procedures**

   * **Read Encoders**

     ```
     e\r
     ```

     Expect: `-4274 1471` (values change when wheels turn)

   * **Motor Control**

     ```
     m 100 100\r   # forward
     m -100 -100\r # backward
     m 0 0\r       # stop
     ```

   * **Spin in Place**

     ```
     m 100 -100\r
     ```

   * **Dynamic Read**

     ```
     m 100 100\r
     e\r
     ```

---

## Integration with ROS2

1. **Prerequisites**

   * ROS2 Foxy installed on Jetson Nano
   * `diffdrive_arduino` ROS2 package

2. **Hardware Config**
   `diffbot_hardware.yaml`:

   ```yaml
   diffdrive_arduino:
     ros__parameters:
       left_wheel_name:  left_wheel_joint
       right_wheel_name: right_wheel_joint
       loop_rate:        30.0
       device:           /dev/ttyACM0
       baud_rate:        115200
       timeout:          1000
       enc_counts_per_rev: 3436
   ```

3. **Launch & Test**

   ```bash
   cd ~/myspace/autoJetsonBot
   source install/setup.bash
   ros2 launch robot_body diffbot.launch.py
   ros2 topic echo /rosout  # look for serial port info
   ros2 topic pub /diff_cont/cmd_vel_unstamped geometry_msgs/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
   ros2 topic echo /odom
   ```

---

## Troubleshooting

* **Garbled Output**: Confirm 115200 baud in firmware and monitor
* **No Serial Response**: Check port, power, permissions:

  ```bash
  sudo chmod 666 /dev/ttyACM0
  ```
* **Motors Not Moving**: Verify wiring, power; try `m 200 200\r`
* **ROS2 Connection Error**: Ensure no other process (e.g., minicom) holds `/dev/ttyACM0`

  ```bash
  lsof /dev/ttyACM0
  pkill -f minicom
  ```

---

## Contributing

1. Fork the repo
2. Create feature branch:

   ```bash
   git checkout -b feature/xyz
   ```
3. Commit & push:

   ```bash
   git commit -m "Add XYZ"
   git push origin feature/xyz
   ```
4. Open a pull request

---

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.

```
```
