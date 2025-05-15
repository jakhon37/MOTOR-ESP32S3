# ESP32‑S3 Firmware for autoJetsonBot

Lightweight firmware for ESP32‑S3 to control a differential‑drive robot via serial commands before ROS2 integration.

## Features
- **Motor Control**: `m <left> <right>\r` (–255…255), responds `OK`  
- **Encoder Read**: `e\r` → `<left_count> <right_count>`  
- **Serial**: ASCII, 115200 baud, 8N1, commands end with `\r`

## Quick Start

1. **Clone & Configure**
   ```bash
   git clone <repo>
   cd autoJetsonBot-firmware

Edit `src/diff_drive_config.h` pins/CPR/baud as needed.

2. **Build & Upload**

   * **Arduino IDE**: Select **ESP32S3 Dev Module**, port `/dev/ttyACM0`, Upload
   * **PlatformIO**:

     ```bash
     pio run -t upload
     ```

3. **Serial Test**

   ```bash
   minicom -D /dev/ttyACM0 -b 115200
   ```

   * **Read Encoders**:

     ```
     e\r
     ```
   * **Drive Motors**:

     ```
     m 100 100\r  # forward
     m 0 0\r      # stop
     ```

## ROS2 Integration

1. **Config** (`diffbot_hardware.yaml`):

   ```yaml
   diffdrive_arduino:
     ros__parameters:
       device: /dev/ttyACM0
       baud_rate: 115200
       enc_counts_per_rev: 3436
   ```
2. **Launch & Test**:

   ```bash
   source install/setup.bash
   ros2 launch robot_body diffbot.launch.py
   ros2 topic pub /diff_cont/cmd_vel_unstamped geometry_msgs/Twist "{linear: {x:0.5},angular:{z:0}}"
   ros2 topic echo /odom
   ```

## Troubleshooting

* **No/garbled serial**: Check baud (`115200`) and line ending (`CR`).
* **No motor response**: Verify wiring, power, try `m 200 200\r`.
* **ROS2 port busy**: Stop minicom (`pkill -f minicom`).

## License

MIT – see [LICENSE](LICENSE).

```
```
