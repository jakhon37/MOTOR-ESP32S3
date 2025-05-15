autoJetsonBot ESP32-S3 Firmware (ROS2 Control)
This firmware runs on an ESP32-S3 microcontroller to control the autoJetsonBot differential drive robot, interfacing with ROS2 Humble via the diffdrive_arduino/DiffDriveArduino hardware interface over UART. It manages the TB6612FNG motor driver for two JGA25-370 DC motors with encoders, providing velocity control and odometry feedback. Serial monitor commands (m and e) enable standalone testing before ROS2 integration. Future plans include micro-ROS support.
Features

Motor Control: Drives two JGA25-370 motors using TB6612FNG with PWM and direction signals.
Encoder Feedback: Tracks wheel positions and velocities via quadrature encoders.
ROS2 Integration: Communicates with ros2_control via UART, publishing odometry and receiving velocity commands.
Serial Testing: Supports m (motor) and e (encoder) commands for debugging.
PlatformIO: Built with PlatformIO for efficient development.

Hardware Requirements (ROS2 Control)

Microcontroller: ESP32-S3 (e.g., ESP32-S3-DevKitC-1)
Motor Driver: TB6612FNG (dual motor driver)
Motors: 2x JGA25-370 DC 6V geared motors with quadrature encoders
Power:
Motors: 7.2V 4500mAh Li-ion battery (TB6612FNG VM)
ESP32-S3: 3.3V (e.g., via USB or regulator)


Jetson Nano B01: Runs ROS2 Humble, communicates via UART (/dev/ttyUSB0)

Software Dependencies

PlatformIO: Build and flash tool
ROS2: Humble Hawksbill (Jetson-side)
Python: 3.8+ (for PlatformIO)

Project Structure
├── include/
│   ├── README                # Configuration guide
│   └── config.h              # Pin assignments, robot parameters
├── lib/
│   ├── MicroROSNode/         # Placeholder for future micro-ROS integration
│   ├── MotorControl/
│   │   ├── MotorControl.cpp  # Motor control logic
│   │   ├── MotorControl.h    # Motor control interface
│   │   └── README            # Motor control notes
├── src/
│   └── main.cpp              # Main firmware logic
├── .gitignore                # Ignores build artifacts
├── README.md                 # This file
└── platformio.ini            # PlatformIO configuration

Wiring (ESP32-S3 to TB6612FNG and Encoders)

TB6612FNG Motor Driver:
PWMA (Left PWM): GPIO18 → TB6612FNG PWMA
AI1 (Left Dir): GPIO23 → TB6612FNG AI1
AI2 (Left Dir): GPIO24 → TB6612FNG AI2
PWMB (Right PWM): GPIO13 → TB6612FNG PWMB
BI1 (Right Dir): GPIO19 → TB6612FNG BI1
BI2 (Right Dir): GPIO26 → TB6612FNG BI2
STBY & VCC: 3.3V (ESP32-S3) → TB6612FNG
VM: 7.2V battery → TB6612FNG
GND: Common ground


JGA25-370 Motors:
Motor Outputs: TB6612FNG A01/A02 (left), B01/B02 (right)
Encoder Power: 3.3V (ESP32-S3), GND
Encoder Signals:
Left: GPIO5 (A), GPIO6 (B)
Right: GPIO17 (A), GPIO27 (B)




UART to Jetson Nano:
TX (GPIO43) → Jetson RX
RX (GPIO44) → Jetson TX
GND: Common ground



Installation

Clone Repository:
git clone https://github.com/yourusername/autoJetsonBot-firmware.git
cd autoJetsonBot-firmware


Install PlatformIO:

Install PlatformIO CLI or use VS Code with the PlatformIO extension.


Configure:

Edit include/config.h (see Configuration).
Verify platformio.ini for board = esp32-s3-devkitc-1.


Build and Upload:
pio run -t upload -t monitor


Uploads to ESP32-S3 via /dev/ttyUSB0.
Opens serial monitor (115200 baud).



Testing with Serial Monitor
Test motor and encoder functionality before ROS2 integration using the serial monitor (pio device monitor, 115200 baud).

Motor Command (m):
Format: m <left_speed> <right_speed> (floats, -1.0 to 1.0)
Example: m 0.5 -0.5 (left 50% forward, right 50% backward)
Response: Set: Left=0.5, Right=-0.5


Encoder Command (e):
Format: e
Example: e
Response: Enc: Left=1234, Right=5678



ROS2 Control Integration

Connect: Wire ESP32-S3 UART to Jetson Nano (/dev/ttyUSB0).
Configure ROS2:
Use diffdrive_arduino/DiffDriveArduino in ros2_control.
Update my_controllers.yaml:
left_wheel_names: ['left_wheel_joint']
right_wheel_names: ['right_wheel_joint']
wheel_separation: 0.18
wheel_radius: 0.035


Set UART in ros2_control.xacro:
Device: /dev/ttyUSB0
Baud rate: 57600
Encoder counts: 3436




Run:
Publish geometry_msgs/Twist to /cmd_vel.
Subscribe to /odom for odometry.


Future: Planned micro-ROS integration for direct ROS2 node functionality.

Configuration
Edit include/config.h:
#define WHEEL_RADIUS 0.035        // Meters
#define WHEEL_SEPARATION 0.18     // Meters
#define ENC_COUNTS_PER_REV 3436   // Encoder ticks per revolution
#define BAUD_RATE 57600           // ROS2 UART baud rate
#define SERIAL_BAUD 115200        // Serial monitor baud rate
#define PWM_LEFT_PINS {18}        // Left motor PWM
#define DIR_LEFT_PINS {23, 24}    // Left direction pins
#define PWM_RIGHT_PINS {13}       // Right motor PWM
#define DIR_RIGHT_PINS {19, 26}   // Right direction pins
#define ENC_LEFT_PINS {5, 6}      // Left encoder A, B
#define ENC_RIGHT_PINS {17, 27}   // Right encoder A, B

Troubleshooting

Serial Monitor Fails: Verify 115200 baud, USB permissions (sudo chmod 666 /dev/ttyUSB0).
Motors Not Moving: Check TB6612FNG STBY (high), VM (7.2V), and pin assignments.
Encoder Errors: Ensure 3.3V to encoders, correct GPIO mappings.
ROS2 No Data: Confirm UART baud (57600), ros2_control settings.

Contributing
Submit issues or pull requests to GitHub. See CONTRIBUTING.md.
License
MIT License. See LICENSE.
Acknowledgments

PlatformIO
ROS2 ros2_control

