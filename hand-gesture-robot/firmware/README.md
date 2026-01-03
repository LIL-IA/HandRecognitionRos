# Firmware for ESP32 + Arduino Master-Slave Configuration

This directory contains firmware code for connecting your hand gesture control system to a TurtleBot.

## Directory Structure

```
firmware/
├── README.md                    # This file
├── esp32_master/
│   ├── esp32_master.ino         # ESP32 master code (micro-ROS subscriber)
│   └── platformio.ini           # PlatformIO configuration (optional)
└── arduino_slave/
    └── arduino_slave.ino        # Arduino slave code (motor control)
```

## Quick Start

1. **ESP32 (Master)**:
   - Open `esp32_master/esp32_master.ino` in Arduino IDE or PlatformIO
   - Configure WiFi credentials (if using WiFi)
   - Upload to ESP32

2. **Arduino (Slave)**:
   - Open `arduino_slave/arduino_slave.ino` in Arduino IDE
   - Adjust pin numbers to match your wiring
   - Upload to Arduino

3. **Connect Hardware**:
   - Connect ESP32 TX → Arduino RX
   - Connect ESP32 RX → Arduino TX
   - Connect GND → GND

4. **Start micro-ROS Agent**:
   ```bash
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
   ```

5. **Test**:
   ```bash
   ros2 topic echo /cmd_vel
   ```

## Configuration

### ESP32 Configuration

Edit these constants in `esp32_master.ino`:
- `WIFI_SSID`: Your WiFi network name
- `WIFI_PASSWORD`: Your WiFi password
- `SERIAL_BAUD`: Serial communication speed (default: 115200)

### Arduino Configuration

Edit these constants in `arduino_slave.ino`:
- `LEFT_MOTOR_PWM`: PWM pin for left motor speed
- `LEFT_MOTOR_IN1`: Left motor forward pin
- `LEFT_MOTOR_IN2`: Left motor backward pin
- `RIGHT_MOTOR_PWM`: PWM pin for right motor speed
- `RIGHT_MOTOR_IN1`: Right motor forward pin
- `RIGHT_MOTOR_IN2`: Right motor backward pin
- `SERIAL_BAUD`: Must match ESP32 (default: 115200)
- `WHEEL_RADIUS`: Wheel radius in meters
- `WHEEL_BASE`: Distance between wheels in meters

## Communication Protocol

### ESP32 → Arduino

Serial communication at 115200 baud, format:
```
<linear_velocity>,<angular_velocity>\n
```

Example:
```
0.15,-0.5\n
```

Where:
- `linear_velocity`: Forward/backward speed in m/s (-0.22 to +0.22)
- `angular_velocity`: Rotational speed in rad/s (-2.84 to +2.84)

### Error Handling

- If no message received for 1 second, motors stop (safety feature)
- Invalid messages are ignored
- Checksum can be added for reliability

## Motor Control

The Arduino implements differential drive kinematics:

```
v_left = linear_vel - (angular_vel * wheel_base / 2)
v_right = linear_vel + (angular_vel * wheel_base / 2)
```

Motor speeds are converted to PWM values (0-255) and sent to motor driver.

