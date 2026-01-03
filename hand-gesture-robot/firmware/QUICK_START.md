# Quick Start Guide: ESP32 + Arduino Setup

This is a condensed quick-start guide. For detailed information, see [HARDWARE_SETUP.md](../HARDWARE_SETUP.md).

## Prerequisites Checklist

- [ ] ESP32 development board
- [ ] Arduino Uno/Nano
- [ ] Motor driver (L298N, TB6612FNG, or DRV8833)
- [ ] 2x DC motors
- [ ] 12V battery for motors
- [ ] USB cables for programming
- [ ] Jumper wires
- [ ] ROS2 Jazzy installed
- [ ] micro-ROS installed

## Step 1: Install micro-ROS (5-10 minutes)

```bash
# Install ESP-IDF
mkdir -p ~/esp && cd ~/esp
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf && ./install.sh esp32
. ./export.sh

# Install micro-ROS
mkdir -p ~/microros_ws/src
cd ~/microros_ws
git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

## Step 2: Flash ESP32 (2-3 minutes)

```bash
# Find ESP32 port
ls /dev/ttyUSB* /dev/ttyACM*

# Flash micro-ROS firmware
ros2 run micro_ros_setup create_firmware_ws.sh host
ros2 run micro_ros_setup configure_firmware.sh subscriber --transport serial
ros2 run micro_ros_setup build_firmware.sh
ros2 run micro_ros_setup flash_firmware.sh /dev/ttyUSB0
```

## Step 3: Upload Firmware Code

### ESP32 (Master)
1. Open `esp32_master/esp32_master.ino` in Arduino IDE
2. Install ESP32 board support (if not already installed)
3. Select board: Tools → Board → ESP32 Dev Module
4. Upload code

### Arduino (Slave)
1. Open `arduino_slave/arduino_slave.ino` in Arduino IDE
2. Adjust pin numbers if your wiring differs
3. Upload code

## Step 4: Wire Hardware

### ESP32 ↔ Arduino
- ESP32 TX → Arduino RX
- ESP32 RX → Arduino TX  
- GND → GND

### Arduino → Motor Driver (L298N example)
- D5 → IN1 (Left forward)
- D6 → IN2 (Left backward)
- D9 → ENA (Left PWM)
- D10 → IN3 (Right forward)
- D11 → IN4 (Right backward)
- D3 → ENB (Right PWM)

### Motor Driver → Motors
- OUT1/OUT2 → Left Motor
- OUT3/OUT4 → Right Motor
- 12V Battery → VCC
- GND → GND

## Step 5: Start micro-ROS Agent

```bash
# Terminal 1: Start agent
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

## Step 6: Test Connection

```bash
# Terminal 2: Check topics
source /opt/ros/jazzy/setup.bash
ros2 topic list
ros2 topic echo /cmd_vel
```

You should see `/cmd_vel` in the topic list.

## Step 7: Start Hand Gesture Control

```bash
# Terminal 3: Start hand gesture node
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch hand_gesture_robot full_system.launch.py
```

## Step 8: Test Robot

1. Make hand gestures in front of camera
2. Verify `/cmd_vel` messages are published (Terminal 2)
3. Robot should move according to gestures

## Troubleshooting

**ESP32 not connecting?**
```bash
# Check port permissions
sudo chmod 666 /dev/ttyUSB0
# Or add user to dialout group
sudo usermod -a -G dialout $USER
# Log out and back in
```

**Arduino not receiving commands?**
- Check Serial Monitor is closed (blocks communication)
- Verify baud rate matches (115200)
- Check wiring (TX→RX, RX→TX)

**Motors not moving?**
- Check motor driver power (12V)
- Test motors directly with battery
- Verify PWM pins are correct
- Check motor driver enable pins

## Next Steps

- Calibrate motor speeds in Arduino code
- Add encoder feedback
- Add safety features (emergency stop)
- Monitor battery voltage

For detailed information, see [HARDWARE_SETUP.md](../HARDWARE_SETUP.md).

