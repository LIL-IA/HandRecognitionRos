# Hand Gesture Controlled Robot

A ROS2 Jazzy project for controlling a robot using hand gestures detected via webcam.

## Quick Start

**New to ROS2? Start here:** [SETUP_GUIDE.md](SETUP_GUIDE.md)

This guide will walk you through:
1. Installing ROS2 Jazzy
2. Setting up dependencies
3. Building the workspace
4. Testing the hand gesture system
5. Troubleshooting

## Overview

This project implements a hand gesture control system for robots based on the CENG483 Behavioral Robotics course project. It uses MediaPipe for hand detection and a **handlebar analogy** to translate gestures into velocity commands published on ROS2 topics that can control any robot subscribing to `/cmd_vel`.

### How It Works

Imagine holding a bike handlebar in front of the camera:
- **Push forward/pull back** → Control speed (throttle)
- **Twist handlebar (left hand forward, right back)** → Steer left/right
- **Hold still** → System calibrates neutral position
- Commands are **quantized to discrete steps** (-10 to +10) for stable, predictable robot control

### Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        System Architecture                               │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌──────────────┐      ┌─────────────────┐      ┌───────────────────┐   │
│  │   Webcam     │ ───► │ hand_drive_node │ ───► │    /cmd_vel       │   │
│  └──────────────┘      │   (MediaPipe)   │      │    (Twist)        │   │
│                        │                 │      └─────────┬─────────┘   │
│                        │  Handlebar      │                │             │
│                        │  Detection      │                ▼             │
│                        │       │         │      ┌───────────────────┐   │
│                        │       ▼         │      │    Your Robot     │   │
│                        │  Calibration    │      │  (subscribes to   │   │
│                        │       │         │      │    cmd_vel)       │   │
│                        │       ▼         │      └───────────────────┘   │
│                        │  Quantized      │                              │
│                        │  Commands       │      ┌───────────────────┐   │
│                        │  (-10 to +10)   │ ───► │  /drive_enabled   │   │
│                        └─────────────────┘      │    (Bool)         │   │
│                                                 └───────────────────┘   │
└─────────────────────────────────────────────────────────────────────────┘
```

## Hand Gesture Controls (Handlebar Analogy)

The system uses a **handlebar metaphor** - hold both hands in front of the camera as if gripping a bike handlebar.

### Calibration
| Action | Description |
|--------|-------------|
| **Hold both hands still** | System auto-calibrates to set neutral position (origin) |
| **Press 'C' key** | Manually reset calibration |
| **Auto-recalibration** | When hands are lost and return, system re-calibrates automatically |

### Speed Control (Throttle)
| Gesture | Action |
|---------|--------|
| **Push both hands forward** | Accelerate (move forward) |
| **Pull both hands back** | Decelerate / Reverse |
| **Enlarge handlebar span** | Increase speed |
| **Neutral position** | Stop |

### Steering Control
| Gesture | Action |
|---------|--------|
| **Left hand forward, right hand back** | Turn left |
| **Right hand forward, left hand back** | Turn right |
| **Tilt handlebar (rotate hands)** | Fine steering adjustment |

### System Controls
| Control | Action |
|---------|--------|
| **Both Hands Open (1.5s)** | Toggle drive enable/disable |
| **Press 'E' key** | Quick toggle drive enable/disable |
| **Press 'C' key** | Reset calibration (set new origin) |
| **Press 'Q' or ESC** | Quit |

### Control Details
- **Speed**: Calculated from depth push (55%), size change (35%), and span change (10%)
- **Steering**: Calculated from depth difference between hands (70%) and handlebar angle (30%)
- Commands are quantized to integer steps (-10 to +10) for smooth, stable control
- Built-in hysteresis prevents command chatter
- Safe stop activates when hands are lost for >0.4 seconds

## Installation

### Prerequisites

- Ubuntu 24.04 (Noble Numbat) or macOS (for development)
- ROS2 Jazzy Jalisco
- Python 3.10+
- Webcam

### Step-by-Step Installation

**Follow the complete setup guide:** [SETUP_GUIDE.md](SETUP_GUIDE.md)

Quick summary:
```bash
# 1. Install ROS2 Jazzy
sudo apt install ros-jazzy-desktop

# 2. Install Python dependencies
pip3 install numpy opencv-python mediapipe

# 3. Create workspace and build
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# Add your packages here
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Usage

### Launch Hand Gesture Control

```bash
# Source ROS2 and workspace
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Launch the hand gesture control node
ros2 launch hand_gesture_robot full_system.launch.py
```

Or run the node directly:

```bash
ros2 run hand_recognition hand_drive_node
```

## What to Expect

When you run the system, you should see:

### 1. **Camera Preview Window**
- A window titled "Bike Hands Drive" showing your webcam feed
- Hand landmarks drawn on detected hands (MediaPipe visualization)
- Real-time status overlay showing:
  - **Drive Status**: "Drive ENABLED" (green) or "Drive DISABLED" (red) with progress bar
  - **Cmd speed**: Current speed step (-10 to +10) with float value
  - **Cmd steer**: Current steering step (-10 to +10) with float value
  - **Calibration Status**: Progress indicator during calibration

### 2. **Terminal Output**
- ROS2 node initialization messages
- Camera opening confirmation
- Periodic status updates at ~15 Hz showing:
  ```
  drive_enabled=1 speed_step=+03 dir_step=-02 speed=+0.300 direction=-0.200
  ```
- Any error messages or warnings

### 3. **ROS2 Topics**

You can monitor the published topics in another terminal:

```bash
# Monitor velocity commands
ros2 topic echo /cmd_vel

# Monitor drive enabled status
ros2 topic echo /drive_enabled

# List all topics
ros2 topic list
```

### 4. **Expected Behavior**

- **Startup**: System waits for both hands to appear and calibrate
- **Calibration**: Hold both hands still - progress bar shows calibration status
- **Need both hands**: Message "Need both hands visible" appears if one hand is missing
- **Both hands open for 1.5s**: Drive toggles between ENABLED/DISABLED
- **Drive ENABLED**: Commands are published to `/cmd_vel`
- **Drive DISABLED**: Zero velocity commands are published (robot stops)
- **Hands lost (>0.4s)**: Safe stop activates with exponential decay
- **Auto-recalibration**: When hands return after being lost, system re-calibrates

## Launch Parameters

### hand_control.launch.py / full_system.launch.py

| Parameter | Default | Description |
|-----------|---------|-------------|
| `camera_id` | `0` | Camera device ID |
| `show_preview` | `true` | Show camera preview window |
| `max_linear_vel` | `0.22` | Maximum linear velocity (m/s) |
| `max_angular_vel` | `2.84` | Maximum angular velocity (rad/s) |
| `use_sim_time` | `false` | Use simulation clock (set to true if using with simulator) |

Example with custom parameters:

```bash
ros2 launch hand_gesture_robot hand_control.launch.py camera_id:=1 max_linear_vel:=0.15 show_preview:=false
```

## Troubleshooting

### Camera Not Found

```bash
# List available cameras
ls /dev/video*

# On macOS, check camera permissions
# System Preferences > Security & Privacy > Camera

# Try different camera ID
ros2 launch hand_gesture_robot hand_control.launch.py camera_id:=1
```

### Hand Detection Issues

- Ensure good lighting conditions
- Keep **both hands** within camera frame (handlebar control requires both)
- Maintain ~1-2 feet distance from camera
- Avoid complex backgrounds
- Make sure camera permissions are granted (macOS)
- Hold hands still during calibration (progress bar will fill up)

### Calibration Issues

- **"Hold still or press 'C' to set origin"**: Keep both hands steady for ~0.8 seconds
- **Calibration keeps resetting**: Reduce hand movement during calibration phase
- **Controls feel off**: Press 'C' to recalibrate and set a new neutral position
- **Auto-recalibration pending**: System lost sight of hands - show both hands again

### ROS2 Topics Not Publishing

```bash
# Check if node is running
ros2 node list

# Check if topics exist
ros2 topic list

# Check topic info
ros2 topic info /cmd_vel
ros2 topic hz /cmd_vel
```

### Preview Window Not Showing

If `show_preview:=false` or if you're running headless, the preview won't appear but the node will still publish commands. Check with:

```bash
ros2 topic echo /cmd_vel
```

## Project Structure

```
hand-gesture-robot/
├── package.xml                    # ROS2 package manifest
├── setup.py                       # Python package setup
├── setup.cfg                      # Setup configuration
├── README.md                      # This file
├── SETUP_GUIDE.md                 # Complete setup instructions
├── HARDWARE_SETUP.md              # ESP32 + Arduino setup guide
├── resource/
│   └── hand_gesture_robot         # Package marker
├── hand_gesture_robot/
│   └── __init__.py               # Python package init
├── launch/
│   ├── hand_control.launch.py    # Hand gesture control node
│   └── full_system.launch.py     # Main launch file
├── config/
│   └── robot_params.yaml         # Robot parameters
└── firmware/
    ├── README.md                  # Firmware documentation
    ├── QUICK_START.md             # Quick start for hardware
    ├── esp32_master/
    │   └── esp32_master.ino       # ESP32 master code
    └── arduino_slave/
        └── arduino_slave.ino       # Arduino slave code

HandRecognition/
├── package.xml                    # ROS2 package manifest
├── setup.py                       # Python package setup
├── setup.cfg                      # Setup configuration
├── requirements.txt               # Python dependencies
├── resource/
│   └── hand_recognition          # Package marker
├── hand_recognition/
│   ├── __init__.py               # Python package init
│   └── hand_drive_node.py        # ROS2 hand gesture control node
└── hand_drive.py                  # Original standalone script
```

## ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands for robot (linear.x, angular.z) |
| `/drive_enabled` | `std_msgs/Bool` | Drive enable/disable status |

### Topic Details

**`/cmd_vel` (geometry_msgs/Twist)**:
- `linear.x`: Forward/backward velocity (m/s), range: -max_linear_vel to +max_linear_vel
  - Derived from quantized speed steps (-10 to +10) × step_size (0.1)
- `linear.y`: Always 0 (not used)
- `linear.z`: Always 0 (not used)
- `angular.x`: Always 0 (not used)
- `angular.y`: Always 0 (not used)
- `angular.z`: Rotational velocity (rad/s), range: -max_angular_vel to +max_angular_vel
  - Derived from quantized steering steps (-10 to +10) × step_size (0.1)

**`/drive_enabled` (std_msgs/Bool)**:
- `data`: `true` when drive is enabled, `false` when disabled

### Quantized Command System

The handlebar control uses quantized steps for stable, predictable control:

| Parameter | Value | Description |
|-----------|-------|-------------|
| Step size | 0.1 | Each step represents 10% of max velocity |
| Range | -10 to +10 | 21 discrete positions |
| Hysteresis | 55% of step | Prevents command chatter |
| Zero lock | 0.02 | Tiny values snap to zero |
| Smoothing | 6.0/s | Low-pass filter rate |

## Connecting to Your Robot

### Option 1: ESP32 + Arduino (Master-Slave Configuration)

For connecting to a real TurtleBot using ESP32 and Arduino, see the detailed guide:

**[HARDWARE_SETUP.md](HARDWARE_SETUP.md)** - Complete step-by-step guide for ESP32 + Arduino setup

This includes:
- Hardware wiring diagrams
- ESP32 firmware (micro-ROS subscriber)
- Arduino firmware (motor control)
- Communication protocol
- Troubleshooting guide

### Option 2: Direct ROS2 Node

To connect this to any robot that already has ROS2, ensure your robot's control node subscribes to `/cmd_vel`:

```python
# Example robot subscriber
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
    
    def cmd_vel_callback(self, msg):
        # Convert Twist message to motor commands
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        # ... send to your robot hardware
```

## License

MIT License

## Authors

- Esin Eltimür
- Hasan Çoban
- Lilian L'helgoualc'h

CENG483 Behavioral Robotics - Fall 2025
