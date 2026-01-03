# Hand Gesture Controlled Robot (ROS2)

Control a robot using hand gestures detected via webcam. Uses a **handlebar analogy** - hold both hands like gripping a bike handlebar to steer and accelerate.

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![Python](https://img.shields.io/badge/Python-3.10+-green)](https://python.org)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)

## Quick Demo

```
        🖐️ ────────────────── 🖐️
        Left                Right
        
    Push forward  →  Accelerate
    Pull back     →  Reverse
    Twist bar     →  Steer left/right
```

---

## 📦 Project Structure

```
HandGestureRos/
├── HandRecognition/           # Hand gesture detection ROS2 package
│   ├── hand_recognition/
│   │   └── hand_drive_node.py # Main ROS2 node
│   └── hand_drive.py          # Standalone demo script
│
└── hand-gesture-robot/        # Robot control package
    ├── launch/                # ROS2 launch files
    ├── config/                # Robot parameters
    ├── firmware/              # ESP32 + Arduino code
    ├── README.md              # Detailed documentation
    └── SETUP_GUIDE.md         # Step-by-step setup
```

---

## 🚀 Quick Installation

### Prerequisites

| Requirement | Version |
|-------------|---------|
| Ubuntu | 24.04 (Noble Numbat) |
| ROS2 | Jazzy Jalisco |
| Python | 3.10+ |
| Webcam | Any USB webcam |

### Step 1: Install ROS2 Jazzy

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Set locale
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 repository
sudo apt install software-properties-common curl -y
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Jazzy Desktop
sudo apt update
sudo apt install ros-jazzy-desktop python3-argcomplete python3-colcon-common-extensions -y

# Add to bashrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 2: Install Python Dependencies

```bash
pip3 install numpy opencv-python mediapipe
```

### Step 3: Create ROS2 Workspace & Build

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone or symlink this repository
git clone <your-repo-url> HandGestureRos
# Or if you already have it:
# ln -s /path/to/HandGestureRos .

# Create symlinks to packages (required for ROS2)
ln -s HandGestureRos/HandRecognition hand_recognition
ln -s HandGestureRos/hand-gesture-robot hand_gesture_robot

# Build
cd ~/ros2_ws
colcon build --symlink-install

# Source workspace
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 4: Verify Installation

```bash
# Check packages are found
ros2 pkg list | grep -E "hand_recognition|hand_gesture"

# Should output:
# hand_gesture_robot
# hand_recognition
```

---

## 🎮 Running the System

### Option 1: Launch File (Recommended)

```bash
ros2 launch hand_gesture_robot full_system.launch.py
```

### Option 2: Run Node Directly

```bash
ros2 run hand_recognition hand_drive_node
```

### Option 3: Standalone Demo (No ROS2)

```bash
cd HandRecognition
python3 hand_drive.py
```

---

## 🖐️ Controls

| Gesture | Action |
|---------|--------|
| **Hold both hands still** | Calibrate neutral position |
| **Push forward** | Accelerate |
| **Pull back** | Reverse |
| **Left hand forward, right back** | Turn left |
| **Right hand forward, left back** | Turn right |
| **Both hands open (1.5s)** | Toggle drive ON/OFF |
| **Press 'E'** | Toggle drive ON/OFF |
| **Press 'C'** | Recalibrate |
| **Press 'Q' / ESC** | Quit |

---

## 📡 ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands (linear.x, angular.z) |
| `/drive_enabled` | `std_msgs/Bool` | Drive enable status |

### Monitor Topics

```bash
# In another terminal
ros2 topic echo /cmd_vel
ros2 topic echo /drive_enabled
ros2 topic hz /cmd_vel
```

---

## ⚙️ Launch Parameters

```bash
ros2 launch hand_gesture_robot hand_control.launch.py \
    camera_id:=0 \
    max_linear_vel:=0.22 \
    max_angular_vel:=2.84 \
    show_preview:=true
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `camera_id` | `0` | Camera device ID |
| `max_linear_vel` | `0.22` | Max forward speed (m/s) |
| `max_angular_vel` | `2.84` | Max turn rate (rad/s) |
| `show_preview` | `true` | Show camera window |

---

## 🔧 Troubleshooting

### "ros2: command not found"
```bash
source /opt/ros/jazzy/setup.bash
```

### "Package not found"
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Camera not opening
```bash
# Check available cameras
ls /dev/video*

# Try different camera ID
ros2 launch hand_gesture_robot hand_control.launch.py camera_id:=1
```

### Python import errors
```bash
pip3 install --upgrade numpy opencv-python mediapipe
```

---

## 📚 Documentation

| Document | Description |
|----------|-------------|
| [hand-gesture-robot/README.md](hand-gesture-robot/README.md) | Detailed system documentation |
| [hand-gesture-robot/SETUP_GUIDE.md](hand-gesture-robot/SETUP_GUIDE.md) | Complete step-by-step setup |
| [hand-gesture-robot/HARDWARE_SETUP.md](hand-gesture-robot/HARDWARE_SETUP.md) | ESP32 + Arduino wiring guide |
| [hand-gesture-robot/firmware/README.md](hand-gesture-robot/firmware/README.md) | Firmware documentation |

---

## 🤝 Authors

- **Esin Eltimür**
- **Hasan Çoban**
- **Lilian L'helgoualc'h**

CENG483 Behavioral Robotics - Fall 2025

---

## 📄 License

MIT License - see [LICENSE](LICENSE) for details.
