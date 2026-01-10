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

## Architecture

> 📖 **Full architecture documentation**: [ARCHITECTURE.md](ARCHITECTURE.md)

The project uses a **client-server architecture** where gesture recognition runs locally on your laptop, and the server bridges to ROS2 and ESP32 via MQTT.

```
┌─────────────────────────────────────────────────────────────────────────────┐
│  CLIENT (Your Laptop - macOS/Windows/Linux)                                 │
│                                                                             │
│   Camera → Frame Gate → MediaPipe → Hand Control → JSON → WebSocket        │
│            (validate)               (gestures)     (validate)               │
└─────────────────────────────────────────│───────────────────────────────────┘
                                          │ ws://server:8080/control
                                          ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│  SERVER (Ubuntu VM / AWS EC2)                                               │
│                                                                             │
│   WebSocket ──┬──→ ROS2 Bridge ──→ /cmd_vel, /drive_enabled                │
│   (auth)      │                                                             │
│               └──→ MQTT Bridge ──→ robot/cmd ──→ ESP32 ──→ Motors          │
│                                                                             │
│   Deadman Timer: Auto-stop if no message for 300ms                         │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Why This Architecture?

| Problem | Solution |
|---------|----------|
| RTSP H264 decoder errors produce corrupted frames | **Frame Gate** on client detects and drops bad frames |
| Bad frames → wrong velocity commands | Gestures computed **locally**, only valid commands sent |
| ESP32 can't run ROS2 easily | **MQTT Bridge** sends simple JSON to ESP32 |
| Client and robot on different networks | Server can run on **cloud** (AWS EC2) with public IP |

### Key Safety Features

| Feature | Description |
|---------|-------------|
| 🖼️ **Frame Gate** | Drops corrupted/invalid frames before processing |
| 🖐️ **Hand Gate** | Requires both hands visible + calibrated |
| ✅ **Message Validation** | Rejects NaN/Inf/out-of-bounds values |
| ⏱️ **Deadman Timer** | Server stops robot if silent for 300ms |
| 🔒 **Single Controller** | Only one client can control at a time |
| 🛑 **Graceful Shutdown** | Sends STOP on disconnect |

---

## 📦 Project Structure

```
HandGestureRos/
├── client_hand_control/       # Standalone client (NO ROS2)
│   ├── main.py               # CLI entry point
│   ├── frame_gate.py         # Frame quality validation
│   ├── hand_control.py       # Handlebar control logic
│   ├── message.py            # JSON schema + validation
│   ├── ws_client.py          # WebSocket client
│   └── requirements.txt
│
├── server_gateway/            # ROS2 + MQTT gateway server
│   ├── main.py               # Server entry point
│   ├── ws_server.py          # WebSocket server + auth
│   ├── ros_bridge.py         # ROS2 publishers + deadman
│   ├── mqtt_bridge.py        # MQTT for ESP32
│   └── requirements.txt
│
├── HandRecognition/           # Original ROS2 node (direct mode)
│   ├── hand_recognition/
│   │   └── hand_drive_node.py
│   └── hand_drive.py         # Standalone demo
│
└── hand-gesture-robot/        # Robot control package
    ├── launch/
    ├── config/
    └── firmware/             # ESP32 + Arduino code
```

---

## 🚀 Quick Start

### Prerequisites

| Requirement | Version |
|-------------|---------|
| Ubuntu | 22.04+ or macOS |
| Python | 3.10+ |
| ROS2 (server only) | Jazzy/Humble |
| Webcam | Any USB webcam or RTSP stream |

### Installation

#### Client (User Laptop - No ROS2 Needed)

```bash
cd client_hand_control
pip install -r requirements.txt
```

#### Server (Robot Machine - Needs ROS2)

```bash
# Install ROS2 dependencies
sudo apt install ros-jazzy-desktop python3-colcon-common-extensions -y

# Install server dependencies
cd server_gateway
pip install -r requirements.txt

# Build ROS2 workspace
cd ~/ros2_ws
ln -s /path/to/HandGestureRos/server_gateway src/
colcon build --packages-select server_gateway
source install/setup.bash
```

---

## 🎮 Running the System

### Local Testing (All on One Machine)

**Terminal 1: Start MQTT Broker (Optional)**
```bash
# Only needed if using MQTT bridge to ESP32
mosquitto -v
```

**Terminal 2: Start Server Gateway**
```bash
cd /path/to/HandGestureRos
export CONTROL_TOKEN=test123
python -m server_gateway.main
```

**Terminal 3: Start Client**
```bash
cd /path/to/HandGestureRos

# With local webcam
python -m client_hand_control.main \
    --server ws://127.0.0.1:8080/control \
    --token test123 \
    --camera 0 \
    --preview

# Or with RTSP stream
python -m client_hand_control.main \
    --server ws://127.0.0.1:8080/control \
    --token test123 \
    --rtsp "rtsp://10.8.34.150:8554/handcam" \
    --preview
```

**Terminal 4: Verify ROS2 Output**
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /cmd_vel
ros2 topic echo /drive_enabled
```

### Direct ROS2 Mode (Original)

```bash
# Launch with ROS2
ros2 launch hand_gesture_robot full_system.launch.py

# Or run node directly
ros2 run hand_recognition hand_drive_node
```

---

## 🛠️ Configuration

### Client CLI Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `--server` | `ws://127.0.0.1:8080/control` | WebSocket server URL |
| `--token` | (required) | Authentication token |
| `--camera` | `0` | Camera device index |
| `--rtsp` | `None` | RTSP URL (overrides camera) |
| `--max-linear` | `0.22` | Max linear velocity (m/s) |
| `--max-angular` | `2.84` | Max angular velocity (rad/s) |
| `--rate` | `30` | Control loop rate (Hz) |
| `--preview` | `false` | Show camera preview window |
| `--invalid-timeout` | `300` | Frame invalid timeout (ms) |

### Server Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `CONTROL_TOKEN` | (required) | Client authentication token |
| `DEADMAN_MS` | `300` | Deadman timeout (ms) |
| `MAX_LINEAR` | `0.22` | Max linear velocity (m/s) |
| `MAX_ANGULAR` | `2.84` | Max angular velocity (rad/s) |
| `MQTT_HOST` | `localhost` | MQTT broker host |
| `MQTT_PORT` | `1883` | MQTT broker port |

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
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |
| `/drive_enabled` | `std_msgs/Bool` | Drive enable status |

---

## 🔧 Troubleshooting

### RTSP Stream Corruption

If you see H264 decoder errors like "corrupted macroblock" or "Invalid level prefix":

1. **Use TCP transport** (more reliable than UDP):
   ```bash
   --rtsp "rtsp://10.8.34.150:8554/handcam?rtsp_transport=tcp"
   ```

2. **Lower camera bitrate** to 2-4 Mbps

3. **Reduce resolution** to 720p or lower

4. **Expected behavior**: The frame gate will automatically drop corrupted frames. The robot will NOT receive incorrect commands from bad frames.

### "Connection refused" Error

```bash
# Make sure server is running first
export CONTROL_TOKEN=test123
python -m server_gateway.main
```

### Camera Not Opening

```bash
# List available cameras
ls /dev/video*

# Try different camera index
python -m client_hand_control.main --camera 1 --token test123 --preview
```

### ROS2 Topics Not Publishing

```bash
# Check if ROS2 bridge is running
ros2 node list | grep gateway

# Check for errors in server output
# Server will log "ROS bridge started" if successful
```

### MQTT Connection Failed

```bash
# Start local MQTT broker
mosquitto -v

# Or disable MQTT (server will continue without it)
# Just don't connect any ESP32 via MQTT
```

---

## 🔒 Safety Features

| Feature | Description |
|---------|-------------|
| **Frame Gate** | Drops corrupted/invalid camera frames |
| **Hand Gate** | Requires both hands visible + calibrated |
| **Message Validation** | Rejects NaN/Inf/out-of-bounds values |
| **Deadman Timer** | Auto-stop if no message for 300ms |
| **Single Controller Lock** | Only one client can control at a time |
| **Graceful Shutdown** | Sends STOP on client disconnect |

---

## 🌐 Deployment Options

> 📖 **Full details**: [ARCHITECTURE.md](ARCHITECTURE.md#deployment-options)

| Setup | Client | Server | ESP32 | Best For |
|-------|--------|--------|-------|----------|
| **Local** | Same machine | Same machine | USB/local MQTT | Development |
| **VM** | macOS | Ubuntu VM (192.168.64.x) | Local MQTT | Testing with ROS2 |
| **Cloud** | Anywhere | AWS EC2 (public IP) | Internet MQTT | Production |

### ESP32 Connection (MQTT)

The ESP32 connects to the server via **MQTT** (not ROS2). Update your ESP32 firmware:

```cpp
// esp32_master.ino
const char* mqtt_server = "YOUR_SERVER_IP";  // VM IP or EC2 public IP
const int mqtt_port = 1883;
```

The server's MQTT bridge converts WebSocket commands to MQTT messages:
- **Topic**: `robot/cmd`
- **Format**: `{"left": -255..255, "right": -255..255}`

---

## 📚 Documentation

| Document | Description |
|----------|-------------|
| [ARCHITECTURE.md](ARCHITECTURE.md) | **System architecture, deployment options, ESP32 connection** |
| [hand-gesture-robot/README.md](hand-gesture-robot/README.md) | Robot hardware setup |
| [hand-gesture-robot/SETUP_GUIDE.md](hand-gesture-robot/SETUP_GUIDE.md) | Complete setup guide |
| [hand-gesture-robot/firmware/README.md](hand-gesture-robot/firmware/README.md) | ESP32/Arduino firmware |

---

## 🤝 Authors

- **Esin Eltimür**
- **Hasan Çoban**
- **Lilian L'helgoualc'h**

CENG483 Behavioral Robotics - Fall 2025

---

## 📄 License

MIT License - see [LICENSE](LICENSE) for details.
