# System Architecture Guide

This document explains the complete architecture of the Hand Gesture Robot control system, including deployment options and how all components connect.

---

## Table of Contents

1. [Overview](#overview)
2. [Architecture Diagram](#architecture-diagram)
3. [Components](#components)
4. [Data Flow](#data-flow)
5. [Deployment Options](#deployment-options)
6. [ESP32 Connection](#esp32-connection)
7. [What is micro-ROS?](#what-is-micro-ros)
8. [FAQ](#faq)

---

## Overview

The system uses a **client-server architecture** where:

- **Client** (your laptop/phone): Captures camera, runs MediaPipe, computes gestures, sends JSON commands
- **Server** (Ubuntu VM or cloud): Receives commands, publishes to ROS2 topics, bridges to MQTT for ESP32

This separation solves a critical problem: **corrupted RTSP frames must NOT produce robot commands**. By processing video locally on the client, we can validate frames before computing any controls.

---

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│  CLIENT (macOS / Windows / Linux laptop)                                         │
│  ┌───────────────────────────────────────────────────────────────────────────┐  │
│  │  client_hand_control/                                                      │  │
│  │                                                                            │  │
│  │   ┌─────────┐      ┌─────────────┐      ┌───────────────┐                 │  │
│  │   │ Camera  │  ──▶ │ Frame Gate  │  ──▶ │ MediaPipe     │                 │  │
│  │   │ /RTSP   │      │ (validate)  │      │ Hands         │                 │  │
│  │   └─────────┘      └─────────────┘      └───────────────┘                 │  │
│  │                                                │                           │  │
│  │                                                ▼                           │  │
│  │   ┌─────────────────────────────────────────────────────────────────────┐ │  │
│  │   │  Hand Control Logic                                                  │ │  │
│  │   │  • Calibration (neutral position)                                    │ │  │
│  │   │  • Handlebar metrics (tilt, push, openness)                         │ │  │
│  │   │  • Compute linear/angular velocity                                   │ │  │
│  │   │  • DriveState machine (calibrating → active → hands_lost)           │ │  │
│  │   └─────────────────────────────────────────────────────────────────────┘ │  │
│  │                                                │                           │  │
│  │                                                ▼                           │  │
│  │   ┌─────────────┐      ┌─────────────┐                                    │  │
│  │   │ Message     │  ──▶ │ WebSocket   │  ────────────────┐                 │  │
│  │   │ Validator   │      │ Client      │                  │                 │  │
│  │   └─────────────┘      └─────────────┘                  │                 │  │
│  │                                                          │                 │  │
│  └──────────────────────────────────────────────────────────│─────────────────┘  │
│                                                              │                    │
└──────────────────────────────────────────────────────────────│────────────────────┘
                                                               │
                                                               │ WebSocket (ws:// or wss://)
                                                               │ JSON: {"linear": 0.1, "angular": 0.5,
                                                               │        "enable": true, "ts_ms": 12345}
                                                               ▼
┌─────────────────────────────────────────────────────────────────────────────────┐
│  SERVER (Ubuntu VM / AWS EC2 / Local)                                           │
│  ┌───────────────────────────────────────────────────────────────────────────┐  │
│  │  server_gateway/                                                           │  │
│  │                                                                            │  │
│  │   ┌─────────────────┐                                                     │  │
│  │   │  WebSocket       │  • Bearer token authentication                     │  │
│  │   │  Server          │  • Single-controller lock (first client wins)     │  │
│  │   │  (FastAPI)       │  • Message validation                              │  │
│  │   └─────────────────┘                                                     │  │
│  │           │                                                                │  │
│  │           ├──────────────────────┬────────────────────────────────────────┤  │
│  │           ▼                      ▼                                        │  │
│  │   ┌─────────────────┐    ┌─────────────────┐                              │  │
│  │   │  ROS Bridge     │    │  MQTT Bridge    │                              │  │
│  │   │  (rclpy node)   │    │  (paho-mqtt)    │                              │  │
│  │   │                 │    │                 │                              │  │
│  │   │  /cmd_vel       │    │  robot/cmd      │                              │  │
│  │   │  /drive_enabled │    │  robot/telemetry│                              │  │
│  │   │                 │    │                 │                              │  │
│  │   │  Deadman: 300ms │    │                 │                              │  │
│  │   └─────────────────┘    └─────────────────┘                              │  │
│  │           │                      │                                        │  │
│  └───────────│──────────────────────│────────────────────────────────────────┘  │
│              │                      │                                            │
└──────────────│──────────────────────│────────────────────────────────────────────┘
               │                      │
               ▼                      ▼
        ┌─────────────┐        ┌─────────────┐
        │ ROS2        │        │ MQTT        │
        │ Robot       │        │ Broker      │
        │ (optional)  │        │ (mosquitto) │
        └─────────────┘        └─────────────┘
                                     │
                                     ▼
                               ┌─────────────┐
                               │   ESP32     │
                               │   Master    │
                               │             │
                               │ WiFi+MQTT   │
                               └─────────────┘
                                     │
                                     │ I2C
                                     ▼
                               ┌─────────────┐
                               │   Arduino   │
                               │   Slave     │
                               │             │
                               │ PWM Motors  │
                               └─────────────┘
```

---

## Components

### 1. Client (`client_hand_control/`)

**Purpose**: Captures video, detects hands, computes controls, sends JSON commands.

| File | Purpose |
|------|---------|
| `main.py` | CLI entry point, main control loop |
| `frame_gate.py` | Validates frames (detects corruption) |
| `hand_control.py` | Handlebar gesture logic |
| `message.py` | JSON schema + validation |
| `ws_client.py` | WebSocket with reconnect |

**Key Features**:
- **NO ROS2 dependency** - runs on any OS
- Frame quality gate (drops corrupted RTSP frames)
- Hand quality gate (both hands required)
- Message validation (rejects NaN/Inf)

### 2. Server (`server_gateway/`)

**Purpose**: Receives JSON commands, publishes to ROS2 and MQTT.

| File | Purpose |
|------|---------|
| `main.py` | Entry point, orchestrates bridges |
| `ws_server.py` | FastAPI WebSocket endpoint |
| `ros_bridge.py` | ROS2 publisher node |
| `mqtt_bridge.py` | MQTT publisher for ESP32 |

**Key Features**:
- Bearer token authentication
- Single-controller lock
- Deadman timer (300ms)
- Dual output: ROS2 + MQTT

### 3. ESP32 Firmware (`hand-gesture-robot/firmware/`)

**Purpose**: Receives MQTT commands, drives motors via Arduino.

| File | Purpose |
|------|---------|
| `esp32_master/esp32_master.ino` | WiFi + MQTT + I2C master |
| `arduino_slave/arduino_slave.ino` | PWM motor control |

---

## Data Flow

### Normal Operation

```
1. Camera captures frame (30 fps)
2. Frame Gate validates:
   - Read success?
   - Not empty?
   - Correct shape?
   - No corruption?
3. MediaPipe detects hands
4. Hand Control computes:
   - Are both hands visible?
   - Calibration complete?
   - Compute tilt, push, openness
   - Calculate linear/angular velocity
5. Message created and validated:
   - Values finite?
   - Within bounds?
   - Timestamp monotonic?
6. WebSocket sends to server
7. Server authenticates and validates
8. ROS Bridge publishes /cmd_vel, /drive_enabled
9. MQTT Bridge publishes robot/cmd
10. ESP32 receives, sends I2C to Arduino
11. Arduino drives motors
```

### Safety Timeout Flow

```
Frame invalid for >300ms
         │
         ▼
   Force STOP message sent
         │
         ▼
   Server receives enable=false
         │
         ▼
   Motors stop
```

---

## Deployment Options

### Option 1: All Local (Development)

```
┌──────────────────────────────────────────────────┐
│  Same Machine (Ubuntu VM via UTM/Parallels)      │
│                                                  │
│  Terminal 1: server_gateway                      │
│  Terminal 2: client_hand_control                 │
│  Terminal 3: ros2 topic echo                     │
│                                                  │
│  Server: ws://127.0.0.1:8080/control             │
└──────────────────────────────────────────────────┘
```

**Pros**: Simple, no network config needed
**Cons**: VM must have camera passthrough

### Option 2: Client on macOS, Server on VM (Current Setup)

```
┌────────────────────┐     ┌────────────────────┐
│  macOS (Client)    │     │  Ubuntu VM         │
│                    │     │  (Server + ROS2)   │
│  python -m         │────▶│                    │
│  client_hand_      │     │  192.168.64.3:8080 │
│  control.main      │     │                    │
└────────────────────┘     └────────────────────┘
```

**Network**: Client uses `ws://192.168.64.3:8080/control`

### Option 3: Cloud Server (Production)

```
┌────────────────────┐     ┌────────────────────┐     ┌────────────────────┐
│  Laptop (Client)   │     │  AWS EC2           │     │  Robot (ESP32)     │
│                    │     │  (Server + ROS2)   │     │                    │
│  Anywhere on       │────▶│                    │◀────│  WiFi to Internet  │
│  internet          │     │  Public IP         │     │                    │
└────────────────────┘     └────────────────────┘     └────────────────────┘
```

**Pros**: 
- Client and robot can be anywhere
- No NAT/port forwarding needed
- ESP32 connects to public MQTT

**Setup**:
```bash
# On AWS EC2 (Ubuntu)
sudo apt install mosquitto mosquitto-clients
mosquitto -v &

# Start server
export CONTROL_TOKEN=your_secret_token
python -m server_gateway.main --mqtt-host localhost
```

**ESP32 Config**:
```cpp
const char* mqtt_server = "your-ec2-ip";
const int mqtt_port = 1883;
```

---

## ESP32 Connection

The ESP32 connects to the server via **MQTT** (not ROS2 directly). This is simpler and more reliable than micro-ROS for most use cases.

### Connection Flow

```
1. ESP32 boots, connects to WiFi
2. ESP32 connects to MQTT broker (on server)
3. ESP32 subscribes to "robot/cmd"
4. Server receives WebSocket command from client
5. Server publishes to "robot/cmd" via MQTT bridge
6. ESP32 receives command: {"left": 100, "right": 100}
7. ESP32 sends I2C command to Arduino
8. Arduino drives motors
```

### MQTT Topics

| Topic | Direction | Format |
|-------|-----------|--------|
| `robot/cmd` | Server → ESP32 | `{"left": -255..255, "right": -255..255}` |
| `robot/telemetry` | ESP32 → Server | `{"battery": 12.1, "status": "ok"}` |

### ESP32 Firmware Changes

Update your ESP32 firmware to connect to the correct broker:

```cpp
// esp32_master.ino

// For local testing (VM)
const char* mqtt_server = "192.168.64.3";

// For cloud deployment (AWS EC2)
const char* mqtt_server = "your-ec2-public-ip";

// For same network
const char* mqtt_server = "10.8.34.100";  // Server IP on same WiFi
```

---

## What is micro-ROS?

**micro-ROS** is a way to run ROS2 directly on microcontrollers (like ESP32). It allows the ESP32 to be a full ROS2 node, publishing and subscribing to topics.

### Do We Need micro-ROS?

**No, not for this project.** Here's why:

| Approach | Pros | Cons |
|----------|------|------|
| **MQTT (current)** | Simple, lightweight, works anywhere | Not "pure" ROS2 |
| **micro-ROS** | Full ROS2 on ESP32, native topics | Complex setup, memory-heavy, requires special agent |

### When to Use micro-ROS

Use micro-ROS if:
- You need the ESP32 to publish/subscribe to ROS2 topics directly
- You're using ROS2 tools (rviz, rosbag) to analyze ESP32 data
- Your robot is fully ROS2-based with many nodes

### When to Use MQTT (Recommended)

Use MQTT if:
- You just need to send commands to ESP32
- You want simple, reliable communication
- ESP32 doesn't need to interact with other ROS2 nodes

**Our architecture uses MQTT because**:
1. The ESP32 only receives motor commands
2. Telemetry can flow through MQTT just as easily
3. No micro-ROS agent setup required
4. Works over the internet (micro-ROS is typically LAN-only)

---

## FAQ

### Q: Why not stream video through ROS2?

Video streaming through ROS2 (image_transport) adds latency and complexity. More importantly, if the RTSP stream has H264 decoder errors, we need to detect and filter those frames **before** they enter the ROS2 pipeline. Processing locally on the client ensures bad frames never produce commands.

### Q: Can I run the client on my phone?

Not directly (Python/MediaPipe required), but you could:
1. Run the client on a laptop
2. Use a phone as an IP camera (RTSP stream)
3. Point the client at the phone's RTSP URL

### Q: How do I add TLS/WSS?

For production, use a reverse proxy like Caddy or Nginx:

```
# Caddyfile
robot.example.com {
    reverse_proxy localhost:8080
}
```

Client connects to `wss://robot.example.com/control`.

### Q: What if the client disconnects?

The server's deadman timer kicks in after 300ms:
1. No messages received for 300ms
2. Server publishes `enable=false` and zero velocity
3. Motors stop

### Q: Can multiple clients connect?

Yes, but only one can control at a time. The first client to send `enable=true` becomes the controller. Other clients are rejected until the controller disconnects.

### Q: How do I test without hardware?

```bash
# Terminal 1: Server
export CONTROL_TOKEN=test123
python -m server_gateway.main

# Terminal 2: Client
python -m client_hand_control.main \
    --server ws://127.0.0.1:8080/control \
    --token test123 \
    --camera 0 \
    --preview

# Terminal 3: Monitor ROS2
ros2 topic echo /cmd_vel

# Terminal 4: Monitor MQTT
mosquitto_sub -t "robot/cmd" -v
```

---

## Quick Reference

### Client Commands

```bash
# Local webcam
python -m client_hand_control.main \
    --server ws://SERVER_IP:8080/control \
    --token YOUR_TOKEN \
    --camera 0 \
    --preview

# RTSP stream
python -m client_hand_control.main \
    --server ws://SERVER_IP:8080/control \
    --token YOUR_TOKEN \
    --rtsp "rtsp://CAMERA_IP:8554/stream" \
    --preview
```

### Server Commands

```bash
# Start server
export CONTROL_TOKEN=your_secret_token
python -m server_gateway.main

# With custom MQTT broker
python -m server_gateway.main --mqtt-host 10.8.34.100 --mqtt-port 1883
```

### Debug Commands

```bash
# Monitor ROS2 topics
ros2 topic echo /cmd_vel
ros2 topic echo /drive_enabled

# Monitor MQTT
mosquitto_sub -t "robot/#" -v

# Test MQTT manually
mosquitto_pub -t "robot/cmd" -m '{"left": 100, "right": 100}'
```

---

## Changelog

| Date | Change |
|------|--------|
| 2026-01-10 | Initial architecture documentation |
| 2026-01-10 | Added micro-ROS comparison |
| 2026-01-10 | Added deployment options |

