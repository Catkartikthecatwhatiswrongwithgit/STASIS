<div align="center">

```
███████╗ ████████╗  █████╗  ███████╗ ██╗ ███████╗
██╔════╝ ╚══██╔══╝ ██╔══██╗ ██╔════╝ ██║ ██╔════╝
███████╗    ██║    ███████║ ███████╗ ██║ ███████╗
╚════██║    ██║    ██╔══██║ ╚════██║ ██║ ╚════██║
███████║    ██║    ██║  ██║ ███████║ ██║ ███████║
╚══════╝    ╚═╝    ╚═╝  ╚═╝ ╚══════╝ ╚═╝ ╚══════╝
```

**Autonomous Hazard Monitoring Rover**
*Team EDGECASE*

[![Platform](https://img.shields.io/badge/Platform-ESP32--S3-red?style=flat-square)](https://www.espressif.com/)
[![Firmware](https://img.shields.io/badge/Firmware-Arduino%20C%2B%2B-blue?style=flat-square)](https://www.arduino.cc/)
[![Base Station](https://img.shields.io/badge/Base%20Station-Python%203-green?style=flat-square)](https://python.org)
[![Status](https://img.shields.io/badge/Status-Demo%20Ready-brightgreen?style=flat-square)]()

</div>

---

## What is STASIS?

STASIS is a compact autonomous rover built for reliable real-world operation on a student budget. It patrols a defined area, detects hazards (fire, motion, seismic activity), sends SMS alerts, and returns to a charging dock using AprilTag visual guidance — all without human intervention.

It is designed to **work at a demo**, not just in a lab.

---

## System Architecture

```
┌───────────────────────────────────────────────────────────────┐
│                      ROVER  (Mobile)                          │
│                                                               │
│   ┌─────────────┐  UART   ┌────────────────────────────────┐  │
│   │  ESP32-CAM  │────────►│          ESP32-S3              │  │
│   │  (Vision)   │         │  State machine · Motors        │  │
│   └─────────────┘         │  Sensors · GPS · GSM · UART    │  │
│                           └──────────────┬─────────────────┘  │
│                                          │ ESP-NOW             │
└──────────────────────────────────────────┼─────────────────────┘
                                           │
┌──────────────────────────────────────────┼─────────────────────┐
│                   BASE STATION           │  (Static)           │
│                                          ▼                     │
│   ┌──────────────────┐  UART  ┌──────────────────┐            │
│   │   Raspberry Pi   │◄──────►│    ESP32-C3       │            │
│   │   Zero W         │        │  (Wireless bridge)│            │
│   │   Flask + SocketIO        └──────────────────┘            │
│   └────────┬─────────┘                                         │
│            │ HTTP + Socket.IO                                  │
└────────────┼──────────────────────────────────────────────────┘
             ▼
    ┌──────────────────────────┐
    │     Web Dashboard        │
    │  stasis_app/index.html   │   offline-safe, served from Pi
    └──────────────────────────┘
```

---

## Hardware

### Rover

| Component | Part | Role |
|-----------|------|------|
| Main MCU | ESP32-S3 | State machine, motor control, sensor fusion |
| Vision | ESP32-CAM (AI Thinker) | Fire / motion / human detection |
| Motor Driver | L9110S | 4WD differential drive |
| Obstacle | HC-SR04 | Non-blocking ultrasonic — stops at 25 cm |
| Temperature | DS18B20 | Environmental monitoring, fire threshold |
| Light | LDR | Ambient light / smoke indicator |
| GPS | NEO-6M | Geofence enforcement + home position |
| GSM | SIM800L | SMS alerts on hazard or geofence breach |
| Battery | 2S Li-ion 7.4 V | Monitored via ADC + 30k/10k divider |

### Base Station

| Component | Part | Role |
|-----------|------|------|
| Supervisor | Raspberry Pi Zero W | Flask API, AprilTag docking, PDF reports |
| Bridge | ESP32-C3 | ESP-NOW ↔ UART relay |
| Tag | tag36h11 ID 0, 10 cm | Visual beacon on docking station |

---

## State Machine

```
             ┌──────────────┐
   ┌─────────│  SAFE_IDLE   │◄──────────────────────┐
   │         └──────┬───────┘                       │
   │                │ PATROL or cmd                 │
   │                ▼                               │
   │         ┌──────────────┐  geofence/hazard      │
   │         │   PATROL     │────────────┐          │
   │         └──────┬───────┘            │          │
   │                │ hazard             │          │
   │                ▼                   │          │
   │         ┌──────────────┐           │          │
   │         │    ALERT     │─────timeout 30s──────►│
   │         └──────────────┘                      │
   │                                               │
   │  low bat / geofence                           │
   │         ┌──────────────┐                      │
   └────────►│ RETURN_BASE  │                      │
             └──────┬───────┘                      │
                    │ dockingEnabled                │
                    ▼                              │
             ┌──────────────┐  voltage rise        │
             │   DOCKING    │─────────────────────►│
             │ (AprilTag)   │                      │
             └──────────────┘                      │
                    │ docked                       │
                    ▼                              │
             ┌────────────────────┐                │
             │  DOCKED_CHARGING   │                │
             │  motors disabled   │────undocked───►│
             └────────────────────┘
```

| State | Description |
|-------|-------------|
| `SAFE_IDLE` | Motors off, sensors active, waiting for command |
| `PATROL` | Autonomous patrol within GPS geofence |
| `ALERT` | Hazard confirmed — buzzer on, SMS sent, investigating |
| `RETURN_TO_BASE` | Low battery or geofence breach — heading home |
| `DOCKING` | Pi-guided AprilTag precision approach |
| `DOCKED_CHARGING` | Passive contact charging — motors fully disabled |

---

## AprilTag Docking

The Pi Zero runs a continuous visual docking control loop whenever the rover enters `STATE_DOCKING`.

```
ESP32-CAM MJPEG stream
        │
        ▼
  Pi Zero  (pupil-apriltags + OpenCV)
        │  detect tag36h11 ID 0
        │  measure: X error (steering) + Z depth (distance)
        ▼
  Proportional controller
    turn  = clamp(err_x × 0.6,  -100, +100)
    speed = clamp(dist_m × 40,    10,    40)
    if |err_x| < 15 px → turn = 0  (dead-band)
    if dist_m  < 0.25 m → DOCKED
        │
        ▼
  6-byte binary packet   [0xAA | turn | speed | docked | XOR | 0x55]
        │
        ▼
  UART → ESP32-C3 → ESP-NOW → ESP32-S3 → motors
```

**One-time setup on Pi:**
```bash
pip install pupil-apriltags opencv-python-headless
```

Print a `tag36h11` tag at **10 cm**, ID **0**. Mount it vertically on the docking station, facing the rover's approach path. Ensure there is consistent lighting.

---

## Quick Start

### 1. Flash firmware

```bash
# ESP32-S3 (rover main controller)
# Open: rover_main/rover_main.ino
# Board: ESP32S3 Dev Module  |  Upload speed: 921600

# ESP32-CAM (vision)
# Open: cam_firmware/cam_firmware.ino
# Board: AI Thinker ESP32-CAM  |  Connect IO0→GND during flash only

# ESP32-C3 (wireless bridge)
# Open: base_bridge/base_bridge.ino
# Board: ESP32C3 Dev Module
# ⚠  Set roverMac[] to your ESP32-S3's actual MAC address before flashing
```

### 2. Set up the Pi

```bash
# Install Python dependencies
pip install flask flask-cors flask-socketio pyserial fpdf2 eventlet
pip install pupil-apriltags opencv-python-headless

# Cache Leaflet maps for offline use (run once with internet access)
mkdir -p ~/stasis/static
wget -O ~/stasis/static/leaflet.css https://unpkg.com/leaflet@1.9.4/dist/leaflet.css
wget -O ~/stasis/static/leaflet.js  https://unpkg.com/leaflet@1.9.4/dist/leaflet.js

# Start the base station
python3 base_station/station_monitor.py
```

### 3. Auto-start on boot

```bash
sudo nano /etc/systemd/system/stasis.service
```

```ini
[Unit]
Description=STASIS Base Station
After=network.target

[Service]
User=pi
WorkingDirectory=/home/pi/stasis
ExecStart=/usr/bin/python3 /home/pi/stasis/base_station/station_monitor.py
Restart=always
RestartSec=10
Environment="SERIAL_PORT=/dev/serial0"
Environment="CAM_STREAM_URL=http://192.168.4.1:81/stream"

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl daemon-reload && sudo systemctl enable --now stasis
```

### 4. Open the dashboard

Connect to the `Stasis-Base` WiFi AP (password: `stasis123`), then visit:

```
http://192.168.4.1:5000/stasis_app/index.html
```

---

## Commands

| Command | Action |
|---------|--------|
| `STOP` | Emergency stop → SAFE_IDLE |
| `FORWARD` | Drive forward |
| `BACKWARD` | Drive in reverse |
| `LEFT` / `RIGHT` | Turn |
| `PATROL` | Start autonomous patrol |
| `STOP_PATROL` | Stop patrol → SAFE_IDLE |
| `HOME` / `RETURN_BASE` | Return to dock |
| `DOCKING` | Enable AprilTag docking |
| `DISABLE_DOCKING` | Cancel docking |
| `SETHOME` | Save current GPS position as home |
| `RESET` | Reset all flags and states |

---

## Safety Features

| Feature | How it works |
|---------|-------------|
| Obstacle stop | HC-SR04 non-blocking; halts at < 25 cm |
| GPS geofence | 50 m radius, Haversine calculation, 80 % hysteresis on re-entry |
| GPS staleness guard | `SAFE_HALT` if no fix > 5 s; 90 s grace period after cold boot |
| Dock detection | 0.4 V battery rise sustained across 20 frames (~2 s) |
| Emergency stop | `ledcWrite(pin, 0)` on all PWM channels — LEDC-safe |
| GSM brownout guard | Motors halted 100 ms before every SMS attempt |
| SMS rate limit | Maximum one SMS per 60 seconds |
| Low battery | Speed capped at 60 % below 20 %; `RETURN_TO_BASE` triggered |
| Hard voltage cutoff | Forced `SAFE_IDLE` at 6.2 V |
| Charge complete | Detected at 8.4 V or after 10-minute timeout |

---

## Repository Structure

```
STASIS/
├── rover_main/
│   └── rover_main.ino           ESP32-S3 main firmware
├── cam_firmware/
│   └── cam_firmware.ino         ESP32-CAM vision firmware
├── base_bridge/
│   └── base_bridge.ino          ESP32-C3 wireless bridge
├── base_station/
│   └── station_monitor.py       Pi: Flask API + AprilTag docking thread
├── stasis_app/
│   └── index.html               Web dashboard (offline-safe single file)
├── docking/
│   └── apriltag_docking.py      Standalone AprilTag docking utility
└── docs/
    ├── ESP32-S3_INSTRUCTIONS.md
    ├── ESP32-CAM_INSTRUCTIONS.md
    ├── ESP32-C3_INSTRUCTIONS.md
    ├── RPI0_INSTRUCTIONS.md
    ├── APRILTAG_DOCKING_GUIDE.md
    ├── WIRING_DIAGRAMS.md
    ├── PROTOCOL.md
    ├── QUICK_REFERENCE.md
    ├── SSH_PI0_SETUP.md
    └── 3D_PRINTED_PARTS.md
```

---

## Pin Reference (ESP32-S3)

| GPIO | Function | Notes |
|------|----------|-------|
| 1 | Battery ADC | 30 kΩ / 10 kΩ divider → reads 0–3.3 V for 0–13.2 V |
| 2 | Motor A DIR | L9110S |
| 4 | Motor A PWM | L9110S — LEDC channel |
| 5 | Motor B PWM | L9110S — LEDC channel |
| 6 | Motor C PWM | L9110S — LEDC channel |
| 7 | Motor C DIR | L9110S |
| 8 | GPS Serial2 RX | NEO-6M TX |
| 9 | Ultrasonic ECHO | HC-SR04 |
| 10 | Ultrasonic TRIG | HC-SR04 |
| 12 | SIM800 RX | **Serial1** — not Serial2 |
| 13 | SIM800 TX | **Serial1** — not Serial2 |
| 14 | Buzzer | Active buzzer |
| 15 | Motor D PWM | L9110S — LEDC channel |
| 16 | Motor D DIR | L9110S |
| 18 | Motor B DIR | L9110S |
| 19 | GPS Serial2 TX | NEO-6M RX |
| 21 | DS18B20 | OneWire + 4.7 kΩ pull-up to 3.3 V |
| 43 | UART1 TX | Pi Zero RX (via ESP32-C3) |
| 44 | UART1 RX | Pi Zero TX (via ESP32-C3) |
| 45 | Status LED | – |

---

## Changelog — v2 (March 2026)

| # | Issue | Fix |
|---|-------|-----|
| C1 | `estop()` used `digitalWrite` on LEDC pins — motors did not stop | Changed to `ledcWrite(pin, 0)` |
| C2 | `BACKWARD` command called `drive(0,0)` immediately, cancelling the speed | Fixed to set `currentSpeed = -50` |
| C3 | `chargeBeepDone` declared as local `static` inside `executeState()`, shadowing global | Removed local declaration; uses global |
| C4 | GPS and SIM800 both on Serial2 — corrupted each other | SIM800 moved to Serial1 (pins 12, 13) |
| C5 | Global variables used but never declared — code would not compile | Added full globals block after enum declarations |
| C6 | AprilTag docking existed in spec but was not implemented anywhere | Full Pi-side docking thread added to `station_monitor.py` |
| C7 | `uartTask` only called `parseDockCommand` on the header byte; bytes 2–6 routed to text parser | Added `binaryMode` flag to capture entire packet |
| I1 | `captureAndSend()` wrote raw binary struct then text on same Serial — corrupted S3 parser | Binary write removed; text-only output |
| I2 | GPS staleness check halted rover immediately on cold boot before any fix acquired | Added 90-second startup grace period |
| I3 | FreeRTOS task stacks set to 2048 bytes — overflowed on String / float operations | Increased to 3072–4096 bytes per task |
| W1 | `L is not defined` (×6) — Leaflet CDN unavailable on isolated AP network | Dynamic loader: CDN → `/static/leaflet.js` → offline message |
| W2 | Chat used raw `WebSocket` — Flask-SocketIO uses Socket.IO framing, not raw WS | Replaced with Socket.IO `io()` client |
| W3 | Font Awesome CDN missing on offline AP — all icons blank | Replaced with inline SVG, zero CDN dependency |

---

*Team EDGECASE · March 2026*
