# STASIS - Autonomous Forest Monitoring Rover System

## Overview

STASIS is an autonomous forest/environmental monitoring rover system designed to:
- Patrol forest environments autonomously
- Detect environmental hazards (fire, earthquakes, intruders)
- Send real-time alerts to forest officers via SMS
- Stream video when near the charging station
- Generate daily mission reports

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        ROVER UNIT                               │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐      │
│  │  ESP32-S3    │◄──►│  ESP32-CAM   │    │   SIM800     │      │
│  │  (Main Ctrl) │    │  (Vision)    │    │   (GSM)      │      │
│  └──────┬───────┘    └──────────────┘    └──────────────┘      │
│         │                                                       │
│  ┌──────▼───────┐    ┌──────────────┐    ┌──────────────┐      │
│  │   Sensors    │    │    Motors    │    │     GPS      │      │
│  │ Temp/Ultra/  │    │  4WD L9110S  │    │   Neo-6M     │      │
│  │   MPU6050    │    │              │    │              │      │
│  └──────────────┘    └──────────────┘    └──────────────┘      │
└─────────────────────────────────────────────────────────────────┘
                              │
                              │ ESP-NOW (Wireless)
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                      BASE STATION                               │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐      │
│  │  ESP32-C3    │◄──►│ Raspberry Pi │    │  LCD Display │      │
│  │   (Bridge)   │    │    Zero      │    │    (I2C)     │      │
│  └──────────────┘    └──────┬───────┘    └──────────────┘      │
│                             │                                   │
│                      ┌──────▼───────┐                          │
│                      │ Flask API    │                          │
│                      │ SQLite DB    │                          │
│                      │ Reports      │                          │
│                      └──────────────┘                          │
└─────────────────────────────────────────────────────────────────┘
                              │
                              │ HTTP/HTTPS (Internet)
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                      CLIENT APPS                                │
│  ┌──────────────┐    ┌──────────────┐                          │
│  │  Web Dashboard│    │  Mobile App  │                          │
│  │  (index.html) │    │  (Capacitor) │                          │
│  └──────────────┘    └──────────────┘                          │
└─────────────────────────────────────────────────────────────────┘
```

## Hardware Components

### Rover Unit
| Component | Model | Purpose |
|-----------|-------|---------|
| Main Controller | ESP32-S3 | Navigation, sensors, logic |
| Vision | ESP32-CAM | Fire/motion/human detection |
| Motors | 4x DC Motors + L9110S | 4WD propulsion |
| GSM | SIM800L | SMS alerts to forest officers |
| GPS | Neo-6M | Position tracking |
| Temperature | DS18B20 | Fire detection |
| Distance | HC-SR04 Ultrasonic | Obstacle avoidance |
| IMU | MPU6050 | Earthquake detection |
| Power | LiPo Battery | Power supply |

### Base Station
| Component | Model | Purpose |
|-----------|-------|---------|
| Bridge | ESP32-C3 | ESP-NOW to UART relay |
| Computer | Raspberry Pi Zero | Data processing, API |
| Display | LCD 16x2 I2C | Status display |

## Software Components

### 1. Rover Firmware (`rover_main/rover_main.ino`)

**State Machine:**
- `PATROL` - Normal autonomous patrol
- `RESEARCH` - Investigating point of interest
- `ALERT` - Hazard detected, sending notifications
- `RETURN_BASE` - Low battery, returning to charge
- `DOCKED` - Charging at base station

**Features:**
- Obstacle avoidance using ultrasonic sensor
- Fire detection via temperature sensor
- Earthquake detection via MPU6050
- GPS position tracking
- Manual override via ESP-NOW commands

### 2. Camera Firmware (`cam_firmware/cam_firmware.ino`)

**Detection Capabilities:**
- Fire detection (color analysis)
- Motion detection (frame differencing)
- Human detection (simplified pattern)

**Output:**
- `HAZARD:FIRE` - Fire detected
- `HAZARD:MOTION` - Motion detected
- `CLEAR` - No hazards

### 3. Base Bridge Firmware (`base_bridge/base_bridge.ino`)

**Communication:**
- Receives ESP-NOW from rover
- Converts to JSON for Raspberry Pi
- Relays commands from Pi to rover

### 4. Base Station Monitor (`base_station/station_monitor.py`)

**Features:**
- Receives telemetry from ESP32-C3 bridge via UART
- Parses both JSON telemetry and non-JSON camera messages (HAZARD:FIRE, HAZARD:HUMAN, etc.)
- Stores all telemetry in SQLite database
- Generates PDF/text daily mission reports
- Sends commands to rover (FORWARD, BACKWARD, LEFT, RIGHT, STOP, PATROL, HOME, DOCKING, etc.)
- WebSocket support for real-time chat and telemetry
- Configurable SMS recipients for alerts
- API key authentication

**API Endpoints:**
| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/status` | GET | Latest telemetry |
| `/api/history` | GET | Telemetry history |
| `/api/stats` | GET | Daily statistics |
| `/api/alerts` | GET | Alert history |
| `/api/command` | POST | Send command to rover |
| `/api/reports` | GET | List generated reports |
| `/api/health` | GET | System health check |
| `/api/config` | GET/POST | Configuration |
| `/ws` | WS | Real-time chat/telemetry |

### 5. Web Dashboard (`stasis_app/index.html`)

**Features:**
- Dark blue GitHub-style UI
- Dashboard with rover status, battery, temperature, distance
- Mini map with rover position
- Command controls (Stop, Patrol, Home)
- Full-screen map with Leaflet
- Chat system with channels (general, alerts, patrol)
- Geofencing controls for patrol area
- Login system (optional, saves to localStorage)
- Real-time simulation mode when Pi not connected

## Installation

### Prerequisites

1. **Arduino IDE** with ESP32 board support
2. **Python 3** with Flask
3. **PlatformIO** (optional, for building)

### Rover Setup

1. Install required Arduino libraries:
   - Adafruit MPU6050
   - TinyGPSPlus
   - OneWire
   - DallasTemperature
   - ESP32 ESP-NOW

2. Configure pins in `rover_main.ino`:
   ```cpp
   #define PIN_MOTOR_L_SPEED 4
   #define PIN_MOTOR_L_DIR   5
   // ... other pins
   ```

3. Set base station MAC address:
   ```cpp
   uint8_t baseMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
   ```

4. Upload to ESP32-S3

### Camera Setup

1. Select AI Thinker ESP32-CAM board
2. Upload `cam_firmware.ino`
3. Connect to ESP32-S3 via UART

### Base Station Setup

1. Install Python dependencies:
   ```bash
   pip install flask flask-cors flask-socketio pyserial fpdf2 eventlet
   ```

2. Run the monitor:
   ```bash
   cd base_station
   python3 station_monitor.py
   ```

3. Access dashboard at `http://<pi-ip>:5000/stasis_app/index.html`

### Web Dashboard

The dashboard is a single HTML file with no build step required:
- `stasis_app/index.html` - Open directly in browser or serve via Flask
- No Node.js required - pure HTML/CSS/JavaScript
- Works offline with simulated data when Pi not connected

## Configuration

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `SERIAL_PORT` | `/dev/serial0` | UART port for ESP32-C3 |
| `BAUD_RATE` | `115200` | Serial baud rate |
| `FLASK_HOST` | `0.0.0.0` | API host |
| `FLASK_PORT` | `5000` | API port |
| `DATABASE_PATH` | `rover_telemetry.db` | SQLite database file |

### Dashboard Settings

Access the settings panel (gear icon) to configure:
- Base station IP address
- API port
- Refresh rate

## Troubleshooting

### Common Issues

1. **Rover not connecting to base**
   - Check ESP-NOW MAC address configuration
   - Verify both devices are on same WiFi channel

2. **Camera not working**
   - Check UART connections (TX/RX)
   - Verify camera module is properly seated

3. **GPS not getting fix**
   - Ensure antenna has clear sky view
   - Wait 2-5 minutes for cold start

4. **SMS not sending**
   - Check SIM card balance
   - Verify GSM module power (needs 2A peak)

## Safety Notes

1. **Battery Monitoring**: Rover returns to base at 30% battery
2. **Obstacle Avoidance**: Stops and turns when obstacle < 30cm
3. **Fire Detection**: Stops immediately, sounds buzzer, sends SMS
4. **Earthquake Detection**: Triggers alert on high acceleration

## License

This project is for educational and research purposes.

## Team EdgeCase

### Main Team Members
- **Kartik** - Leader and Initiative
- **Harish** - Team Member
- **Ryan Ahmed** - Team Member
- **Mohaideen Ijaz** - Team Member
- **Swethen** - Team Member

### Supporters
- **Stephen** - Team Member
- **Krish** - Team Member
- **Inba** - Team Member
- **Baavasri** - Team Member
- **Janane** - Team Member

---

*Last updated: 2026-02-28*
