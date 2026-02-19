# Aero Sentinel - Autonomous Environmental Monitoring Rover

A complete system for autonomous forest/environmental monitoring with hazard detection and base station reporting.

## System Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         AERO SENTINEL SYSTEM                                 │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│    ROVER UNIT                          BASE STATION                          │
│    ┌──────────────────┐               ┌──────────────────┐                  │
│    │                  │               │                  │                  │
│    │  ┌────────────┐  │    ESP-NOW    │  ┌────────────┐  │                  │
│    │  │ ESP32-S3   │  │   ◄───────►   │  │ ESP32-C3   │  │                  │
│    │  │ (Main)     │  │    Wireless   │  │ (Bridge)   │  │                  │
│    │  └────────────┘  │               │  └─────┬──────┘  │                  │
│    │                  │               │        │ UART    │                  │
│    │  ┌────────────┐  │               │        ▼         │                  │
│    │  │ ESP32-CAM  │  │               │  ┌────────────┐  │                  │
│    │  │ (Vision)   │  │               │  │ RPi Zero   │  │                  │
│    │  └────────────┘  │               │  │ (Monitor)  │  │                  │
│    │                  │               │  └────────────┘  │                  │
│    │  Sensors:        │               │                  │                  │
│    │  - GPS           │               │  Outputs:        │                  │
│    │  - IMU           │               │  - REST API      │                  │
│    │  - Temp          │               │  - PDF Reports   │                  │
│    │  - Ultrasonic    │               │  - LCD Display   │                  │
│    │  - GSM/SMS       │               │                  │                  │
│    │                  │               │                  │                  │
│    └──────────────────┘               └──────────────────┘                  │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Quick Start Guide

### Step 1: Flash the Firmware

Follow these instruction documents in order:

1. **ESP32-CAM** (Vision Module): [ESP32-CAM_INSTRUCTIONS.md](./ESP32-CAM_INSTRUCTIONS.md)
2. **ESP32-C3** (Base Station Bridge): [ESP32-C3_INSTRUCTIONS.md](./ESP32-C3_INSTRUCTIONS.md)
3. **ESP32-S3** (Rover Main): [ESP32-S3_INSTRUCTIONS.md](./ESP32-S3_INSTRUCTIONS.md)
4. **Raspberry Pi Zero** (Base Station): [RPI0_INSTRUCTIONS.md](./RPI0_INSTRUCTIONS.md)

### Step 2: Wire the Hardware

See complete wiring diagrams: [WIRING_DIAGRAMS.md](./WIRING_DIAGRAMS.md)

### Step 3: Configure MAC Addresses

1. Flash ESP32-C3 first and note its MAC address
2. Update `baseMac[]` in `rover_main.ino` with ESP32-C3's MAC
3. Re-flash ESP32-S3 with the correct MAC address

### Step 4: Test the System

1. Power on the base station (RPi Zero + ESP32-C3)
2. Power on the rover
3. Connect to WiFi AP: `AeroSentinel-Base` (password: `sentinel123`)
4. Open `http://192.168.4.1` to check ESP32-C3 status
5. Open `http://aerosentinel-base.local:5000/api/status` for rover data

---

## Documentation Index

| Document | Description |
|----------|-------------|
| [ESP32-S3_INSTRUCTIONS.md](./ESP32-S3_INSTRUCTIONS.md) | Flashing instructions for rover main controller |
| [ESP32-C3_INSTRUCTIONS.md](./ESP32-C3_INSTRUCTIONS.md) | Flashing instructions for base station bridge |
| [ESP32-CAM_INSTRUCTIONS.md](./ESP32-CAM_INSTRUCTIONS.md) | Flashing instructions for vision module |
| [RPI0_INSTRUCTIONS.md](./RPI0_INSTRUCTIONS.md) | Raspberry Pi Zero auto-boot setup |
| [WIRING_DIAGRAMS.md](./WIRING_DIAGRAMS.md) | Complete wiring diagrams and pinouts |

---

## Hardware Requirements

### Rover Unit

| Component | Quantity | Notes |
|-----------|----------|-------|
| ESP32-S3 DevKit | 1 | Main controller |
| ESP32-CAM | 1 | Vision module with OV2640 |
| L9110S Motor Driver | 1 | 2-channel H-bridge |
| DC Motors | 4 | 6V-12V, with wheels |
| Neo-6M GPS | 1 | With antenna |
| MPU6050 | 1 | IMU sensor |
| DS18B20 | 1 | Temperature sensor |
| HC-SR04 | 1 | Ultrasonic sensor |
| SIM800L | 1 | GSM module |
| Active Buzzer | 1 | 5V |
| LiPo Battery | 1 | 3S (11.1V-12.6V) |
| Buck Converter | 1 | 12V to 5V, 5A+ |

### Base Station

| Component | Quantity | Notes |
|-----------|----------|-------|
| Raspberry Pi Zero | 1 | Any version |
| ESP32-C3 DevKit | 1 | Communication bridge |
| MicroSD Card | 1 | 16GB+ |
| 5V Power Supply | 1 | 2.5A+ |
| LCD 16x2 I2C | 1 | Optional |

---

## Software Architecture

### Rover State Machine

```
┌─────────┐     Hazard Detected     ┌─────────┐
│ PATROL  │ ───────────────────────►│  ALERT  │
│         │                         │         │
│         │◄────────────────────────┤         │
└────┬────┘    After 3 seconds      └────┬────┘
     │                                    │
     │ Battery < 30%                      │
     │                                    │
     ▼                                    ▼
┌─────────────┐                    ┌──────────┐
│ RETURN_BASE │                    │ RESEARCH │
│             │                    │          │
│             │                    │          │
└──────┬──────┘                    └────┬─────┘
       │                                │
       │ Arrived at base                │
       │                                │
       ▼                                │
┌─────────┐                              │
│ DOCKED  │◄─────────────────────────────┘
│         │
│ Charging│
└─────────┘
```

### Communication Flow

```
┌──────────────┐                    ┌──────────────┐
│   ESP32-S3   │                    │   ESP32-C3   │
│   (Rover)    │                    │   (Bridge)   │
├──────────────┤                    ├──────────────┤
│              │    ESP-NOW         │              │
│ Telemetry ───┼───────────────────►│ JSON via UART│
│              │    (Binary)        │      │       │
│              │                    │      ▼       │
│              │    ESP-NOW         │   Serial1    │
│ Commands ◄───┼────────────────────│              │
│              │    (Text)          │              │
└──────────────┘                    └──────────────┘
                                           │
                                           ▼
                                    ┌──────────────┐
                                    │  RPi Zero    │
                                    │  (Monitor)   │
                                    ├──────────────┤
                                    │              │
                                    │ Flask API    │
                                    │ SQLite DB    │
                                    │ PDF Reports  │
                                    │              │
                                    └──────────────┘
```

---

## API Endpoints

The Raspberry Pi Zero serves a REST API on port 5000:

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/status` | GET | Latest telemetry |
| `/api/history` | GET | Last 200 entries |
| `/api/stats` | GET | Daily statistics |
| `/api/alerts` | GET | Alert history |
| `/api/command` | POST | Send command to rover |
| `/api/health` | GET | System health check |
| `/api/config` | GET/POST | Configuration |
| `/api/reports` | GET | List PDF reports |
| `/api/export/csv` | GET | Export as CSV |

### Example Usage

```bash
# Get latest status
curl http://aerosentinel-base.local:5000/api/status

# Send command to rover
curl -X POST http://aerosentinel-base.local:5000/api/command \
  -H "Content-Type: application/json" \
  -d '{"cmd": "FORWARD"}'

# Get daily stats
curl http://aerosentinel-base.local:5000/api/stats
```

---

## Detection Capabilities

### Fire Detection
- Temperature threshold (configurable, default 55°C)
- Visual detection via camera (color analysis)
- Immediate SMS alert on detection

### Earthquake Detection
- MPU6050 accelerometer monitoring
- Triggers on high acceleration (>15 m/s²)
- Logs event with timestamp

### Human Detection
- Camera-based pattern recognition
- Skin-tone color analysis
- Motion detection integration

### Obstacle Avoidance
- Ultrasonic distance measurement
- Automatic path correction
- Tree avoidance during fire detection

---

## Power Management

### Rover Power States

| State | Motor Speed | Sensor Rate | Notes |
|-------|-------------|-------------|-------|
| Normal | 100% | 500ms | Battery > 30% |
| Power Saving | 50% | 1000ms | Battery < 30% |
| Return Base | 50% | 500ms | Navigating to charger |
| Docked | Off | 1000ms | Charging |

### Battery Monitoring

- ADC reads battery voltage via voltage divider
- Percentage calculated from 10.5V-12.6V range
- Automatic return-to-base at 30%

---

## Troubleshooting

### Common Issues

| Problem | Solution |
|---------|----------|
| No telemetry received | Check ESP-NOW MAC addresses match |
| Camera not working | Re-seat ribbon cable, contacts down |
| GPS no fix | Wait 5 minutes, ensure sky view |
| SIM800 not responding | Check power supply (2A peak) |
| Motors not moving | Verify L9110S connections and power |
| Serial communication fails | Check TX/RX not swapped |

### LED Indicators

**ESP32-C3 (Base Station Bridge):**
- 3 blinks: Startup OK
- Continuous fast blink: ESP-NOW failed
- 1 blink: Data received
- 2 blinks: Command sent

**ESP32-CAM:**
- 3 blinks: Camera initializing
- Solid on: Camera ready
- Continuous blink: Camera error

---

## Project Structure

```
RoverProject/
├── rover_main/
│   └── rover_main.ino        # ESP32-S3 firmware
├── cam_firmware/
│   └── cam_firmware.ino      # ESP32-CAM firmware
├── base_bridge/
│   └── base_bridge.ino       # ESP32-C3 firmware
├── base_station/
│   └── station_monitor.py    # RPi Zero Python script
├── rover_dashboard/
│   └── (Web dashboard files)
└── docs/
    ├── README.md             # This file
    ├── ESP32-S3_INSTRUCTIONS.md
    ├── ESP32-C3_INSTRUCTIONS.md
    ├── ESP32-CAM_INSTRUCTIONS.md
    ├── RPI0_INSTRUCTIONS.md
    └── WIRING_DIAGRAMS.md
```

---

## License

This project is provided for educational and research purposes.

---

## Support

For issues and questions:
1. Check the troubleshooting section above
2. Review the individual instruction documents
3. Verify all wiring connections
4. Check serial monitor output for error messages

---

*Document Version: 1.0*  
*Last Updated: February 2026*

A complete system for autonomous forest/environmental monitoring with hazard detection and base station reporting.

## System Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         AERO SENTINEL SYSTEM                                 │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│    ROVER UNIT                          BASE STATION                          │
│    ┌──────────────────┐               ┌──────────────────┐                  │
│    │                  │               │                  │                  │
│    │  ┌────────────┐  │    ESP-NOW    │  ┌────────────┐  │                  │
│    │  │ ESP32-S3   │  │   ◄───────►   │  │ ESP32-C3   │  │                  │
│    │  │ (Main)     │  │    Wireless   │  │ (Bridge)   │  │                  │
│    │  └────────────┘  │               │  └─────┬──────┘  │                  │
│    │                  │               │        │ UART    │                  │
│    │  ┌────────────┐  │               │        ▼         │                  │
│    │  │ ESP32-CAM  │  │               │  ┌────────────┐  │                  │
│    │  │ (Vision)   │  │               │  │ RPi Zero   │  │                  │
│    │  └────────────┘  │               │  │ (Monitor)  │  │                  │
│    │                  │               │  └────────────┘  │                  │
│    │  Sensors:        │               │                  │                  │
│    │  - GPS           │               │  Outputs:        │                  │
│    │  - IMU           │               │  - REST API      │                  │
│    │  - Temp          │               │  - PDF Reports   │                  │
│    │  - Ultrasonic    │               │  - LCD Display   │                  │
│    │  - GSM/SMS       │               │                  │                  │
│    │                  │               │                  │                  │
│    └──────────────────┘               └──────────────────┘                  │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Quick Start Guide

### Step 1: Flash the Firmware

Follow these instruction documents in order:

1. **ESP32-CAM** (Vision Module): [ESP32-CAM_INSTRUCTIONS.md](./ESP32-CAM_INSTRUCTIONS.md)
2. **ESP32-C3** (Base Station Bridge): [ESP32-C3_INSTRUCTIONS.md](./ESP32-C3_INSTRUCTIONS.md)
3. **ESP32-S3** (Rover Main): [ESP32-S3_INSTRUCTIONS.md](./ESP32-S3_INSTRUCTIONS.md)
4. **Raspberry Pi Zero** (Base Station): [RPI0_INSTRUCTIONS.md](./RPI0_INSTRUCTIONS.md)

### Step 2: Wire the Hardware

See complete wiring diagrams: [WIRING_DIAGRAMS.md](./WIRING_DIAGRAMS.md)

### Step 3: Configure MAC Addresses

1. Flash ESP32-C3 first and note its MAC address
2. Update `baseMac[]` in `rover_main.ino` with ESP32-C3's MAC
3. Re-flash ESP32-S3 with the correct MAC address

### Step 4: Test the System

1. Power on the base station (RPi Zero + ESP32-C3)
2. Power on the rover
3. Connect to WiFi AP: `AeroSentinel-Base` (password: `sentinel123`)
4. Open `http://192.168.4.1` to check ESP32-C3 status
5. Open `http://aerosentinel-base.local:5000/api/status` for rover data

---

## Documentation Index

| Document | Description |
|----------|-------------|
| [ESP32-S3_INSTRUCTIONS.md](./ESP32-S3_INSTRUCTIONS.md) | Flashing instructions for rover main controller |
| [ESP32-C3_INSTRUCTIONS.md](./ESP32-C3_INSTRUCTIONS.md) | Flashing instructions for base station bridge |
| [ESP32-CAM_INSTRUCTIONS.md](./ESP32-CAM_INSTRUCTIONS.md) | Flashing instructions for vision module |
| [RPI0_INSTRUCTIONS.md](./RPI0_INSTRUCTIONS.md) | Raspberry Pi Zero auto-boot setup |
| [WIRING_DIAGRAMS.md](./WIRING_DIAGRAMS.md) | Complete wiring diagrams and pinouts |

---

## Hardware Requirements

### Rover Unit

| Component | Quantity | Notes |
|-----------|----------|-------|
| ESP32-S3 DevKit | 1 | Main controller |
| ESP32-CAM | 1 | Vision module with OV2640 |
| L9110S Motor Driver | 1 | 2-channel H-bridge |
| DC Motors | 4 | 6V-12V, with wheels |
| Neo-6M GPS | 1 | With antenna |
| MPU6050 | 1 | IMU sensor |
| DS18B20 | 1 | Temperature sensor |
| HC-SR04 | 1 | Ultrasonic sensor |
| SIM800L | 1 | GSM module |
| Active Buzzer | 1 | 5V |
| LiPo Battery | 1 | 3S (11.1V-12.6V) |
| Buck Converter | 1 | 12V to 5V, 5A+ |

### Base Station

| Component | Quantity | Notes |
|-----------|----------|-------|
| Raspberry Pi Zero | 1 | Any version |
| ESP32-C3 DevKit | 1 | Communication bridge |
| MicroSD Card | 1 | 16GB+ |
| 5V Power Supply | 1 | 2.5A+ |
| LCD 16x2 I2C | 1 | Optional |

---

## Software Architecture

### Rover State Machine

```
┌─────────┐     Hazard Detected     ┌─────────┐
│ PATROL  │ ───────────────────────►│  ALERT  │
│         │                         │         │
│         │◄────────────────────────┤         │
└────┬────┘    After 3 seconds      └────┬────┘
     │                                    │
     │ Battery < 30%                      │
     │                                    │
     ▼                                    ▼
┌─────────────┐                    ┌──────────┐
│ RETURN_BASE │                    │ RESEARCH │
│             │                    │          │
│             │                    │          │
└──────┬──────┘                    └────┬─────┘
       │                                │
       │ Arrived at base                │
       │                                │
       ▼                                │
┌─────────┐                              │
│ DOCKED  │◄─────────────────────────────┘
│         │
│ Charging│
└─────────┘
```

### Communication Flow

```
┌──────────────┐                    ┌──────────────┐
│   ESP32-S3   │                    │   ESP32-C3   │
│   (Rover)    │                    │   (Bridge)   │
├──────────────┤                    ├──────────────┤
│              │    ESP-NOW         │              │
│ Telemetry ───┼───────────────────►│ JSON via UART│
│              │    (Binary)        │      │       │
│              │                    │      ▼       │
│              │    ESP-NOW         │   Serial1    │
│ Commands ◄───┼────────────────────│              │
│              │    (Text)          │              │
└──────────────┘                    └──────────────┘
                                           │
                                           ▼
                                    ┌──────────────┐
                                    │  RPi Zero    │
                                    │  (Monitor)   │
                                    ├──────────────┤
                                    │              │
                                    │ Flask API    │
                                    │ SQLite DB    │
                                    │ PDF Reports  │
                                    │              │
                                    └──────────────┘
```

---

## API Endpoints

The Raspberry Pi Zero serves a REST API on port 5000:

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/status` | GET | Latest telemetry |
| `/api/history` | GET | Last 200 entries |
| `/api/stats` | GET | Daily statistics |
| `/api/alerts` | GET | Alert history |
| `/api/command` | POST | Send command to rover |
| `/api/health` | GET | System health check |
| `/api/config` | GET/POST | Configuration |
| `/api/reports` | GET | List PDF reports |
| `/api/export/csv` | GET | Export as CSV |

### Example Usage

```bash
# Get latest status
curl http://aerosentinel-base.local:5000/api/status

# Send command to rover
curl -X POST http://aerosentinel-base.local:5000/api/command \
  -H "Content-Type: application/json" \
  -d '{"cmd": "FORWARD"}'

# Get daily stats
curl http://aerosentinel-base.local:5000/api/stats
```

---

## Detection Capabilities

### Fire Detection
- Temperature threshold (configurable, default 55°C)
- Visual detection via camera (color analysis)
- Immediate SMS alert on detection

### Earthquake Detection
- MPU6050 accelerometer monitoring
- Triggers on high acceleration (>15 m/s²)
- Logs event with timestamp

### Human Detection
- Camera-based pattern recognition
- Skin-tone color analysis
- Motion detection integration

### Obstacle Avoidance
- Ultrasonic distance measurement
- Automatic path correction
- Tree avoidance during fire detection

---

## Power Management

### Rover Power States

| State | Motor Speed | Sensor Rate | Notes |
|-------|-------------|-------------|-------|
| Normal | 100% | 500ms | Battery > 30% |
| Power Saving | 50% | 1000ms | Battery < 30% |
| Return Base | 50% | 500ms | Navigating to charger |
| Docked | Off | 1000ms | Charging |

### Battery Monitoring

- ADC reads battery voltage via voltage divider
- Percentage calculated from 10.5V-12.6V range
- Automatic return-to-base at 30%

---

## Troubleshooting

### Common Issues

| Problem | Solution |
|---------|----------|
| No telemetry received | Check ESP-NOW MAC addresses match |
| Camera not working | Re-seat ribbon cable, contacts down |
| GPS no fix | Wait 5 minutes, ensure sky view |
| SIM800 not responding | Check power supply (2A peak) |
| Motors not moving | Verify L9110S connections and power |
| Serial communication fails | Check TX/RX not swapped |

### LED Indicators

**ESP32-C3 (Base Station Bridge):**
- 3 blinks: Startup OK
- Continuous fast blink: ESP-NOW failed
- 1 blink: Data received
- 2 blinks: Command sent

**ESP32-CAM:**
- 3 blinks: Camera initializing
- Solid on: Camera ready
- Continuous blink: Camera error

---

## Project Structure

```
RoverProject/
├── rover_main/
│   └── rover_main.ino        # ESP32-S3 firmware
├── cam_firmware/
│   └── cam_firmware.ino      # ESP32-CAM firmware
├── base_bridge/
│   └── base_bridge.ino       # ESP32-C3 firmware
├── base_station/
│   └── station_monitor.py    # RPi Zero Python script
├── rover_dashboard/
│   └── (Web dashboard files)
└── docs/
    ├── README.md             # This file
    ├── ESP32-S3_INSTRUCTIONS.md
    ├── ESP32-C3_INSTRUCTIONS.md
    ├── ESP32-CAM_INSTRUCTIONS.md
    ├── RPI0_INSTRUCTIONS.md
    └── WIRING_DIAGRAMS.md
```

---

## License

This project is provided for educational and research purposes.

---

## Support

For issues and questions:
1. Check the troubleshooting section above
2. Review the individual instruction documents
3. Verify all wiring connections
4. Check serial monitor output for error messages

---

*Document Version: 1.0*  
*Last Updated: February 2026*

