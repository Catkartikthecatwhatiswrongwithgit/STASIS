# STASIS - Autonomous Environmental Monitoring Rover

A production-ready autonomous rover system for environmental monitoring, hazard detection, and autonomous docking. Built with ESP32-S3, Raspberry Pi Zero, and modular sensor integration.

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           STASIS ROVER SYSTEM                                │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   ┌──────────────┐    ┌──────────────┐    ┌──────────────┐                 │
│   │   ROVER     │    │    BASE      │    │   REMOTE    │                 │
│   │   (Mobile)  │◄──►│  (Station)   │◄──►│  (Dashboard)│                 │
│   └──────┬───────┘    └──────┬───────┘    └──────────────┘                 │
│          │                   │                                             │
│          │    ┌──────────────┴───────┐                                   │
│          │    │   COMMUNICATION      │                                   │
│          │    │   • ESP-NOW          │                                   │
│          │    │   • UART (Pi↔ESP32)  │                                   │
│          │    │   • WiFi (Dashboard) │                                   │
│          │    └───────────────────────┘                                   │
│          │                                                                   │
│   ┌──────┴────────────────────────────────────────────────────────┐       │
│   │                     ROVER HARDWARE                             │       │
│   ├───────────────────────────────────────────────────────────────┤       │
│   │  ESP32-S3 (Main Controller)                                   │       │
│   │    ├── Navigation & State Machine                            │       │
│   │    ├── Motor Control (L9110S)                                │       │
│   │    ├── Sensor Fusion                                         │       │
│   │    └── Communication Bridge                                  │       │
│   │                                                               │       │
│   │  ESP32-CAM (Vision)                                          │       │
│   │    ├── Fire Detection                                         │       │
│   │    ├── Human Detection                                        │       │
│   │    └── Tree Detection                                         │       │
│   │                                                               │       │
│   │  Sensors                                                     │       │
│   │    ├── DS18B20 (Temperature)                                 │       │
│   │    ├── HC-SR04 (Ultrasonic)                                  │       │
│   │    ├── MPU6050 (Tilt/Seismic)                               │       │
│   │    └── Neo-6M (GPS)                                          │       │
│   │                                                               │       │
│   │  Communication                                                │       │
│   │    ├── SIM800 (SMS Alerts)                                   │       │
│   │    └── ESP-NOW (Base Link)                                   │       │
│   └───────────────────────────────────────────────────────────────┘       │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Hardware List

### Rover
| Component | Model | Purpose |
|-----------|-------|---------|
| Main MCU | ESP32-S3 | Navigation, state machine, motor control |
| Camera | ESP32-CAM | Fire/human/tree detection |
| Motor Driver | L9110S | 4WD differential drive |
| Temperature | DS18B20 | Environmental monitoring |
| Ultrasonic | HC-SR04 | Obstacle avoidance |
| IMU | MPU6050 | Tilt/seismic detection |
| GPS | Neo-6M | Location tracking |
| Modem | SIM800 | SMS alerts |
| Battery | LiPo 3S | Power |

### Base Station
| Component | Model | Purpose |
|-----------|-------|---------|
| Processing | Raspberry Pi Zero | AprilTag detection |
| Bridge | ESP32-C3 | UART↔ESP-NOW |
| Vision | USB Camera | AprilTag detection |
| Display | I2C LCD | Status display |
| Tag | AprilTag 36h11 | Docking beacon |

---

## State Machine

```
                    ┌──────────────────────────────────────┐
                    │                                      │
                    ▼                                      │
    ┌───────────────────────────┐                          │
    │         PATROL            │◄─────────────────────────┤
    │  (Normal operation)       │                          │
    └─────────────┬─────────────┘                          │
                  │                                         │
    ┌─────────────┴─────────────┐     ┌───────────────────┐ │
    │                           │     │                   │ │
    ▼                           ▼     ▼                   ▼ │
┌────────────┐  ┌──────────────────┐  ┌────────────────┐    │
│  RESEARCH  │  │ RETURN_TO_BASE  │  │     DOCKING    │    │
│(Data collect)│ │ (Low battery)   │  │ (AprilTag)    │    │
└──────┬──────┘  └────────┬─────────┘  └───────┬────────┘    │
       │                 │                      │             │
       │    ┌────────────┴──────────┐           │             │
       │    │                      │           │             │
       ▼    ▼                      ▼           ▼             │
┌──────────────────────────────────────────────────────────┐ │
│                        ALERT                              │ │
│  (Fire/Earthquake/Intruder - Investigate & Alert)       │ │
└──────────────────────────────────────────────────────────┘ │
                              │                              │
                              │     ┌───────────────────────┘
                              │     │
                              ▼     ▼
                    ┌──────────────────┐
                    │     DOCKED       │
                    │ (Charging/Idle)  │
                    └──────────────────┘
```

### State Transitions

| From State | To State | Trigger Condition |
|------------|----------|-------------------|
| PATROL | RESEARCH | Timer or command |
| PATROL | ALERT | fireDetected OR humanDetected OR tiltDetected |
| PATROL | RETURN_TO_BASE | batteryPercent < 30% |
| PATROL | DOCKING | dockingEnabled = true |
| RESEARCH | PATROL | Timeout (5 min) |
| RESEARCH | ALERT | Hazard detected |
| ALERT | PATROL | Timeout or resolved |
| RETURN_TO_BASE | DOCKING | dockingEnabled = true |
| DOCKING | DOCKED | Successfully docked |
| DOCKED | PATROL | dockingEnabled = false |

---

## Folder Structure

```
STASIS-Rover/
├── .gitignore
├── README.md
├── LICENSE
│
├── rover_s3_firmware/          # ESP32-S3 main controller
│   ├── platformio.ini
│   ├── src/
│   │   └── rover_main.cpp
│   └── lib/
│       └── (dependencies)
│
├── esp32_cam_firmware/         # ESP32-CAM vision
│   ├── cam_firmware.ino
│   └── libraries/
│
├── base_station_c3/            # ESP32-C3 bridge
│   ├── platformio.ini
│   └── src/
│       └── bridge.cpp
│
├── pi_zero_station/            # Raspberry Pi Zero
│   ├── requirements.txt
│   ├── apriltag_docking.py    # AprilTag detection + control
│   ├── station_monitor.py     # Main station logic
│   └── config/
│       └── camera_calibration.json
│
├── cad/                        # 3D printed parts
│   └── (STL files)
│
└── docs/
    ├── PROTOCOL.md            # Communication protocols
    ├── HARDWARE.md           # Wiring diagrams
    ├── CALIBRATION.md        # Sensor calibration
    └── TROUBLESHOOTING.md    # Common issues
```

---

## Quick Start

### 1. Hardware Assembly

1. **Motor Driver**: Connect L9110S to ESP32-S3 GPIO pins (see `WIRING_DIAGRAMS.md`)
2. **Sensors**: Wire DS18B20, HC-SR04, MPU6050 per pin definitions
3. **GPS**: Connect Neo-6M to UART2
4. **SIM800**: Connect to UART0 with level shifting

### 2. Base Station Setup

```bash
# Install dependencies
cd pi_zero_station
pip install -r requirements.txt

# Run AprilTag docking system
python apriltag_docking.py --device /dev/ttyUSB0
```

### 3. Rover Flash

```bash
# Using PlatformIO
cd rover_s3_firmware
pio run --target upload
```

### 4. Commissioning

1. Verify all sensors respond
2. Test motor direction
3. Calibrate camera focal length
4. Test AprilTag detection at various distances
5. Verify ESP-NOW pairing

---

## Command Protocol

### Pi Zero → ESP32-S3 (Docking)

| Byte | Field | Range |
|------|-------|-------|
| 0 | HEADER | 0xAA |
| 1 | turn | -100 to +100 |
| 2 | speed | 0-100 |
| 3 | docked | 0 or 1 |
| 4 | checksum | XOR(1,2,3) |
| 5 | TAIL | 0x55 |

### ESP32-S3 → Base (Telemetry)

See `docs/PROTOCOL.md` for full specification.

---

## Current Limitations

- **GPS**: Neo-6M requires clear sky view; indoor operation limited
- **AprilTag**: Requires adequate lighting; performance degrades in low-light
- **WiFi**: Range limited to ~100m outdoors; use ESP-NOW for extended range
- **Battery**: Actual runtime depends on terrain and load; 30% threshold is conservative

---

## Future Work

- [ ] Implement SLAM for autonomous navigation
- [ ] Add machine learning for hazard classification
- [ ] Multi-rover coordination via ESP-NOW mesh
- [ ] Solar charging integration
- [ ] Web interface for real-time monitoring

---

## License

MIT License - See LICENSE file

---

## Authors

- AeroSentinel Team

## Acknowledgments

- AprilTag library by University of Michigan
- ESP32 Arduino framework
