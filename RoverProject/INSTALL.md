# STASIS Autonomous Rover - Installation & Setup Guide

This guide covers the complete setup process for the STASIS autonomous environmental monitoring rover.

---

## Table of Contents

1. [Hardware Assembly](#hardware-assembly)
2. [Software Installation](#software-installation)
3. [Firmware Flashing](#firmware-flashing)
4. [Testing & Calibration](#testing--calibration)
5. [Operation](#operation)

---

## Hardware Assembly

### Required Components

#### Rover Unit
| Component | Model | Quantity |
|-----------|-------|----------|
| Main Controller | ESP32-S3 | 1 |
| Camera Module | ESP32-CAM | 1 |
| Motor Driver | L9110S | 1 |
| DC Motors | 6V-12V | 4 |
| Temperature Sensor | DS18B20 | 1 |
| Ultrasonic Sensor | HC-SR04 | 1 |
| IMU | MPU6050 | 1 |
| GPS Module | Neo-6M | 1 |
| GSM Module | SIM800L | 1 |
| Buzzer | 5V Active | 1 |
| Battery | LiPo 3S | 1 |

#### Base Station
| Component | Model | Quantity |
|-----------|-------|----------|
| Single Board Computer | Raspberry Pi Zero | 1 |
| Bridge Controller | ESP32-C3 | 1 |
| Camera | USB Webcam | 1 |
| Display | I2C LCD 16x2 | 1 |
| AprilTag | tag36h11 (200mm) | 1 |

### Wiring Connections

#### ESP32-S3 Pinout
```
GPIO4  → Motor A PWM
GPIO2  → Motor A Direction
GPIO5  → Motor B PWM
GPIO18 → Motor B Direction
GPIO6  → Motor C PWM
GPIO7  → Motor C Direction
GPIO15 → Motor D PWM
GPIO16 → Motor D Direction
GPIO10 → Ultrasonic Trig
GPIO9  → Ultrasonic Echo
GPIO14 → Buzzer
GPIO11 → DS18B20
```

#### L9110S Connection
```
Motor A: IA→GPIO4, IB→GPIO2
Motor B: IA→GPIO5, IB→GPIO18
Motor C: IA→GPIO6, IB→GPIO7
Motor D: IA→GPIO15, IB→GPIO16
```

---

## Software Installation

### Raspberry Pi Zero Setup

1. **Flash Raspberry Pi OS**
   ```bash
   # Download Raspberry Pi Imager
   # Select Raspberry Pi OS Lite (32-bit)
   # Enable SSH, set WiFi credentials
   ```

2. **Install Dependencies**
   ```bash
   sudo apt update
   sudo apt install -y python3-pip git
   pip3 install opencv-python numpy apriltag pyserial flask
   ```

3. **Configure Camera**
   ```bash
   sudo raspi-config
   # Interface Options → Camera → Enable
   # Reboot
   ```

### ESP32 Toolchain

1. **Install PlatformIO**
   ```bash
   # Linux/macOS
   pip3 install platformio
   
   # Windows
   pip install platformio
   ```

---

## Firmware Flashing

### 1. ESP32-CAM (Vision)

```bash
cd RoverProject/cam_firmware
pio run --target upload --environment esp32cam
```

### 2. ESP32-C3 (Base Bridge)

```bash
cd RoverProject/base_bridge
pio run --target upload --environment esp32-c3
```

### 3. ESP32-S3 (Main Controller)

```bash
cd RoverProject/rover_s3_firmware
pio run --target upload --environment esp32-s3
```

### 4. Raspberry Pi Zero

```bash
cd RoverProject/pi_zero_station
pip3 install -r requirements.txt

# Enable auto-start
sudo cp stasis.service /etc/systemd/system/
sudo systemctl enable stasis
sudo systemctl start stasis
```

---

## Testing & Calibration

### 1. Motor Test

```cpp
// In serial monitor, send commands:
F50    // Forward at 50%
B30    // Backward at 30%
L50    // Turn left
R50    // Turn right
S      // Stop
```

### 2. Sensor Test

```bash
# Run diagnostic script
python3 diagnostics.py
```

### 3. Camera Calibration

1. Print calibration checkerboard
2. Run:
   ```bash
   python3 calibrate_camera.py
   ```
3. Follow on-screen instructions

### 4. AprilTag Detection Test

```bash
python3 apriltag_docking.py --simulate
```

---

## Operation

### Starting the System

1. **Base Station**
   ```bash
   # Terminal 1: Start station monitor
   python3 station_monitor.py
   
   # Terminal 2: Start docking system (when needed)
   python3 apriltag_docking.py --device /dev/ttyUSB0
   ```

2. **Rover**
   - Power on battery
   - Wait for ESP-NOW connection (LED blink pattern)
   - System enters PATROL state

### State Transitions

```
PATROL → RESEARCH: Via API command
PATROL → ALERT: Fire/Human/Tilt detected
PATROL → RETURN_TO_BASE: Battery < 30%
PATROL → DOCKING: Docking enabled
```

### API Commands

```bash
# Get status
curl http://localhost:5000/api/status

# Send command
curl -X POST http://localhost:5000/api/command \
  -H "Content-Type: application/json" \
  -d '{"cmd": "RESEARCH"}'

# Enable docking
curl -X POST http://localhost:5000/api/command \
  -H "Content-Type: application/json" \
  -d '{"cmd": "DOCKING", "enabled": true}'
```

---

## Troubleshooting

### Common Issues

| Problem | Solution |
|---------|----------|
| No ESP-NOW connection | Check MAC addresses match |
| Camera not detected | Re-seat ribbon cable |
| Motors not moving | Verify L9110S power |
| GPS no fix | Ensure sky view, wait 5 min |
| Serial errors | Check TX/RX connections |

### LED Patterns

- **ESP32-S3**: 
  - Fast blink: Running
  - Slow blink: Error
  - Solid: Docked

- **ESP32-C3**:
  - 1 blink: Data RX
  - 2 blinks: Command TX
  - 3 blinks: Error

---

## Maintenance

### Battery Care
- Charge to 50% for storage
- Store in cool, dry location
- Check voltage monthly

### Cleaning
- Clean camera lens
- Remove debris from wheels
- Check sensor mounting

---

*Last Updated: February 2026*

This guide covers the complete setup process for the STASIS autonomous environmental monitoring rover.

---

## Table of Contents

1. [Hardware Assembly](#hardware-assembly)
2. [Software Installation](#software-installation)
3. [Firmware Flashing](#firmware-flashing)
4. [Testing & Calibration](#testing--calibration)
5. [Operation](#operation)

---

## Hardware Assembly

### Required Components

#### Rover Unit
| Component | Model | Quantity |
|-----------|-------|----------|
| Main Controller | ESP32-S3 | 1 |
| Camera Module | ESP32-CAM | 1 |
| Motor Driver | L9110S | 1 |
| DC Motors | 6V-12V | 4 |
| Temperature Sensor | DS18B20 | 1 |
| Ultrasonic Sensor | HC-SR04 | 1 |
| IMU | MPU6050 | 1 |
| GPS Module | Neo-6M | 1 |
| GSM Module | SIM800L | 1 |
| Buzzer | 5V Active | 1 |
| Battery | LiPo 3S | 1 |

#### Base Station
| Component | Model | Quantity |
|-----------|-------|----------|
| Single Board Computer | Raspberry Pi Zero | 1 |
| Bridge Controller | ESP32-C3 | 1 |
| Camera | USB Webcam | 1 |
| Display | I2C LCD 16x2 | 1 |
| AprilTag | tag36h11 (200mm) | 1 |

### Wiring Connections

#### ESP32-S3 Pinout
```
GPIO4  → Motor A PWM
GPIO2  → Motor A Direction
GPIO5  → Motor B PWM
GPIO18 → Motor B Direction
GPIO6  → Motor C PWM
GPIO7  → Motor C Direction
GPIO15 → Motor D PWM
GPIO16 → Motor D Direction
GPIO10 → Ultrasonic Trig
GPIO9  → Ultrasonic Echo
GPIO14 → Buzzer
GPIO11 → DS18B20
```

#### L9110S Connection
```
Motor A: IA→GPIO4, IB→GPIO2
Motor B: IA→GPIO5, IB→GPIO18
Motor C: IA→GPIO6, IB→GPIO7
Motor D: IA→GPIO15, IB→GPIO16
```

---

## Software Installation

### Raspberry Pi Zero Setup

1. **Flash Raspberry Pi OS**
   ```bash
   # Download Raspberry Pi Imager
   # Select Raspberry Pi OS Lite (32-bit)
   # Enable SSH, set WiFi credentials
   ```

2. **Install Dependencies**
   ```bash
   sudo apt update
   sudo apt install -y python3-pip git
   pip3 install opencv-python numpy apriltag pyserial flask
   ```

3. **Configure Camera**
   ```bash
   sudo raspi-config
   # Interface Options → Camera → Enable
   # Reboot
   ```

### ESP32 Toolchain

1. **Install PlatformIO**
   ```bash
   # Linux/macOS
   pip3 install platformio
   
   # Windows
   pip install platformio
   ```

---

## Firmware Flashing

### 1. ESP32-CAM (Vision)

```bash
cd RoverProject/cam_firmware
pio run --target upload --environment esp32cam
```

### 2. ESP32-C3 (Base Bridge)

```bash
cd RoverProject/base_bridge
pio run --target upload --environment esp32-c3
```

### 3. ESP32-S3 (Main Controller)

```bash
cd RoverProject/rover_s3_firmware
pio run --target upload --environment esp32-s3
```

### 4. Raspberry Pi Zero

```bash
cd RoverProject/pi_zero_station
pip3 install -r requirements.txt

# Enable auto-start
sudo cp stasis.service /etc/systemd/system/
sudo systemctl enable stasis
sudo systemctl start stasis
```

---

## Testing & Calibration

### 1. Motor Test

```cpp
// In serial monitor, send commands:
F50    // Forward at 50%
B30    // Backward at 30%
L50    // Turn left
R50    // Turn right
S      // Stop
```

### 2. Sensor Test

```bash
# Run diagnostic script
python3 diagnostics.py
```

### 3. Camera Calibration

1. Print calibration checkerboard
2. Run:
   ```bash
   python3 calibrate_camera.py
   ```
3. Follow on-screen instructions

### 4. AprilTag Detection Test

```bash
python3 apriltag_docking.py --simulate
```

---

## Operation

### Starting the System

1. **Base Station**
   ```bash
   # Terminal 1: Start station monitor
   python3 station_monitor.py
   
   # Terminal 2: Start docking system (when needed)
   python3 apriltag_docking.py --device /dev/ttyUSB0
   ```

2. **Rover**
   - Power on battery
   - Wait for ESP-NOW connection (LED blink pattern)
   - System enters PATROL state

### State Transitions

```
PATROL → RESEARCH: Via API command
PATROL → ALERT: Fire/Human/Tilt detected
PATROL → RETURN_TO_BASE: Battery < 30%
PATROL → DOCKING: Docking enabled
```

### API Commands

```bash
# Get status
curl http://localhost:5000/api/status

# Send command
curl -X POST http://localhost:5000/api/command \
  -H "Content-Type: application/json" \
  -d '{"cmd": "RESEARCH"}'

# Enable docking
curl -X POST http://localhost:5000/api/command \
  -H "Content-Type: application/json" \
  -d '{"cmd": "DOCKING", "enabled": true}'
```

---

## Troubleshooting

### Common Issues

| Problem | Solution |
|---------|----------|
| No ESP-NOW connection | Check MAC addresses match |
| Camera not detected | Re-seat ribbon cable |
| Motors not moving | Verify L9110S power |
| GPS no fix | Ensure sky view, wait 5 min |
| Serial errors | Check TX/RX connections |

### LED Patterns

- **ESP32-S3**: 
  - Fast blink: Running
  - Slow blink: Error
  - Solid: Docked

- **ESP32-C3**:
  - 1 blink: Data RX
  - 2 blinks: Command TX
  - 3 blinks: Error

---

## Maintenance

### Battery Care
- Charge to 50% for storage
- Store in cool, dry location
- Check voltage monthly

### Cleaning
- Clean camera lens
- Remove debris from wheels
- Check sensor mounting

---

*Last Updated: February 2026*

