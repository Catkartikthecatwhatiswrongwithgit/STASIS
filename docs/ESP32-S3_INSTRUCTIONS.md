# ESP32-S3 Rover Main Controller - Firmware Guide

## Overview

The ESP32-S3 is the main controller for the STASIS rover. It handles:
- State machine control (PATROL, ALERT, RETURN_TO_BASE, DOCKING, DOCKED_CHARGING, SAFE_IDLE)
- Motor control (L9110S 4WD)
- Sensor fusion (ultrasonic, temperature, GPS)
- Dock detection via battery voltage rise
- GPS geofence monitoring
- GSM alert triggering via SIM800
- Communication with Raspberry Pi via UART

---

## Hardware

### Board
- ESP32-S3 Dev Module
- USB-C for programming

### Pin Assignments

| Function | GPIO | Notes |
|----------|------|-------|
| Motor A PWM | 4 | L9110S |
| Motor A DIR | 2 | L9110S |
| Motor B PWM | 5 | L9110S |
| Motor B DIR | 18 | L9110S |
| Motor C PWM | 6 | L9110S |
| Motor C DIR | 7 | L9110S |
| Motor D PWM | 15 | L9110S |
| Motor D DIR | 16 | L9110S |
| Ultrasonic TRIG | 10 | HC-SR04 |
| Ultrasonic ECHO | 9 | HC-SR04 |
| GPS TX | 8 | NEO-6M |
| GPS RX | 19 | NEO-6M |
| SIM800 TX | 13 | |
| SIM800 RX | 12 | |
| Buzzer | 14 | |
| Status LED | 45 | |
| Battery ADC | 1 | Via voltage divider |

---

## State Machine

```
SAFE_IDLE ←→ PATROL ←→ ALERT
                ↓
        RETURN_TO_BASE
                ↓
           DOCKING ←→ DOCKED_CHARGING
```

### States

| State | Description |
|-------|-------------|
| SAFE_IDLE | Minimal power, waiting for commands |
| PATROL | Autonomous patrol with geofence |
| ALERT | Hazard detected, investigating |
| RETURN_TO_BASE | Geofence breach or manual recall |
| DOCKING | AprilTag precision approach |
| DOCKED_CHARGING | Passive charging, motors disabled |

---

## Commands

Send via UART from Raspberry Pi:

| Command | Action |
|---------|--------|
| `STOP` | Emergency stop → SAFE_IDLE |
| `FORWARD` | Move forward |
| `BACKWARD` | Move backward |
| `LEFT` | Turn left |
| `RIGHT` | Turn right |
| `PATROL` | Start autonomous patrol |
| `STOP_PATROL` | Stop patrol → SAFE_IDLE |
| `HOME` / `RETURN_BASE` | Return to base |
| `DOCKING` | Enable docking mode |
| `DISABLE_DOCKING` | Disable docking |
| `SETHOME` | Set current GPS as home |
| `RESET` | Reset all states |

---

## Safety Features

### Dock Detection
- Monitors battery voltage rise (charging)
- Requires 0.4V above baseline sustained for 2 seconds
- 20-frame debounce to prevent chatter

### GPS Geofence
- 50m radius from home position
- Ignores invalid coordinates (0.0, 0.0)
- SAFE_HALT if GPS older than 5 seconds

### SIM800 Protection
- Motors stop before SMS to prevent brownout
- 60-second cooldown between SMS
- Non-blocking operation

### Ultrasonic
- Non-blocking with 30ms timeout
- Default 999cm when no reading

---

## Configuration Constants

```cpp
// Timeout
#define COMMAND_TIMEOUT_MS  500

// Dock Detection
#define DOCK_VOLTAGE_RISE      0.4    // Volts
#define DOCK_CONFIRM_TIME_MS   2000   // ms
#define DOCK_DEBOUNCE_FRAMES  20

// Geofence
#define GEOFENCE_RADIUS_M   50.0    // meters
#define GPS_VALID_AGE_MS   10000    // ms

// Battery
#define LOW_BATTERY_THRESHOLD 30.0   // percent
#define LOW_BATTERY_VOLTAGE   6.2    // volts (2S cutoff)

// GSM
#define SMS_COOLDOWN_MS 60000  // 60 seconds
```

---

## Uploading

1. Open `rover_main/rover_main.ino` in Arduino IDE
2. Select ESP32S3 Dev Module
3. Set baud to 921600
4. Hold BOOT button while uploading if needed

---

## Serial Output

Monitor at 115200 baud to see:
- State transitions
- GPS coordinates
- Battery voltage
- Dock detection events
- Alert triggers

---

*Last Updated: March 2026*
