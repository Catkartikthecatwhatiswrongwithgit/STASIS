# ESP32-S3 — Rover Main Controller

## Overview

The ESP32-S3 is the brain of the STASIS rover. It runs a FreeRTOS multi-task firmware that handles the state machine, motor control, sensor fusion, GPS geofencing, dock detection, and GSM alerts.

---

## Board Setup (Arduino IDE)

1. **File → Preferences → Additional Boards Manager URLs:**
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
2. **Tools → Board → Boards Manager** → install `esp32 by Espressif Systems`
3. **Tools → Board → ESP32 Arduino → ESP32S3 Dev Module**

| Setting | Value |
|---------|-------|
| Upload Speed | 921600 |
| USB CDC on Boot | Enabled |
| CPU Frequency | 240 MHz |
| Flash Size | 4 MB |
| Partition Scheme | Default 4 MB with spiffs |

---

## Required Libraries

Install via Library Manager:

| Library | Purpose |
|---------|---------|
| TinyGPSPlus | NMEA GPS parsing (optional, custom parser also included) |
| OneWire | DS18B20 temperature sensor |
| DallasTemperature | DS18B20 high-level API |

---

## Uploading

1. Open `rover_main/rover_main.ino`
2. Connect ESP32-S3 via USB-C
3. Select the correct COM port
4. Click **Upload**

> If upload fails, hold the **BOOT** button, click Upload, release BOOT when "Connecting..." appears.

---

## Pin Assignments

| GPIO | Function | Connected To | Notes |
|------|----------|--------------|-------|
| 1 | Battery ADC | 30k/10k divider | Reads pack voltage |
| 2 | Motor A DIR | L9110S | |
| 4 | Motor A PWM | L9110S | LEDC channel |
| 5 | Motor B PWM | L9110S | LEDC channel |
| 6 | Motor C PWM | L9110S | LEDC channel |
| 7 | Motor C DIR | L9110S | |
| 8 | GPS Serial2 RX | NEO-6M TX | Hardware Serial2 |
| 9 | Ultrasonic ECHO | HC-SR04 | |
| 10 | Ultrasonic TRIG | HC-SR04 | |
| 12 | SIM800 RX | SIM800L TXD | **Serial1** |
| 13 | SIM800 TX | SIM800L RXD | **Serial1** |
| 14 | Buzzer | Active buzzer | |
| 15 | Motor D PWM | L9110S | LEDC channel |
| 16 | Motor D DIR | L9110S | |
| 18 | Motor B DIR | L9110S | |
| 19 | GPS Serial2 TX | NEO-6M RX | Hardware Serial2 |
| 21 | DS18B20 | OneWire data | 4.7 kΩ pull-up to 3.3 V |
| 43 | UART1 TX | Pi Zero (via C3) | |
| 44 | UART1 RX | Pi Zero (via C3) | |
| 45 | Status LED | — | |

> **Serial port assignment:**
> GPS (NEO-6M) → **Serial2** on pins 8, 19.
> SIM800L → **Serial1** on pins 12, 13.
> These are separate hardware UARTs and do not conflict.

---

## State Machine

| State | Behaviour |
|-------|-----------|
| `SAFE_IDLE` | Motors off. Sensors active. Waits for command. |
| `PATROL` | Drives forward at patrol speed. Monitors geofence, hazards, and obstacles. |
| `ALERT` | Motors stop. Buzzer sounds. SMS sent. Waits for confirmation or timeout. |
| `RETURN_TO_BASE` | Drives toward home GPS coordinates. |
| `DOCKING` | Receives binary `DockCommand` packets from Pi via UART and applies turn/speed directly. |
| `DOCKED_CHARGING` | Motors fully disabled. Battery and temperature monitored. |

All transitions use `millis()` — no `delay()` in any state path.

---

## Commands (via UART from Pi)

| Command | Effect |
|---------|--------|
| `STOP` | → SAFE_IDLE, speed = 0 |
| `FORWARD` | speed = 50 forward |
| `BACKWARD` | speed = 50 reverse |
| `LEFT` | turn = −50, speed = 30 |
| `RIGHT` | turn = +50, speed = 30 |
| `PATROL` | → PATROL state |
| `STOP_PATROL` | → SAFE_IDLE |
| `HOME` / `RETURN_BASE` | → RETURN_TO_BASE |
| `DOCKING` | → DOCKING state |
| `DISABLE_DOCKING` | → SAFE_IDLE |
| `SETHOME` | Save current GPS as home (requires valid fix) |
| `RESET` | Reset all flags, → SAFE_IDLE |

---

## Docking Binary Packet (Pi → Rover)

During `STATE_DOCKING`, the Pi sends 6-byte binary packets over UART:

```
Byte 0: 0xAA       (header)
Byte 1: turn       (int8, -100 to +100)
Byte 2: speed      (uint8, 0-100)
Byte 3: docked     (uint8, 0 or 1)
Byte 4: checksum   (XOR of bytes 1, 2, 3)
Byte 5: 0x55       (tail)
```

The rover's `uartTask` uses a `binaryMode` flag to collect all 6 bytes before parsing. Text commands cannot be mixed with binary packets mid-stream.

---

## Key Configuration Constants

```cpp
// Timeouts
#define COMMAND_TIMEOUT_MS    500     // Stop if no dock command for 500 ms
#define GPS_VALID_AGE_MS      10000   // Discard GPS fixes older than 10 s
#define GPS_STARTUP_GRACE_MS  90000   // Don't GPS-halt for first 90 s (cold start)

// Dock detection
#define DOCK_VOLTAGE_RISE     0.4     // Volts above baseline to confirm dock
#define DOCK_DEBOUNCE_FRAMES  20      // Frames (~2 s at 100 ms/frame)

// Geofence
#define GEOFENCE_RADIUS_M     50.0    // Metres from home

// Battery
#define LOW_BATTERY_THRESHOLD 20      // % — triggers low-power mode
#define LOW_BATTERY_VOLTAGE   6.2     // V — hard shutdown
#define CHARGE_COMPLETE_VOLTAGE 8.4   // V — full cell voltage (2S)

// GSM
#define SMS_COOLDOWN_MS       60000   // One SMS per 60 s maximum

// Motor
#define SPEED_RAMP_RATE       5       // PWM units per control cycle
```

---

## FreeRTOS Task Layout

| Task | Core | Priority | Stack | Responsibility |
|------|------|----------|-------|----------------|
| `uartTask` | 0 | 2 | 3072 | Receive commands and dock packets from Pi |
| `cameraTask` | 1 | 1 | 3072 | Forward ESP32-CAM hazard messages to Pi |
| `sensorTask` | 1 | 1 | 4096 | Ultrasonic, temperature, LDR, battery, GSM |
| `gpsTask` | 1 | 1 | 3072 | NMEA parsing, geofence updates |
| `stateTask` | 1 | 1 | 3072 | State machine transitions and motor execution |

---

## Serial Monitor Output

Connect at **115200 baud** to see:

```
STASIS Rover Initializing...
DOCK: Baseline established at 7.41V
GEOFENCE: Home set to 12.345678, 77.123456
State: 1                          ← state enum value
GPS: No fix - SAFE_HALT           ← if GPS lost
DOCK: Confirmed at 7.83V (rise: 0.42V)
CHARGING: Voltage=7.83V, Started=YES
```

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Rover doesn't stop on STOP command | `estop()` broken | Ensure `ledcWrite` used, not `digitalWrite` |
| BACKWARD drives forward | Old bug (fixed in v2) | Flash latest firmware |
| GPS halts rover immediately on boot | Startup grace missing | Flash latest firmware (90 s grace added) |
| SMS sends but GPS is garbled | Serial2 conflict | Check SIM800 is on Serial1, GPS on Serial2 |
| Docking doesn't steer | Binary packet parser | Check `binaryMode` flag present in `uartTask` |
| Stack overflow crash | Task stack too small | Ensure stacks are 3072 / 4096 as set |

---

*Last updated: March 2026 — v2*
