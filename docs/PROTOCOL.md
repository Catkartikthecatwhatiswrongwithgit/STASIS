# STASIS Communication Protocols

---

## Architecture Summary

```
ESP32-CAM ──Serial──► ESP32-S3 ──ESP-NOW──► ESP32-C3 ──UART──► Raspberry Pi
                        (rover)              (bridge)             (Flask API)
                           ▲                                          │
                           └──────────── binary DockCommand ◄─────────┘
                                         (during docking only)
```

---

## 1. Camera → Rover (Serial, UART0)

The ESP32-CAM sends plain-text messages to the ESP32-S3 over its default Serial port at 115200 baud.

**Output from ESP32-CAM:**

| Message | Meaning |
|---------|---------|
| `CAM:READY` | Camera initialised successfully |
| `CAM:ERROR` | Camera failed to initialise |
| `HAZARD:FIRE` | Fire pixels detected above threshold |
| `HAZARD:MOTION` | Motion pixels detected above threshold |
| `HAZARD:HUMAN` | Skin-tone pattern detected above threshold |
| `CLEAR` | No hazards in current frame |

> **Note (v2):** Binary `DetectionResult` struct writes were removed. All output is text-only. This eliminates corruption of the S3's line parser.

**Commands from ESP32-S3 to ESP32-CAM (single byte):**

| Byte | Command | Effect |
|------|---------|--------|
| `0x01` | CAPTURE | Capture one frame and return detection result |
| `0x02` | STREAM_ON | Start continuous MJPEG stream |
| `0x03` | STREAM_OFF | Stop streaming |
| `0x04` | DETECT | Run detection only, no stream |
| `0x05` | APRIL_STREAM | Switch to JPEG mode and start streaming (for AprilTag) |

---

## 2. Rover → Base (ESP-NOW, wireless)

The ESP32-S3 broadcasts telemetry to the ESP32-C3 bridge via ESP-NOW at approximately 1 Hz.

**Telemetry packet (JSON, forwarded to Pi over UART):**

```json
{
  "id":       123,
  "temp":     25.5,
  "bat":      82.0,
  "lat":      12.345678,
  "lng":      77.123456,
  "hazard":   false,
  "status":   "PATROL",
  "distance": 142.3,
  "speed":    40,
  "heading":  180.0
}
```

| Field | Type | Description |
|-------|------|-------------|
| `id` | int | Incrementing packet counter |
| `temp` | float | DS18B20 temperature in °C |
| `bat` | float | Battery percentage (0–100) |
| `lat` / `lng` | float | GPS coordinates (decimal degrees) |
| `hazard` | bool | True if any hazard is active |
| `status` | string | Current state name |
| `distance` | float | Ultrasonic reading in cm |
| `speed` | int | Current motor speed (0–100) |
| `heading` | float | Compass heading in degrees (if available) |

**Status values:**

| Value | State |
|-------|-------|
| `SAFE_IDLE` | Idle |
| `PATROL` | Autonomous patrol |
| `ALERT` | Hazard detected |
| `RETURN_TO_BASE` | Returning home |
| `DOCKING` | AprilTag approach active |
| `DOCKED_CHARGING` | On charge |

---

## 3. Pi → Rover (UART → ESP32-C3 → ESP-NOW)

### 3a. Text commands (all states except DOCKING)

Plain text strings terminated with `\n`. The bridge forwards them to the rover via ESP-NOW.

| Command | Effect |
|---------|--------|
| `STOP` | Emergency stop |
| `FORWARD` | Drive forward at speed 50 |
| `BACKWARD` | Drive in reverse at speed 50 |
| `LEFT` | Turn left |
| `RIGHT` | Turn right |
| `PATROL` | Start autonomous patrol |
| `STOP_PATROL` | Stop patrol |
| `HOME` / `RETURN_BASE` / `RETURN_HOME` | Return to dock |
| `DOCKING` / `ENABLE_DOCKING` | Activate docking mode |
| `DISABLE_DOCKING` | Cancel docking |
| `SETHOME` | Set current GPS as home |
| `ALERT` | Force ALERT state |
| `RESET` | Reset all state flags |

### 3b. Binary DockCommand packets (DOCKING state only)

During `STATE_DOCKING`, the Pi sends 6-byte binary packets at ~20 Hz. These bypass the text parser.

```
Byte  Field     Type    Range
  0   Header    uint8   0xAA  (fixed)
  1   turn      int8    -100 to +100  (negative = left, positive = right)
  2   speed     uint8   0 to 100
  3   docked    uint8   0 or 1
  4   checksum  uint8   XOR(byte1, byte2, byte3)
  5   Tail      uint8   0x55  (fixed)
```

The rover uses a `binaryMode` flag in `uartTask` to capture all 6 bytes before parsing. The `parseDockCommand()` function validates header, tail, and XOR checksum before accepting a packet.

---

## 4. Pi REST API

Base URL: `http://192.168.4.1:5000`

Most endpoints require the API key header: `X-API-Key: stasis-2024`

| Endpoint | Method | Auth | Description |
|----------|--------|------|-------------|
| `/api/health` | GET | No | Liveness check |
| `/api/status` | GET | Yes | Latest telemetry JSON |
| `/api/history` | GET | Yes | Telemetry history (`?limit=N`) |
| `/api/alerts` | GET | Yes | Alert history |
| `/api/stats` | GET | Yes | Daily statistics |
| `/api/command` | POST | Yes | Send command: `{"cmd": "PATROL"}` |
| `/api/reports` | GET | Yes | List PDF reports |
| `/api/reports/<file>` | GET | Yes | Download a report |
| `/api/export/csv` | GET | Yes | Export telemetry as CSV |
| `/api/export/json` | GET | Yes | Export telemetry as JSON |
| `/api/config` | GET/POST | Yes | Read / write config |

### Socket.IO events (real-time)

Connect with `io()` (no URL argument — auto-connects to origin).

| Event | Direction | Payload |
|-------|-----------|---------|
| `telemetry` | Server → Client | Latest telemetry object |
| `alert` | Server → Client | Alert object |
| `chat` | Bidirectional | `{user, message, channel, timestamp}` |
| `user_joined` | Server → Client | `{user}` |
| `user_left` | Server → Client | `{user}` |
| `apriltag` | Server → Client | `{found, tag_id, err_x, dist_m}` |
| `connected` | Server → Client | `{message}` on first connect |

---

## 5. Timing

| Data stream | Rate | Method |
|-------------|------|--------|
| Rover telemetry | ~1 Hz | ESP-NOW broadcast |
| Dock command packets | ~20 Hz | UART binary during DOCKING only |
| Camera hazard messages | Event-driven | UART text |
| AprilTag Socket.IO events | ~20 Hz | Socket.IO during DOCKING only |
| Battery updates | 10 Hz | Internal (sensorTask) |
| GPS NMEA | 1 Hz | Serial2 hardware UART |

---

*Last updated: March 2026 — v2*
