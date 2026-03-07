# STASIS Communication Protocols

This document describes all communication protocols used in the STASIS rover system.

---

## Table of Contents

1. [ESP-NOW Protocol](#esp-now-protocol)
2. [UART Protocol](#uart-protocol)
3. [Command Format](#command-format)
4. [Telemetry Format](#telemetry-format)
5. [Camera Messages](#camera-messages)

---

## ESP-NOW Protocol

### Overview

ESP-NOW is used for wireless communication between the rover (ESP32-S3) and base station bridge (ESP32-C3).

- **Range**: Up to 500m outdoors with external antenna
- **Latency**: ~5-10ms
- **Frequency**: 2.4 GHz WiFi band

### MAC Addresses

Each device must have a unique MAC address. Default addresses:

| Device | MAC Address |
|--------|-------------|
| ESP32-S3 (Rover) | `30:AE:A4:XX:XX:XX` |
| ESP32-C3 (Base) | `DC:54:75:XX:XX:XX` |

### Message Types

| Type ID | Direction | Description |
|---------|-----------|-------------|
| 0x01 | Rover → Base | Telemetry packet (JSON) |
| 0x02 | Base → Rover | Command packet |
| 0x03 | Bidirectional | Heartbeat/keepalive |

---

## UART Protocol

### Overview

UART is used for communication between ESP32-C3 (bridge) and Raspberry Pi Zero.

- **Baud Rate**: 115200
- **Data Bits**: 8
- **Parity**: None
- **Stop Bits**: 1

### Data Flow

```
ESP32-S3 (Rover) --ESP-NOW--> ESP32-C3 (Bridge) --UART--> Raspberry Pi Zero
                           (JSON parsing)         (station_monitor.py)
```

---

## Command Format

### Rover Commands (Pi → Rover)

Commands sent from Raspberry Pi to rover via ESP32-C3 bridge.

| Command | Description |
|---------|-------------|
| `STOP` | Emergency stop |
| `FORWARD` | Move forward |
| `BACKWARD` | Move backward |
| `LEFT` | Turn left |
| `RIGHT` | Turn right |
| `START_PATROL` | Start autonomous patrol |
| `PATROL` | Start autonomous patrol (alias) |
| `STOP_PATROL` | Stop patrol |
| `RETURN_BASE` | Return to base station |
| `RETURN_HOME` | Return to base station (alias) |
| `HOME` | Return to base station (alias) |
| `START_RESEARCH` | Start research/investigation mode |
| `RESEARCH` | Start research mode (alias) |
| `ENABLE_DOCKING` | Enable AprilTag docking mode |
| `DOCKING` | Docking mode (alias) |
| `DISABLE_DOCKING` | Disable docking mode |
| `ALERT` | Trigger alert |
| `RESET` | Reset rover |

---

## Telemetry Format

### Rover Telemetry (JSON)

Telemetry sent from rover to base station. Format:

```json
{
  "id": 123,
  "temp": 25.5,
  "bat": 85.0,
  "lat": 12.3456,
  "lng": 78.9012,
  "hazard": false,
  "status": "PATROL",
  "distance": 150.5,
  "accelX": 0.1,
  "accelY": 0.05
}
```

### Status Values

| Value | State | Description |
|-------|-------|-------------|
| PATROL | PATROL | Normal patrol operation |
| RESEARCH | RESEARCH | Research mode |
| ALERT | ALERT | Hazard detected |
| RETURN_BASE | RETURN_BASE | Low battery return |
| DOCKING | DOCKING | AprilTag docking |
| DOCKED | DOCKED | Successfully docked |
| IDLE | IDLE | Idle/standby |
| EMERGENCY | EMERGENCY | Critical error |

---

## Camera Messages

### Non-JSON Messages from ESP32-CAM

The ESP32-CAM sends text messages that are parsed by station_monitor.py:

| Message | Description |
|---------|-------------|
| `HAZARD:FIRE` | Fire detected by camera |
| `HAZARD:MOTION` | Motion detected |
| `HAZARD:HUMAN` | Human detected |
| `CAM:READY` | Camera initialized |
| `CAM:ERROR` | Camera error |
| `ERROR:...` | Generic error |

### Message Handling

The Pi parses these messages and:
1. Adds them to alert history
2. Broadcasts via WebSocket to connected clients
3. Logs warnings for hazard messages

---

## Communication Timing

### Update Rates

| Data | Rate | Method |
|------|------|--------|
| Telemetry | 1 Hz | ESP-NOW |
| Commands | As needed | ESP-NOW |
| Camera Messages | Event-based | UART |

### Timeouts

| Condition | Timeout | Action |
|-----------|---------|--------|
| No telemetry | 10 seconds | Mark rover offline |
| No command (docking) | 500 ms | Emergency stop |

---

## API Endpoints

### Base Station (station_monitor.py)

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/status` | GET | Latest telemetry |
| `/api/history` | GET | Telemetry history (limit param) |
| `/api/stats` | GET | Daily statistics |
| `/api/alerts` | GET | Alert history |
| `/api/command` | POST | Send command to rover |
| `/api/reports` | GET | List generated reports |
| `/api/reports/<filename>` | GET | Download report |
| `/api/health` | GET | System health (no auth) |
| `/api/config` | GET/POST | Configuration |
| `/api/config/sms` | GET/POST | SMS recipients |
| `/api/export/csv` | GET | Export telemetry as CSV |
| `/api/export/json` | GET | Export telemetry as JSON |
| `/ws` | WebSocket | Real-time chat/telemetry |

### Authentication

Most endpoints require API key in header:
- Header: `X-API-Key: stasis-2024` (default)
- Or query param: `?api_key=stasis-2024`
- `/api/health` is public (no auth required)

---

## Notes

1. All multi-byte integers are sent little-endian
2. GPS coordinates are in decimal degrees (WGS84)
3. Temperature is in Celsius
4. Distance is in centimeters
5. Accelerometer values are in m/s²

---

*Document Version: 2.0*
*Last Updated: February 2026*
