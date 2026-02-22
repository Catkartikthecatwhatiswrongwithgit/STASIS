# STASIS Communication Protocols

This document describes all communication protocols used in the STASIS rover system.

---

## Table of Contents

1. [ESP-NOW Protocol](#esp-now-protocol)
2. [UART Protocol](#uart-protocol)
3. [Command Format](#command-format)
4. [Telemetry Format](#telemetry-format)

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
| 0x01 | Rover → Base | Telemetry packet |
| 0x02 | Base → Rover | Command packet |
| 0x03 | Bidirectional | Heartbeat/keepalive |
| 0x04 | Base → Rover | Configuration update |

---

## UART Protocol

### Overview

UART is used for communication between ESP32-C3 (bridge) and Raspberry Pi Zero.

- **Baud Rate**: 115200
- **Data Bits**: 8
- **Parity**: None
- **Stop Bits**: 1

### Packet Framing

All UART messages use the following frame format:

```
[HEADER] [TYPE] [LENGTH] [PAYLOAD...] [CHECKSUM] [TAIL]
```

| Field | Size | Value | Description |
|-------|------|-------|-------------|
| HEADER | 1 | 0xAA | Packet start marker |
| TYPE | 1 | 0x01-0x04 | Message type |
| LENGTH | 1 | N | Payload length in bytes |
| PAYLOAD | N | varies | Message data |
| CHECKSUM | 1 | uint8 | XOR of all bytes except TAIL |
| TAIL | 1 | 0x55 | Packet end marker |

---

## Command Format

### Rover Commands (Base → Rover)

Commands sent from base station to rover via ESP-NOW.

#### Structure

```c
struct RoverCommand {
    uint8_t type;        // Always 0x02
    uint8_t sequence;    // Sequence number (0-255)
    uint8_t cmd_id;      // Command ID
    int16_t param;       // Command parameter
    uint8_t checksum;
};
```

#### Command IDs

| ID | Command | Parameter | Description |
|----|---------|-----------|-------------|
| 0x01 | STOP | 0 | Emergency stop |
| 0x02 | FORWARD | 0-100 | Move forward (speed) |
| 0x03 | BACKWARD | 0-100 | Move backward (speed) |
| 0x04 | LEFT | 0-100 | Turn left (angle/speed) |
| 0x05 | RIGHT | 0-100 | Turn right (angle/speed) |
| 0x06 | SET_STATE | enum | Set rover state |
| 0x07 | ENABLE_DOCKING | 0/1 | Enable/disable docking mode |
| 0x08 | SET_SPEED | 0-100 | Set max speed |
| 0x09 | RESET | 0 | Reset rover |
| 0x0A | GOTO_WAYPOINT | index | Navigate to waypoint |

---

## Telemetry Format

### Rover Telemetry (Rover → Base)

Telemetry sent from rover to base station.

#### Structure

```c
struct RoverTelemetry {
    uint8_t type;            // Always 0x01
    uint8_t sequence;        // Sequence number
    
    // State
    uint8_t state;           // Current state machine state
    uint8_t battery_pct;     // Battery percentage 0-100
    
    // Location
    float latitude;          // GPS latitude
    float longitude;         // GPS longitude
    float heading;          // Compass heading (degrees)
    uint8_t gps_fix;         // GPS fix quality (0-3)
    
    // Sensors
    float temperature;       // DS18B20 temperature (°C)
    float distance;          // Ultrasonic distance (cm)
    float accel_x, accel_y, accel_z;  // MPU6050
    
    // Flags
    uint8_t fire_detected:1;
    uint8_t human_detected:1;
    uint8_t motion_detected:1;
    uint8_t obstacle_detected:1;
    uint8_t docked:1;
    uint8_t charging:1;
    uint8_t error_flags:2;
    
    // Timestamp
    uint32_t timestamp;      // Milliseconds since boot
    
    uint8_t checksum;
};
```

#### State Values

| Value | State | Description |
|-------|-------|-------------|
| 0 | PATROL | Normal patrol operation |
| 1 | RESEARCH | Research mode |
| 2 | ALERT | Hazard detected |
| 3 | RETURN_TO_BASE | Low battery return |
| 4 | DOCKING | AprilTag docking |
| 5 | DOCKED | Successfully docked |
| 6 | EMERGENCY | Critical error |
| 7 | SLEEP | Night mode |

#### Error Flags

| Bit | Error | Description |
|-----|-------|-------------|
| 0 | ERR_GPS | GPS failure |
| 1 | ERR_SENSOR | Sensor failure |

---

## Docking Command Protocol

### Pi Zero → ESP32-S3 (via UART)

Special command protocol for AprilTag docking.

#### Packet Format (6 bytes)

```
[HEADER] [TURN] [SPEED] [DOCKED] [CHECKSUM] [TAIL]
```

| Byte | Field | Range | Description |
|------|-------|-------|-------------|
| 0 | HEADER | 0xAA | Packet start |
| 1 | TURN | -100 to +100 | Turn rate |
| 2 | SPEED | 0-100 | Forward speed |
| 3 | DOCKED | 0 or 1 | Docked flag |
| 4 | CHECKSUM | uint8 | XOR(1,2,3) |
| 5 | TAIL | 0x55 | Packet end |

#### Command Rate

- **Update Rate**: 15 Hz (~66ms interval)
- **Timeout**: 500 ms (rover stops if no command received)

---

## Communication Timing

### Update Rates

| Data | Rate | Method |
|------|------|--------|
| Telemetry | 1 Hz | ESP-NOW |
| Commands | 10 Hz | ESP-NOW |
| Docking | 15 Hz | UART |
| Heartbeat | 0.5 Hz | ESP-NOW |

### Timeouts

| Condition | Timeout | Action |
|-----------|---------|--------|
| No telemetry | 5 seconds | Reconnect ESP-NOW |
| No command | 500 ms (docking) | Emergency stop |
| No heartbeat | 10 seconds | Enter EMERGENCY state |
| GPS invalid | 60 seconds | Use last known position |

---

## Checksum Calculation

All checksums use simple XOR:

```c
uint8_t calculate_checksum(uint8_t* data, size_t len) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
}
```

---

## Example Packets

### Telemetry Packet (hex)

```
AA 01 20 
5A 64 12345678 87654321 180 01 
2A 00 00000000 00000000 00000000 
0C 01 2D 00 00001234 
XX 55
```

### Command Packet (hex)

```
AA 02 01 02 50 00 XX 55
```

Decoded:
- HEADER: 0xAA
- TYPE: 0x02 (command)
- SEQ: 0x01
- CMD: 0x02 (FORWARD)
- PARAM: 0x50 (80% speed)
- CHECKSUM: XX
- TAIL: 0x55

---

## Notes

1. All multi-byte integers are sent little-endian
2. GPS coordinates are in decimal degrees (WGS84)
3. Temperature is in Celsius
4. Distance is in centimeters
5. Accelerometer values are in m/s²

---

*Document Version: 1.0*
*Last Updated: February 2026*

This document describes all communication protocols used in the STASIS rover system.

---

## Table of Contents

1. [ESP-NOW Protocol](#esp-now-protocol)
2. [UART Protocol](#uart-protocol)
3. [Command Format](#command-format)
4. [Telemetry Format](#telemetry-format)

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
| 0x01 | Rover → Base | Telemetry packet |
| 0x02 | Base → Rover | Command packet |
| 0x03 | Bidirectional | Heartbeat/keepalive |
| 0x04 | Base → Rover | Configuration update |

---

## UART Protocol

### Overview

UART is used for communication between ESP32-C3 (bridge) and Raspberry Pi Zero.

- **Baud Rate**: 115200
- **Data Bits**: 8
- **Parity**: None
- **Stop Bits**: 1

### Packet Framing

All UART messages use the following frame format:

```
[HEADER] [TYPE] [LENGTH] [PAYLOAD...] [CHECKSUM] [TAIL]
```

| Field | Size | Value | Description |
|-------|------|-------|-------------|
| HEADER | 1 | 0xAA | Packet start marker |
| TYPE | 1 | 0x01-0x04 | Message type |
| LENGTH | 1 | N | Payload length in bytes |
| PAYLOAD | N | varies | Message data |
| CHECKSUM | 1 | uint8 | XOR of all bytes except TAIL |
| TAIL | 1 | 0x55 | Packet end marker |

---

## Command Format

### Rover Commands (Base → Rover)

Commands sent from base station to rover via ESP-NOW.

#### Structure

```c
struct RoverCommand {
    uint8_t type;        // Always 0x02
    uint8_t sequence;    // Sequence number (0-255)
    uint8_t cmd_id;      // Command ID
    int16_t param;       // Command parameter
    uint8_t checksum;
};
```

#### Command IDs

| ID | Command | Parameter | Description |
|----|---------|-----------|-------------|
| 0x01 | STOP | 0 | Emergency stop |
| 0x02 | FORWARD | 0-100 | Move forward (speed) |
| 0x03 | BACKWARD | 0-100 | Move backward (speed) |
| 0x04 | LEFT | 0-100 | Turn left (angle/speed) |
| 0x05 | RIGHT | 0-100 | Turn right (angle/speed) |
| 0x06 | SET_STATE | enum | Set rover state |
| 0x07 | ENABLE_DOCKING | 0/1 | Enable/disable docking mode |
| 0x08 | SET_SPEED | 0-100 | Set max speed |
| 0x09 | RESET | 0 | Reset rover |
| 0x0A | GOTO_WAYPOINT | index | Navigate to waypoint |

---

## Telemetry Format

### Rover Telemetry (Rover → Base)

Telemetry sent from rover to base station.

#### Structure

```c
struct RoverTelemetry {
    uint8_t type;            // Always 0x01
    uint8_t sequence;        // Sequence number
    
    // State
    uint8_t state;           // Current state machine state
    uint8_t battery_pct;     // Battery percentage 0-100
    
    // Location
    float latitude;          // GPS latitude
    float longitude;         // GPS longitude
    float heading;          // Compass heading (degrees)
    uint8_t gps_fix;         // GPS fix quality (0-3)
    
    // Sensors
    float temperature;       // DS18B20 temperature (°C)
    float distance;          // Ultrasonic distance (cm)
    float accel_x, accel_y, accel_z;  // MPU6050
    
    // Flags
    uint8_t fire_detected:1;
    uint8_t human_detected:1;
    uint8_t motion_detected:1;
    uint8_t obstacle_detected:1;
    uint8_t docked:1;
    uint8_t charging:1;
    uint8_t error_flags:2;
    
    // Timestamp
    uint32_t timestamp;      // Milliseconds since boot
    
    uint8_t checksum;
};
```

#### State Values

| Value | State | Description |
|-------|-------|-------------|
| 0 | PATROL | Normal patrol operation |
| 1 | RESEARCH | Research mode |
| 2 | ALERT | Hazard detected |
| 3 | RETURN_TO_BASE | Low battery return |
| 4 | DOCKING | AprilTag docking |
| 5 | DOCKED | Successfully docked |
| 6 | EMERGENCY | Critical error |
| 7 | SLEEP | Night mode |

#### Error Flags

| Bit | Error | Description |
|-----|-------|-------------|
| 0 | ERR_GPS | GPS failure |
| 1 | ERR_SENSOR | Sensor failure |

---

## Docking Command Protocol

### Pi Zero → ESP32-S3 (via UART)

Special command protocol for AprilTag docking.

#### Packet Format (6 bytes)

```
[HEADER] [TURN] [SPEED] [DOCKED] [CHECKSUM] [TAIL]
```

| Byte | Field | Range | Description |
|------|-------|-------|-------------|
| 0 | HEADER | 0xAA | Packet start |
| 1 | TURN | -100 to +100 | Turn rate |
| 2 | SPEED | 0-100 | Forward speed |
| 3 | DOCKED | 0 or 1 | Docked flag |
| 4 | CHECKSUM | uint8 | XOR(1,2,3) |
| 5 | TAIL | 0x55 | Packet end |

#### Command Rate

- **Update Rate**: 15 Hz (~66ms interval)
- **Timeout**: 500 ms (rover stops if no command received)

---

## Communication Timing

### Update Rates

| Data | Rate | Method |
|------|------|--------|
| Telemetry | 1 Hz | ESP-NOW |
| Commands | 10 Hz | ESP-NOW |
| Docking | 15 Hz | UART |
| Heartbeat | 0.5 Hz | ESP-NOW |

### Timeouts

| Condition | Timeout | Action |
|-----------|---------|--------|
| No telemetry | 5 seconds | Reconnect ESP-NOW |
| No command | 500 ms (docking) | Emergency stop |
| No heartbeat | 10 seconds | Enter EMERGENCY state |
| GPS invalid | 60 seconds | Use last known position |

---

## Checksum Calculation

All checksums use simple XOR:

```c
uint8_t calculate_checksum(uint8_t* data, size_t len) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
}
```

---

## Example Packets

### Telemetry Packet (hex)

```
AA 01 20 
5A 64 12345678 87654321 180 01 
2A 00 00000000 00000000 00000000 
0C 01 2D 00 00001234 
XX 55
```

### Command Packet (hex)

```
AA 02 01 02 50 00 XX 55
```

Decoded:
- HEADER: 0xAA
- TYPE: 0x02 (command)
- SEQ: 0x01
- CMD: 0x02 (FORWARD)
- PARAM: 0x50 (80% speed)
- CHECKSUM: XX
- TAIL: 0x55

---

## Notes

1. All multi-byte integers are sent little-endian
2. GPS coordinates are in decimal degrees (WGS84)
3. Temperature is in Celsius
4. Distance is in centimeters
5. Accelerometer values are in m/s²

---

*Document Version: 1.0*
*Last Updated: February 2026*

