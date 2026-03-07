# AprilTag Docking Guide

## Overview

AprilTag docking allows the Stasis rover to automatically align with a charging station using visual markers. This guide explains how to set up and use the AprilTag docking system.

## Architecture

The C3 acts as a **GPIO extender** for the RPi0W, connecting only to the bottom row of the RPi0W GPIO header (pins 2, 4, 6, 8, 10...). The docking data flow is:

```
ESP32-CAM → ESP32-S3 → ESP32-C3 → RPi0W → Processing → Commands → ESP32-S3
   (detect)    (relay)    (bridge)   (process)   (decide)     (execute)
```

### Key Components

| Component | Role | Connection |
|-----------|------|------------|
| ESP32-CAM | AprilTag detection | Connected to ESP32-S3 via UART2 |
| ESP32-S3 | Rover controller | Receives tag data, relays to C3 via ESP-NOW |
| ESP32-C3 | GPIO extender/bridge | Connects to RPi0W bottom row pins (8, 10) |
| RPi0W | Main processor | Processes AprilTag data, makes decisions |
| LCD 16x2 | Status display | Connected to C3 GPIO 4 (SDA), GPIO 5 (SCL) |

---

## Hardware Setup

### 1. AprilTag Placement

- **Tag Family:** tag36h11 (recommended)
- **Tag ID:** 0 (default for primary docking station)
- **Size:** 100mm x 100mm minimum for 1-2 meter detection
- **Mounting:** 
  - At charging station entrance
  - 30-50cm from ground level
  - Perpendicular to rover approach path
  - Ensure good lighting for detection

### 2. Printing AprilTags

Download AprilTags from: https://github.com/AprilRobotics/apriltag-imgs

**Recommended print settings:**
- High contrast (black on white)
- Matte paper to reduce glare
- Laminate for durability
- Border of at least 1 tag width around the tag

### 3. ESP32-CAM Configuration

The ESP32-CAM should be configured with the AprilTag detection firmware:

1. Flash `cam_firmware.ino` to ESP32-CAM
2. Configure UART2 connection to ESP32-S3:
   - TX → S3 GPIO 16 (RX2)
   - RX ← S3 GPIO 17 (TX2)
3. Set detection parameters in firmware

### 4. C3 to RPi0W Connection (Bottom Row Only)

```
RPi0W GPIO Header (Bottom Row - Outer Edge):
┌─────────────────────────────────────────┐
│ Pin 2  │ Pin 4  │ Pin 6  │ Pin 8  │ Pin 10 │
│  5V    │  5V    │  GND   │ TXD    │  RXD   │
│   │    │   │    │   │    │   │    │   │    │
│   └────┴───┴────┴───┴────┴───┴────┴───┘    │
│              │       │       │            │
│           GND      GPIO20  GPIO21         │
│           (C3)     (C3)    (C3)           │
└─────────────────────────────────────────┘
```

| ESP32-C3 | RPi Zero Pin | Function |
|----------|--------------|----------|
| GPIO 21 (TX) | Pin 10 (RXD) | UART TX to Pi |
| GPIO 20 (RX) | Pin 8 (TXD) | UART RX from Pi |
| 5V | Pin 2 (5V) | Power from Pi |
| GND | Pin 6 (GND) | Common Ground |

### 5. LCD Display Connection (to C3)

| ESP32-C3 | LCD Module | Function |
|----------|------------|----------|
| GPIO 4 | SDA | I2C Data |
| GPIO 5 | SCL | I2C Clock |
| 3.3V | VCC | Power |
| GND | GND | Ground |

---

## AprilTag Data Format

### Detection Data

The ESP32-CAM sends detected tag data to the ESP32-S3, which relays it to the C3:

```json
{
  "type": "apriltag",
  "id": 0,
  "cx": 160,
  "cy": 120,
  "distance": 1.5,
  "angle": 5.2
}
```

| Field | Type | Description |
|-------|------|-------------|
| type | string | Always "apriltag" |
| id | int | AprilTag ID (0-30 for tag36h11) |
| cx | int | Center X position (0-320 pixels) |
| cy | int | Center Y position (0-240 pixels) |
| distance | float | Estimated distance in meters |
| angle | float | Angle offset in degrees |

### Alignment Calculations

The RPi0W processes the tag data to determine alignment:

```python
# Image center is (160, 120) for 320x240 resolution
CENTER_X = 160
TOLERANCE = 20  # pixels

def calculate_alignment(cx):
    """Calculate turn direction based on tag position."""
    offset = cx - CENTER_X
    
    if abs(offset) < TOLERANCE:
        return "ALIGNED"
    elif offset < 0:
        return "TURN_LEFT"
    else:
        return "TURN_RIGHT"
```

---

## Docking Procedure

### Phase 1: Approach

1. RPi0W sends `DOCK` command to rover
2. Rover switches to docking mode
3. ESP32-CAM activates AprilTag detection
4. Rover moves slowly toward charging station

### Phase 2: Alignment

1. RPi0W receives AprilTag data via C3 bridge
2. Calculates alignment corrections:
   - `cx < 140`: Turn left
   - `cx > 180`: Turn right
   - `cx` in range 140-180: Aligned
3. Sends turn commands to rover
4. Repeats until aligned

### Phase 3: Final Approach

1. While aligned, move forward slowly
2. Monitor distance decreasing
3. Stop when distance < 0.3m
4. Fine-tune alignment if needed

### Phase 4: Docking Complete

1. Rover makes electrical contact with charger
2. Charging begins automatically
3. LCD displays "CHARGING"
4. Rover enters sleep mode

---

## Python Implementation

### station_monitor.py Functions

The base station monitor includes these AprilTag processing functions:

```python
from dataclasses import dataclass
from typing import Optional

@dataclass
class AprilTagData:
    """AprilTag detection data."""
    tag_id: int
    cx: int           # Center X (0-320)
    cy: int           # Center Y (0-240)
    distance: float   # Estimated distance in meters
    angle: float      # Angle offset in degrees

def process_apriltag_for_docking(tag_data: dict) -> dict:
    """
    Process AprilTag detection and calculate alignment commands.
    
    Args:
        tag_data: Dict with 'id', 'cx', 'cy', 'distance', 'angle'
    
    Returns:
        Dict with alignment information:
        {
            'turn_direction': 'LEFT' | 'RIGHT' | 'NONE',
            'turn_amount': float (0-100),
            'move_forward': bool,
            'aligned': bool,
            'distance': float
        }
    """
    CENTER_X = 160
    TOLERANCE = 20
    
    cx = tag_data.get('cx', 160)
    distance = tag_data.get('distance', 0)
    
    offset = cx - CENTER_X
    
    if abs(offset) < TOLERANCE:
        return {
            'turn_direction': 'NONE',
            'turn_amount': 0,
            'move_forward': True,
            'aligned': True,
            'distance': distance
        }
    elif offset < 0:
        return {
            'turn_direction': 'LEFT',
            'turn_amount': min(abs(offset) / 160 * 100, 100),
            'move_forward': False,
            'aligned': False,
            'distance': distance
        }
    else:
        return {
            'turn_direction': 'RIGHT',
            'turn_amount': min(offset / 160 * 100, 100),
            'move_forward': False,
            'aligned': False,
            'distance': distance
        }

def send_alignment_commands(alignment: dict, serial_conn) -> None:
    """
    Send alignment commands to rover via C3 bridge.
    
    Args:
        alignment: Dict from process_apriltag_for_docking()
        serial_conn: Serial connection to C3
    """
    if alignment['turn_direction'] == 'LEFT':
        cmd = f"TURN_LEFT:{alignment['turn_amount']:.0f}"
    elif alignment['turn_direction'] == 'RIGHT':
        cmd = f"TURN_RIGHT:{alignment['turn_amount']:.0f}"
    elif alignment['move_forward']:
        cmd = "FORWARD:30"
    else:
        cmd = "STOP"
    
    serial_conn.write(f"{cmd}\n".encode())

def send_lcd_command(serial_conn, line: int, text: str) -> None:
    """
    Send LCD display command to C3.
    
    Args:
        serial_conn: Serial connection to C3
        line: Line number (1 or 2)
        text: Text to display (max 16 chars)
    """
    cmd = f"LCD:L{line}:{text[:16]}\n"
    serial_conn.write(cmd.encode())

def display_docking_status(serial_conn, status: str, distance: float) -> None:
    """
    Show docking progress on LCD via C3.
    
    Args:
        serial_conn: Serial connection to C3
        status: Status string
        distance: Distance in meters
    """
    send_lcd_command(serial_conn, 1, f"{status}")
    
    # Create progress bar
    progress = max(0, min(5, int((2.0 - distance) / 2.0 * 5)))
    bar = '▓' * progress + '░' * (5 - progress)
    send_lcd_command(serial_conn, 2, f"D:{distance:.1f}m {bar}")

def handle_docking_sequence(serial_conn, tag_data: dict) -> bool:
    """
    Complete docking sequence handler.
    
    Args:
        serial_conn: Serial connection to C3
        tag_data: AprilTag detection data
    
    Returns:
        True if docking successful, False otherwise
    """
    alignment = process_apriltag_for_docking(tag_data)
    
    # Update LCD
    display_docking_status(serial_conn, "DOCKING", alignment['distance'])
    
    # Check if close enough
    if alignment['distance'] < 0.3 and alignment['aligned']:
        send_lcd_command(serial_conn, 1, "DOCKED!")
        send_lcd_command(serial_conn, 2, "CHARGING...")
        return True
    
    # Send alignment commands
    send_alignment_commands(alignment, serial_conn)
    
    return False
```

---

## LCD Display Messages

### During Docking

```
Line 1: DOCKING...
Line 2: D:0.8m ▓▓▓░░
```

### Status Messages

| Status | Line 1 | Line 2 |
|--------|--------|--------|
| Approaching | APPROACHING | D:X.Xm ▓░░░░ |
| Aligning | ALIGNING... | Turn: LEFT/RIGHT |
| Aligned | ALIGNED | D:X.Xm ▓▓▓░░ |
| Docked | DOCKED! | CHARGING... |
| Charging | CHARGING | Bat: XX% |
| Failed | DOCK_FAIL | No tag found |

---

## Troubleshooting

### AprilTag Not Detected

1. **Check lighting:** Ensure adequate illumination at docking station
2. **Check tag visibility:** Tag should be clean and undamaged
3. **Check camera angle:** ESP32-CAM should have clear view
4. **Check distance:** Tag may be too far or too close
5. **Check tag family:** Ensure firmware uses tag36h11

### Alignment Issues

1. **Oscillating left/right:** Increase TOLERANCE value
2. **Not centering:** Check camera mounting angle
3. **Slow response:** Reduce approach speed
4. **Overshooting:** Add deceleration near target

### Communication Issues

1. **No data from C3:** Check UART connections (pins 8, 10)
2. **No data from S3:** Check ESP-NOW pairing
3. **No data from CAM:** Check UART2 connections to S3

### LCD Not Working

1. **No display:** Check I2C connections (GPIO 4, 5)
2. **Wrong characters:** Check I2C address (0x27 or 0x3F)
3. **Dim display:** Check backlight connection

---

## Testing

### Manual Test

1. Place AprilTag 1 meter from rover
2. Run docking command from base station
3. Observe rover behavior:
   - Should detect tag
   - Should align to center
   - Should approach slowly
   - Should stop at 30cm

### LCD Test

```python
import serial

s = serial.Serial('/dev/serial0', 115200)

# Test LCD commands
s.write(b'LCD:CLEAR\n')
s.write(b'LCD:L1:Test Line 1\n')
s.write(b'LCD:L2:Test Line 2\n')
```

### AprilTag Simulation Test

```python
# Simulate AprilTag data
test_data = {
    'id': 0,
    'cx': 180,      # Slightly right of center
    'cy': 120,
    'distance': 1.5,
    'angle': 3.0
}

result = process_apriltag_for_docking(test_data)
print(result)
# Output: {'turn_direction': 'RIGHT', 'turn_amount': 12.5, ...}
```

---

## Advanced Configuration

### Multiple AprilTags

For multiple docking stations, use different tag IDs:

```python
DOCKING_STATIONS = {
    0: {'name': 'Station A', 'x': 0, 'y': 0},
    1: {'name': 'Station B', 'x': 10, 'y': 0},
    2: {'name': 'Station C', 'x': 0, 'y': 10},
}

def get_station_info(tag_id):
    return DOCKING_STATIONS.get(tag_id)
```

### Custom Alignment Parameters

```python
# Adjust these for your rover
ALIGNMENT_CONFIG = {
    'center_x': 160,        # Image center X
    'center_y': 120,        # Image center Y
    'tolerance': 20,        # Alignment tolerance (pixels)
    'min_distance': 0.3,    # Minimum approach distance (m)
    'max_distance': 2.0,    # Maximum detection distance (m)
    'approach_speed': 30,   # Approach speed (0-100)
    'turn_speed': 20,       # Turn speed (0-100)
}
```

---

## Related Documentation

- [ESP32-C3 Instructions](./ESP32-C3_INSTRUCTIONS.md) - C3 setup and configuration
- [ESP32-CAM Instructions](./ESP32-CAM_INSTRUCTIONS.md) - Camera firmware setup
- [ESP32-S3 Instructions](./ESP32-S3_INSTRUCTIONS.md) - Rover controller setup
- [Wiring Diagrams](./WIRING_DIAGRAMS.md) - Complete wiring reference
- [RPI0 Instructions](./RPI0_INSTRUCTIONS.md) - Raspberry Pi Zero setup

---

*Document Version: 1.0*  
*Last Updated: February 2026*  
*For GPIO extender architecture with bottom-row-only RPi0W connection*
## Overview

AprilTag docking allows the Stasis rover to automatically align with a charging station using visual markers. This guide explains how to set up and use the AprilTag docking system.

## Architecture

The C3 acts as a **GPIO extender** for the RPi0W, connecting only to the bottom row of the RPi0W GPIO header (pins 2, 4, 6, 8, 10...). The docking data flow is:

```
ESP32-CAM → ESP32-S3 → ESP32-C3 → RPi0W → Processing → Commands → ESP32-S3
   (detect)    (relay)    (bridge)   (process)   (decide)     (execute)
```

### Key Components

| Component | Role | Connection |
|-----------|------|------------|
| ESP32-CAM | AprilTag detection | Connected to ESP32-S3 via UART2 |
| ESP32-S3 | Rover controller | Receives tag data, relays to C3 via ESP-NOW |
| ESP32-C3 | GPIO extender/bridge | Connects to RPi0W bottom row pins (8, 10) |
| RPi0W | Main processor | Processes AprilTag data, makes decisions |
| LCD 16x2 | Status display | Connected to C3 GPIO 4 (SDA), GPIO 5 (SCL) |

---

## Hardware Setup

### 1. AprilTag Placement

- **Tag Family:** tag36h11 (recommended)
- **Tag ID:** 0 (default for primary docking station)
- **Size:** 100mm x 100mm minimum for 1-2 meter detection
- **Mounting:** 
  - At charging station entrance
  - 30-50cm from ground level
  - Perpendicular to rover approach path
  - Ensure good lighting for detection

### 2. Printing AprilTags

Download AprilTags from: https://github.com/AprilRobotics/apriltag-imgs

**Recommended print settings:**
- High contrast (black on white)
- Matte paper to reduce glare
- Laminate for durability
- Border of at least 1 tag width around the tag

### 3. ESP32-CAM Configuration

The ESP32-CAM should be configured with the AprilTag detection firmware:

1. Flash `cam_firmware.ino` to ESP32-CAM
2. Configure UART2 connection to ESP32-S3:
   - TX → S3 GPIO 16 (RX2)
   - RX ← S3 GPIO 17 (TX2)
3. Set detection parameters in firmware

### 4. C3 to RPi0W Connection (Bottom Row Only)

```
RPi0W GPIO Header (Bottom Row - Outer Edge):
┌─────────────────────────────────────────┐
│ Pin 2  │ Pin 4  │ Pin 6  │ Pin 8  │ Pin 10 │
│  5V    │  5V    │  GND   │ TXD    │  RXD   │
│   │    │   │    │   │    │   │    │   │    │
│   └────┴───┴────┴───┴────┴───┴────┴───┘    │
│              │       │       │            │
│           GND      GPIO20  GPIO21         │
│           (C3)     (C3)    (C3)           │
└─────────────────────────────────────────┘
```

| ESP32-C3 | RPi Zero Pin | Function |
|----------|--------------|----------|
| GPIO 21 (TX) | Pin 10 (RXD) | UART TX to Pi |
| GPIO 20 (RX) | Pin 8 (TXD) | UART RX from Pi |
| 5V | Pin 2 (5V) | Power from Pi |
| GND | Pin 6 (GND) | Common Ground |

### 5. LCD Display Connection (to C3)

| ESP32-C3 | LCD Module | Function |
|----------|------------|----------|
| GPIO 4 | SDA | I2C Data |
| GPIO 5 | SCL | I2C Clock |
| 3.3V | VCC | Power |
| GND | GND | Ground |

---

## AprilTag Data Format

### Detection Data

The ESP32-CAM sends detected tag data to the ESP32-S3, which relays it to the C3:

```json
{
  "type": "apriltag",
  "id": 0,
  "cx": 160,
  "cy": 120,
  "distance": 1.5,
  "angle": 5.2
}
```

| Field | Type | Description |
|-------|------|-------------|
| type | string | Always "apriltag" |
| id | int | AprilTag ID (0-30 for tag36h11) |
| cx | int | Center X position (0-320 pixels) |
| cy | int | Center Y position (0-240 pixels) |
| distance | float | Estimated distance in meters |
| angle | float | Angle offset in degrees |

### Alignment Calculations

The RPi0W processes the tag data to determine alignment:

```python
# Image center is (160, 120) for 320x240 resolution
CENTER_X = 160
TOLERANCE = 20  # pixels

def calculate_alignment(cx):
    """Calculate turn direction based on tag position."""
    offset = cx - CENTER_X
    
    if abs(offset) < TOLERANCE:
        return "ALIGNED"
    elif offset < 0:
        return "TURN_LEFT"
    else:
        return "TURN_RIGHT"
```

---

## Docking Procedure

### Phase 1: Approach

1. RPi0W sends `DOCK` command to rover
2. Rover switches to docking mode
3. ESP32-CAM activates AprilTag detection
4. Rover moves slowly toward charging station

### Phase 2: Alignment

1. RPi0W receives AprilTag data via C3 bridge
2. Calculates alignment corrections:
   - `cx < 140`: Turn left
   - `cx > 180`: Turn right
   - `cx` in range 140-180: Aligned
3. Sends turn commands to rover
4. Repeats until aligned

### Phase 3: Final Approach

1. While aligned, move forward slowly
2. Monitor distance decreasing
3. Stop when distance < 0.3m
4. Fine-tune alignment if needed

### Phase 4: Docking Complete

1. Rover makes electrical contact with charger
2. Charging begins automatically
3. LCD displays "CHARGING"
4. Rover enters sleep mode

---

## Python Implementation

### station_monitor.py Functions

The base station monitor includes these AprilTag processing functions:

```python
from dataclasses import dataclass
from typing import Optional

@dataclass
class AprilTagData:
    """AprilTag detection data."""
    tag_id: int
    cx: int           # Center X (0-320)
    cy: int           # Center Y (0-240)
    distance: float   # Estimated distance in meters
    angle: float      # Angle offset in degrees

def process_apriltag_for_docking(tag_data: dict) -> dict:
    """
    Process AprilTag detection and calculate alignment commands.
    
    Args:
        tag_data: Dict with 'id', 'cx', 'cy', 'distance', 'angle'
    
    Returns:
        Dict with alignment information:
        {
            'turn_direction': 'LEFT' | 'RIGHT' | 'NONE',
            'turn_amount': float (0-100),
            'move_forward': bool,
            'aligned': bool,
            'distance': float
        }
    """
    CENTER_X = 160
    TOLERANCE = 20
    
    cx = tag_data.get('cx', 160)
    distance = tag_data.get('distance', 0)
    
    offset = cx - CENTER_X
    
    if abs(offset) < TOLERANCE:
        return {
            'turn_direction': 'NONE',
            'turn_amount': 0,
            'move_forward': True,
            'aligned': True,
            'distance': distance
        }
    elif offset < 0:
        return {
            'turn_direction': 'LEFT',
            'turn_amount': min(abs(offset) / 160 * 100, 100),
            'move_forward': False,
            'aligned': False,
            'distance': distance
        }
    else:
        return {
            'turn_direction': 'RIGHT',
            'turn_amount': min(offset / 160 * 100, 100),
            'move_forward': False,
            'aligned': False,
            'distance': distance
        }

def send_alignment_commands(alignment: dict, serial_conn) -> None:
    """
    Send alignment commands to rover via C3 bridge.
    
    Args:
        alignment: Dict from process_apriltag_for_docking()
        serial_conn: Serial connection to C3
    """
    if alignment['turn_direction'] == 'LEFT':
        cmd = f"TURN_LEFT:{alignment['turn_amount']:.0f}"
    elif alignment['turn_direction'] == 'RIGHT':
        cmd = f"TURN_RIGHT:{alignment['turn_amount']:.0f}"
    elif alignment['move_forward']:
        cmd = "FORWARD:30"
    else:
        cmd = "STOP"
    
    serial_conn.write(f"{cmd}\n".encode())

def send_lcd_command(serial_conn, line: int, text: str) -> None:
    """
    Send LCD display command to C3.
    
    Args:
        serial_conn: Serial connection to C3
        line: Line number (1 or 2)
        text: Text to display (max 16 chars)
    """
    cmd = f"LCD:L{line}:{text[:16]}\n"
    serial_conn.write(cmd.encode())

def display_docking_status(serial_conn, status: str, distance: float) -> None:
    """
    Show docking progress on LCD via C3.
    
    Args:
        serial_conn: Serial connection to C3
        status: Status string
        distance: Distance in meters
    """
    send_lcd_command(serial_conn, 1, f"{status}")
    
    # Create progress bar
    progress = max(0, min(5, int((2.0 - distance) / 2.0 * 5)))
    bar = '▓' * progress + '░' * (5 - progress)
    send_lcd_command(serial_conn, 2, f"D:{distance:.1f}m {bar}")

def handle_docking_sequence(serial_conn, tag_data: dict) -> bool:
    """
    Complete docking sequence handler.
    
    Args:
        serial_conn: Serial connection to C3
        tag_data: AprilTag detection data
    
    Returns:
        True if docking successful, False otherwise
    """
    alignment = process_apriltag_for_docking(tag_data)
    
    # Update LCD
    display_docking_status(serial_conn, "DOCKING", alignment['distance'])
    
    # Check if close enough
    if alignment['distance'] < 0.3 and alignment['aligned']:
        send_lcd_command(serial_conn, 1, "DOCKED!")
        send_lcd_command(serial_conn, 2, "CHARGING...")
        return True
    
    # Send alignment commands
    send_alignment_commands(alignment, serial_conn)
    
    return False
```

---

## LCD Display Messages

### During Docking

```
Line 1: DOCKING...
Line 2: D:0.8m ▓▓▓░░
```

### Status Messages

| Status | Line 1 | Line 2 |
|--------|--------|--------|
| Approaching | APPROACHING | D:X.Xm ▓░░░░ |
| Aligning | ALIGNING... | Turn: LEFT/RIGHT |
| Aligned | ALIGNED | D:X.Xm ▓▓▓░░ |
| Docked | DOCKED! | CHARGING... |
| Charging | CHARGING | Bat: XX% |
| Failed | DOCK_FAIL | No tag found |

---

## Troubleshooting

### AprilTag Not Detected

1. **Check lighting:** Ensure adequate illumination at docking station
2. **Check tag visibility:** Tag should be clean and undamaged
3. **Check camera angle:** ESP32-CAM should have clear view
4. **Check distance:** Tag may be too far or too close
5. **Check tag family:** Ensure firmware uses tag36h11

### Alignment Issues

1. **Oscillating left/right:** Increase TOLERANCE value
2. **Not centering:** Check camera mounting angle
3. **Slow response:** Reduce approach speed
4. **Overshooting:** Add deceleration near target

### Communication Issues

1. **No data from C3:** Check UART connections (pins 8, 10)
2. **No data from S3:** Check ESP-NOW pairing
3. **No data from CAM:** Check UART2 connections to S3

### LCD Not Working

1. **No display:** Check I2C connections (GPIO 4, 5)
2. **Wrong characters:** Check I2C address (0x27 or 0x3F)
3. **Dim display:** Check backlight connection

---

## Testing

### Manual Test

1. Place AprilTag 1 meter from rover
2. Run docking command from base station
3. Observe rover behavior:
   - Should detect tag
   - Should align to center
   - Should approach slowly
   - Should stop at 30cm

### LCD Test

```python
import serial

s = serial.Serial('/dev/serial0', 115200)

# Test LCD commands
s.write(b'LCD:CLEAR\n')
s.write(b'LCD:L1:Test Line 1\n')
s.write(b'LCD:L2:Test Line 2\n')
```

### AprilTag Simulation Test

```python
# Simulate AprilTag data
test_data = {
    'id': 0,
    'cx': 180,      # Slightly right of center
    'cy': 120,
    'distance': 1.5,
    'angle': 3.0
}

result = process_apriltag_for_docking(test_data)
print(result)
# Output: {'turn_direction': 'RIGHT', 'turn_amount': 12.5, ...}
```

---

## Advanced Configuration

### Multiple AprilTags

For multiple docking stations, use different tag IDs:

```python
DOCKING_STATIONS = {
    0: {'name': 'Station A', 'x': 0, 'y': 0},
    1: {'name': 'Station B', 'x': 10, 'y': 0},
    2: {'name': 'Station C', 'x': 0, 'y': 10},
}

def get_station_info(tag_id):
    return DOCKING_STATIONS.get(tag_id)
```

### Custom Alignment Parameters

```python
# Adjust these for your rover
ALIGNMENT_CONFIG = {
    'center_x': 160,        # Image center X
    'center_y': 120,        # Image center Y
    'tolerance': 20,        # Alignment tolerance (pixels)
    'min_distance': 0.3,    # Minimum approach distance (m)
    'max_distance': 2.0,    # Maximum detection distance (m)
    'approach_speed': 30,   # Approach speed (0-100)
    'turn_speed': 20,       # Turn speed (0-100)
}
```

---

## Related Documentation

- [ESP32-C3 Instructions](./ESP32-C3_INSTRUCTIONS.md) - C3 setup and configuration
- [ESP32-CAM Instructions](./ESP32-CAM_INSTRUCTIONS.md) - Camera firmware setup
- [ESP32-S3 Instructions](./ESP32-S3_INSTRUCTIONS.md) - Rover controller setup
- [Wiring Diagrams](./WIRING_DIAGRAMS.md) - Complete wiring reference
- [RPI0 Instructions](./RPI0_INSTRUCTIONS.md) - Raspberry Pi Zero setup

---

*Document Version: 1.0*  
*Last Updated: February 2026*  
*For GPIO extender architecture with bottom-row-only RPi0W connection*
