# ESP32-C3 Base Station Bridge - Flashing Instructions

## Overview

The ESP32-C3 serves as a **GPIO Extender and Display Controller** for the Raspberry Pi Zero W, while also functioning as a communication bridge. It handles:
- ESP-NOW wireless communication with the ESP32-S3 rover
- UART serial communication with the Raspberry Pi Zero
- **LCD 16x2 I2C display control** (shows rover status, location, docking progress)
- **AprilTag data routing** (from ESP32-CAM → S3 → C3 → RPi for processing)
- Message queuing and reliability
- WiFi Access Point for status indication
- LED status indicators
- Multi-rover support (up to 4 rovers)
- OTA update capability
- Diagnostic logging

### Key Architecture Change
The C3 now connects **only to the bottom row of the RPi0W GPIO header** (the row starting with the double 5V pins - pins 2, 4, 6, 8, 10...). This allows the RPi0W to maintain full control while the C3 acts as a GPIO extender for:
- LCD display output
- UART communication bridge
- AprilTag data routing

---

## Hardware Requirements

### Components
- ESP32-C3 Development Board (ESP32-C3-DEVKITC-02 or similar)
- USB-C or USB-Micro cable for programming
- Computer with Arduino IDE installed
- **LCD 16x2 I2C Display** (optional, for status display)

### Board Specifications
- Flash Memory: 4MB (typical)
- USB: Some boards have native USB, others use USB-Serial bridge

---

## Software Setup - Arduino IDE

### Step 1: Install Arduino IDE

Download and install Arduino IDE from:
- https://www.arduino.cc/en/software

### Step 2: Add ESP32 Board Support

1. Open Arduino IDE
2. Go to **File** → **Preferences**
3. In "Additional Boards Manager URLs", add:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
4. Go to **Tools** → **Board** → **Boards Manager**
5. Search for "esp32" and install **esp32 by Espressif Systems**

### Step 3: Select ESP32-C3 Board

1. Go to **Tools** → **Board** → **esp32**
2. Select **ESP32C3 Dev Module**

### Step 4: Configure Board Settings

| Setting | Value |
|---------|-------|
| Board | ESP32C3 Dev Module |
| Upload Speed | 921600 |
| USB CDC On Boot | Enabled |
| Partition Scheme | Default 4MB with spiffs |
| CPU Frequency | 160MHz |
| Flash Mode | QIO |
| Flash Size | 4MB |
| Flash Frequency | 80MHz |
| Core Debug Level | None |

---

## Required Libraries

### Install via Library Manager

| Library | Version | Purpose |
|---------|---------|---------|
| ArduinoJson | 6.x or 7.x | JSON serialization |
| LiquidCrystal_I2C | 1.1.4 | LCD display control |

### Built-in Libraries (No installation needed)
- WiFi
- ESP-NOW
- WebServer
- Wire (I2C)

---

## Firmware Upload

### Step 1: Connect ESP32-C3

1. Connect ESP32-C3 to computer via USB
2. Wait for drivers to install

### Step 2: Select Port

1. Go to **Tools** → **Port**
2. Select the appropriate COM port

### Step 3: Upload

1. Open `RoverProject/base_bridge/base_bridge.ino`
2. Click **Verify** to compile
3. Click **Upload**
4. Wait for "Done uploading"

### Boot Mode for ESP32-C3

If upload fails, try:
1. Hold **BOOT** button
2. Click **Upload** in Arduino IDE
3. Release **BOOT** when "Connecting..." appears
4. Press **RST** (Reset) button after upload completes

---

## Configuration Before Upload

### Required Changes in Code

Open `base_bridge.ino` and modify:

```cpp
// Line ~80: Set your rover's MAC address
uint8_t roverMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
// Replace with actual ESP32-S3 MAC address
```

### Finding ESP32-S3 MAC Address

To find the MAC address of your ESP32-S3 (rover):

1. Flash the ESP32-S3 with rover_main.ino
2. Open Serial Monitor at 115200 baud
3. The MAC address will be printed on startup
4. Update the `roverMac[]` array in base_bridge.ino

### Multi-Rover Configuration

The firmware supports up to 5 rovers. Add additional MAC addresses:

```cpp
// Add multiple rovers
uint8_t roverMacs[][6] = {
  {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xF1},  // Rover 1
  {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xF2},  // Rover 2
  {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xF3},  // Rover 3
};
int roverCount = 3;
```

---

## WiFi Access Point Feature

The ESP32-C3 creates a WiFi Access Point for status indication:

| Setting | Value |
|---------|-------|
| SSID | `AeroSentinel-Base` |
| Password | `sentinel123` |
| IP Address | 192.168.4.1 |
| Port | 80 |

### Status Page

Connect to the AP and open `http://192.168.4.1` in a browser to see:
- Connection status
- Packets received count
- Commands sent count
- ESP-NOW status
- Rover registry
- System diagnostics

### Web API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Status page (HTML) |
| `/api/status` | GET | JSON status |
| `/api/rover` | GET | List registered rovers |
| `/api/command` | POST | Send command to rover |
| `/api/config` | GET/POST | Get/set configuration |
| `/api/diagnostics` | GET | System diagnostics |

---

## Pin Connections

### ESP32-C3 to Raspberry Pi Zero (Bottom Row Only)

The C3 connects **only to the bottom row** of the RPi0W GPIO header (the row starting with double 5V - pins 2, 4, 6, 8, 10...):

```
RPi0W GPIO Header (Bottom Row - Outer Edge):
┌─────────────────────────────────────────┐
│ Pin 2  │ Pin 4  │ Pin 6  │ Pin 8  │ Pin 10 │ ...
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

### ESP32-C3 to LCD Display (I2C)

| ESP32-C3 | LCD Module | Function |
|----------|------------|----------|
| GPIO 4 | SDA | I2C Data |
| GPIO 5 | SCL | I2C Clock |
| 3.3V | VCC | Power |
| GND | GND | Ground |

**Note:** LCD I2C address is typically 0x27 or 0x3F. Check your module documentation.

### ESP32-C3 to Status LED (Optional)

| ESP32-C3 | LED | Function |
|----------|-----|----------|
| GPIO 8 | LED Anode | Status indicator (built-in on most boards) |
| GND | LED Cathode | Via 220Ω resistor (if external) |

---

## Communication Protocol

### ESP-NOW (Rover ↔ Bridge)

The bridge receives telemetry from the rover via ESP-NOW:

```cpp
typedef struct {
  uint32_t id;           // Packet ID
  float    temp;         // Temperature (°C)
  float    bat;          // Battery voltage
  float    lat;          // GPS latitude
  float    lng;          // GPS longitude
  bool     hazard;       // Hazard detected
  char     status[16];   // Rover status
  float    distance;     // Ultrasonic distance
  float    accelX;       // Accelerometer X
  float    accelY;       // Accelerometer Y
  uint8_t  fire;         // Fire detection
  uint8_t  motion;       // Motion detection
  uint8_t  human;        // Human detection
} TelemetryPacket;
```

### UART (Bridge ↔ Pi)

Data is forwarded to the Pi via UART at 115200 baud.

**Telemetry Format (JSON):**
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
  "accelY": 0.05,
  "fire": 0,
  "motion": 1,
  "human": 0
}
```

**Command Format:**
Commands from Pi to rover are sent as plain text:
- `START` - Start patrol
- `STOP` - Stop movement
- `RETURN` - Return to base
- `DOCK` - Initiate docking
- `CALIBRATE` - Calibrate sensors
- `STATUS` - Request status

### LCD Commands (Pi → C3)

The RPi0W can send LCD commands via UART:

| Command | Format | Description |
|---------|--------|-------------|
| Clear | `LCD:CLEAR` | Clear display |
| Line 1 | `LCD:L1:text` | Display text on line 1 |
| Line 2 | `LCD:L2:text` | Display text on line 2 |
| Backlight | `LCD:BL:0/1` | Turn backlight on/off |

---

## AprilTag Docking Guide

### Overview

AprilTag docking allows the rover to automatically align with a charging station using visual markers. The data flow is:

```
ESP32-CAM → ESP32-S3 → ESP32-C3 → RPi0W → Processing → Commands → ESP32-S3
   (detect)    (relay)    (bridge)   (process)   (decide)     (execute)
```

### Hardware Setup

1. **AprilTag Placement:**
   - Print an AprilTag (recommended: tag36h11 family, ID 0)
   - Size: 100mm x 100mm minimum for 1-2 meter detection
   - Mount at charging station entrance, 30-50cm from ground
   - Ensure good lighting for detection

2. **ESP32-CAM Configuration:**
   - Connected to ESP32-S3 via UART2
   - Runs AprilTag detection firmware
   - Sends tag data: `{id, cx, cy, distance, angle}`

### AprilTag Data Format

The ESP32-CAM sends detected tag data:

```json
{
  "type": "apriltag",
  "id": 0,
  "cx": 160,        // Center X (0-320 pixels)
  "cy": 120,        // Center Y (0-240 pixels)
  "distance": 1.5,  // Estimated distance in meters
  "angle": 5.2      // Angle offset in degrees
}
```

### Docking Procedure

1. **Approach Phase:**
   - RPi0W sends `DOCK` command
   - Rover switches to docking mode
   - ESP32-CAM activates AprilTag detection

2. **Alignment Phase:**
   - RPi0W receives AprilTag data via C3 bridge
   - Calculates alignment corrections:
     - `cx < 140`: Turn left
     - `cx > 180`: Turn right
     - `cx` in range 140-180: Aligned

3. **Approach Phase:**
   - While aligned, move forward slowly
   - Monitor distance decreasing
   - Stop when distance < 0.3m

4. **Docking Complete:**
   - Rover makes electrical contact
   - Charging begins
   - LCD displays "CHARGING"

### Python Processing Functions

The `station_monitor.py` includes these AprilTag functions:

```python
# Process AprilTag data for docking alignment
def process_apriltag_for_docking(tag_data: dict) -> dict:
    """
    Process AprilTag detection and calculate alignment commands.
    
    Args:
        tag_data: Dict with 'id', 'cx', 'cy', 'distance', 'angle'
    
    Returns:
        Dict with 'turn_direction', 'turn_amount', 'move_forward', 'aligned'
    """
    pass

# Send alignment commands to rover via C3 bridge
def send_alignment_commands(alignment: dict) -> None:
    """Send calculated alignment commands to the rover."""
    pass

# Update LCD display with docking status
def display_docking_status(status: str, progress: int) -> None:
    """Show docking progress on LCD via C3."""
    pass
```

### LCD Display During Docking

The LCD shows docking progress:

```
Line 1: DOCKING...
Line 2: Dist: 0.8m ▓▓▓░░
```

Status messages:
- `APPROACHING` - Moving toward AprilTag
- `ALIGNING` - Adjusting heading
- `DOCKED` - Successfully connected
- `CHARGING` - Battery charging
- `DOCK_FAIL` - Lost tag or timeout

---

## LED Status Indicators

| Pattern | Meaning |
|---------|---------|
| 1 blink | Data received from rover |
| 2 blinks | Command sent successfully |
| 3 blinks | Error in communication |
| 5 blinks | ESP-NOW reconnected |
| Long blink (500ms) | ESP-NOW ready |
| Rapid blink | Error state |

---

## Testing After Upload

### Serial Monitor Test

1. Open Serial Monitor at **115200** baud
2. You should see:
   ```
   ========================
   Aero Sentinel Base Station Bridge
   ========================
   MAC: XX:XX:XX:XX:XX:XX
   SSID: AeroSentinel-Base
   IP: 192.168.4.1
   Status page: http://192.168.4.1
   LCD: Initialized (0x27)
   ========================
   Bridge ready
   ```

### WiFi AP Test

1. On your phone/laptop, scan for WiFi networks
2. Connect to `AeroSentinel-Base` (password: `sentinel123`)
3. Open browser and go to `http://192.168.4.1`
4. You should see the status page with ESP-NOW status

### UART Test with Pi

1. Connect ESP32-C3 to Raspberry Pi Zero
2. On the Pi, run: `python3 -c "import serial; s=serial.Serial('/dev/serial0', 115200); print(s.readline())"`
3. You should see heartbeat messages every 5 seconds

### LCD Test

Send LCD commands via UART:
```python
import serial
s = serial.Serial('/dev/serial0', 115200)
s.write(b'LCD:CLEAR\n')
s.write(b'LCD:L1:Hello World\n')
s.write(b'LCD:L2:AeroSentinel\n')
```

### API Test

Test the REST API with curl:

```bash
# Get status
curl http://192.168.4.1/api/status

# Send command
curl -X POST -d "cmd=START" http://192.168.4.1/api/command

# Get diagnostics
curl http://192.168.4.1/api/diagnostics
```

---

## Troubleshooting

### ESP-NOW Not Initializing
- Check WiFi is in AP mode before ESP-NOW init
- Try resetting the board
- Check for interference from other 2.4GHz devices
- Ensure correct MAC address format

### UART Communication Issues
- Verify TX/RX are not swapped
- Check baud rate matches (115200)
- Ensure common ground connection
- Check serial cable quality

### WiFi AP Not Visible
- Wait 30 seconds after boot for AP to start
- Check if ESP-NOW is using the same channel
- Try restarting the ESP32-C3
- Verify WiFi antenna (if external)

### Upload Fails
- Hold BOOT button during upload start
- Check USB cable is data-capable (not charge-only)
- Try different USB port
- Install CP210x or CH340 drivers if needed

### Commands Not Reaching Rover
- Verify rover MAC address is correct
- Check ESP-NOW peer is registered
- Ensure rover is powered on and in range
- Check for RF interference

### LCD Not Working
- Check I2C connections (GPIO 4 = SDA, GPIO 5 = SCL)
- Verify LCD I2C address (0x27 or 0x3F)
- Run I2C scanner to detect address
- Check power connections (3.3V, GND)

---

## Advanced Features

### OTA Updates

The firmware supports Over-The-Air updates:

1. Connect to the AP
2. Use the `/api/update` endpoint
3. Upload new firmware binary

### Diagnostic Logging

View diagnostic logs via serial:

```
DIAG: Uptime=3600s
DIAG: Heap=45000 bytes
DIAG: Packets=1234
DIAG: Errors=5
DIAG: Rovers=2
DIAG: LCD=OK (0x27)
```

### Configuration Storage

Settings are stored in EEPROM and persist across reboots:
- Rover MAC addresses
- WiFi credentials
- UART baud rate
- LED patterns
- LCD I2C address

---

## Next Steps

After flashing the ESP32-C3:
1. Connect to Raspberry Pi Zero via UART (bottom row pins only)
2. Connect LCD display to C3 GPIO 4/5
3. Set up the [Raspberry Pi Zero software](./RPI0_INSTRUCTIONS.md)
4. Flash the [ESP32-S3 rover](./ESP32-S3_INSTRUCTIONS.md) with the correct MAC address
5. Configure [ESP32-CAM for AprilTag detection](./ESP32-CAM_INSTRUCTIONS.md)
6. Test end-to-end communication
7. Review the complete [wiring diagram](./WIRING_DIAGRAMS.md)

---

*Document Version: 2.2*  
*Last Updated: February 2026*  
*Updated for GPIO extender role with LCD display and AprilTag docking support*
The LCD shows docking progress:

```
Line 1: DOCKING...
Line 2: Dist: 0.8m ▓▓▓░░
```

Status messages:
- `APPROACHING` - Moving toward AprilTag
- `ALIGNING` - Adjusting heading
- `DOCKED` - Successfully connected
- `CHARGING` - Battery charging
- `DOCK_FAIL` - Lost tag or timeout

---

## LED Status Indicators

| Pattern | Meaning |
|---------|---------|
| 1 blink | Data received from rover |
| 2 blinks | Command sent successfully |
| 3 blinks | Error in communication |
| 5 blinks | ESP-NOW reconnected |
| Long blink (500ms) | ESP-NOW ready |
| Rapid blink | Error state |

---

## Testing After Upload

### Serial Monitor Test

1. Open Serial Monitor at **115200** baud
2. You should see:
   ```
   ========================
   Aero Sentinel Base Station Bridge
   ========================
   MAC: XX:XX:XX:XX:XX:XX
   SSID: AeroSentinel-Base
   IP: 192.168.4.1
   Status page: http://192.168.4.1
   LCD: Initialized (0x27)
   ========================
   Bridge ready
   ```

### WiFi AP Test

1. On your phone/laptop, scan for WiFi networks
2. Connect to `AeroSentinel-Base` (password: `sentinel123`)
3. Open browser and go to `http://192.168.4.1`
4. You should see the status page with ESP-NOW status

### UART Test with Pi

1. Connect ESP32-C3 to Raspberry Pi Zero
2. On the Pi, run: `python3 -c "import serial; s=serial.Serial('/dev/serial0', 115200); print(s.readline())"`
3. You should see heartbeat messages every 5 seconds

### LCD Test

Send LCD commands via UART:
```python
import serial
s = serial.Serial('/dev/serial0', 115200)
s.write(b'LCD:CLEAR\n')
s.write(b'LCD:L1:Hello World\n')
s.write(b'LCD:L2:AeroSentinel\n')
```

### API Test

Test the REST API with curl:

```bash
# Get status
curl http://192.168.4.1/api/status

# Send command
curl -X POST -d "cmd=START" http://192.168.4.1/api/command

# Get diagnostics
curl http://192.168.4.1/api/diagnostics
```

---

## Troubleshooting

### ESP-NOW Not Initializing
- Check WiFi is in AP mode before ESP-NOW init
- Try resetting the board
- Check for interference from other 2.4GHz devices
- Ensure correct MAC address format

### UART Communication Issues
- Verify TX/RX are not swapped
- Check baud rate matches (115200)
- Ensure common ground connection
- Check serial cable quality

### WiFi AP Not Visible
- Wait 30 seconds after boot for AP to start
- Check if ESP-NOW is using the same channel
- Try restarting the ESP32-C3
- Verify WiFi antenna (if external)

### Upload Fails
- Hold BOOT button during upload start
- Check USB cable is data-capable (not charge-only)
- Try different USB port
- Install CP210x or CH340 drivers if needed

### Commands Not Reaching Rover
- Verify rover MAC address is correct
- Check ESP-NOW peer is registered
- Ensure rover is powered on and in range
- Check for RF interference

### LCD Not Working
- Check I2C connections (GPIO 4 = SDA, GPIO 5 = SCL)
- Verify LCD I2C address (0x27 or 0x3F)
- Run I2C scanner to detect address
- Check power connections (3.3V, GND)

---

## Advanced Features

### OTA Updates

The firmware supports Over-The-Air updates:

1. Connect to the AP
2. Use the `/api/update` endpoint
3. Upload new firmware binary

### Diagnostic Logging

View diagnostic logs via serial:

```
DIAG: Uptime=3600s
DIAG: Heap=45000 bytes
DIAG: Packets=1234
DIAG: Errors=5
DIAG: Rovers=2
DIAG: LCD=OK (0x27)
```

### Configuration Storage

Settings are stored in EEPROM and persist across reboots:
- Rover MAC addresses
- WiFi credentials
- UART baud rate
- LED patterns
- LCD I2C address

---

## Next Steps

After flashing the ESP32-C3:
1. Connect to Raspberry Pi Zero via UART (bottom row pins only)
2. Connect LCD display to C3 GPIO 4/5
3. Set up the [Raspberry Pi Zero software](./RPI0_INSTRUCTIONS.md)
4. Flash the [ESP32-S3 rover](./ESP32-S3_INSTRUCTIONS.md) with the correct MAC address
5. Configure [ESP32-CAM for AprilTag detection](./ESP32-CAM_INSTRUCTIONS.md)
6. Test end-to-end communication
7. Review the complete [wiring diagram](./WIRING_DIAGRAMS.md)

---

*Document Version: 2.2*  
*Last Updated: February 2026*  
*Updated for GPIO extender role with LCD display and AprilTag docking support*
