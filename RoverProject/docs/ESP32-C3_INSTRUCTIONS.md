# ESP32-C3 Base Station Bridge - Flashing Instructions

## Overview

The ESP32-C3 serves as the communication bridge between the rover and the base station. It handles:
- ESP-NOW wireless communication with the ESP32-S3 rover
- UART serial communication with the Raspberry Pi Zero
- Message queuing and reliability
- WiFi Access Point for status indication
- LED status indicators
- Multi-rover support (up to 5 rovers)
- OTA update capability
- Diagnostic logging

---

## Hardware Requirements

### Components
- ESP32-C3 Development Board (ESP32-C3-DEVKITC-02 or similar)
- USB-C or USB-Micro cable for programming
- Computer with Arduino IDE installed

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

### Built-in Libraries (No installation needed)
- WiFi
- ESP-NOW
- WebServer

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

### ESP32-C3 to Raspberry Pi Zero

| ESP32-C3 | RPi Zero | Function |
|----------|----------|----------|
| GPIO 21 | GPIO 15 (RXD) | UART TX |
| GPIO 20 | GPIO 14 (TXD) | UART RX |
| GND | GND | Common Ground |
| 5V | 5V | Power (optional) |

### ESP32-C3 to Status LED (Optional)

| ESP32-C3 | LED | Function |
|----------|-----|----------|
| GPIO 2 | LED Anode | Status indicator |
| GND | LED Cathode | Via 220Ω resistor |

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
```

### Configuration Storage

Settings are stored in EEPROM and persist across reboots:
- Rover MAC addresses
- WiFi credentials
- UART baud rate
- LED patterns

---

## Next Steps

After flashing the ESP32-C3:
1. Connect to Raspberry Pi Zero via UART
2. Set up the [Raspberry Pi Zero software](./RPI0_INSTRUCTIONS.md)
3. Flash the [ESP32-S3 rover](./ESP32-S3_INSTRUCTIONS.md) with the correct MAC address
4. Test end-to-end communication
5. Review the complete [wiring diagram](./WIRING_DIAGRAMS.md)

---

*Document Version: 2.0*  
*Last Updated: February 2026*  
*Updated for expanded firmware with multi-rover support, OTA updates, and enhanced diagnostics*
