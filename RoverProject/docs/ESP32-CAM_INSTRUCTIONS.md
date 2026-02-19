# ESP32-CAM Vision Module - Flashing Instructions

## Overview

The ESP32-CAM handles all vision-related tasks for the rover:
- Image capture and processing
- Fire detection (multi-algorithm color and pattern analysis)
- Motion detection (frame differencing with adaptive threshold)
- Human detection (skin-tone and shape analysis)
- Smoke detection (haze and color analysis)
- Animal detection (shape and movement patterns)
- Night vision mode with IR LED control
- Frame streaming on demand
- Configurable detection zones
- Multiple detection sensitivity levels

---

## Hardware Requirements

### Components
- ESP32-CAM Module (AI Thinker version recommended)
- FTDI Programmer / USB-TTL Serial Adapter (3.3V compatible)
- Jumper wires
- 5V power supply (for testing)

### ESP32-CAM Specifications
- Flash Memory: 4MB
- PSRAM: 4MB (external)
- Camera: OV2640 (2MP)
- USB: **No native USB** - requires external programmer

---

## Flashing Methods

### Method 1: Arduino IDE (Recommended)

#### Step 1: Install Arduino IDE

Download from: https://www.arduino.cc/en/software

#### Step 2: Add ESP32 Board Support

1. Go to **File** → **Preferences**
2. Add to "Additional Boards Manager URLs":
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
3. Go to **Tools** → **Board** → **Boards Manager**
4. Install **esp32 by Espressif Systems**

#### Step 3: Select ESP32-CAM Board

1. Go to **Tools** → **Board** → **esp32**
2. Select **AI Thinker ESP32-CAM**

#### Step 4: Configure Board Settings

| Setting | Value |
|---------|-------|
| Board | AI Thinker ESP32-CAM |
| Upload Speed | 115200 |
| CPU Frequency | 240MHz (WiFi/BT) |
| Flash Mode | QIO |
| Flash Size | 4MB |
| Partition Scheme | Huge App (3MB No OTA) |
| Core Debug Level | None |

---

## Wiring for Flashing

### FTDI/USB-TTL Connection

The ESP32-CAM does not have a USB port. You must use an FTDI programmer:

```
ESP32-CAM          FTDI Programmer
----------         ---------------
  U0R (RX)  <------  TX
  U0T (TX)  ------>  RX
  GND       -------  GND
  5V        <------- 5V (or 3.3V)
  
  IO0       -------  GND (for flash mode only!)
```

### Important Flash Mode Setup

**To enter flash mode:**
1. Connect **IO0 to GND** (this puts ESP32-CAM in download mode)
2. Power on the ESP32-CAM
3. Upload the code
4. Disconnect **IO0 from GND**
5. Press the **RST** (reset) button or repower

### Wiring Diagram

```
                    ┌─────────────────┐
                    │   FTDI/TTL      │
                    │   Programmer    │
                    │                 │
                    │  5V  ───────────┼─────┐
                    │  GND ───────────┼──┐  │
                    │  TX  ───────────┼──┼──┼──┐
                    │  RX  ───────────┼──┼──┼──┼──┐
                    └─────────────────┘  │  │  │  │
                                         │  │  │  │
                    ┌─────────────────┐  │  │  │  │
                    │   ESP32-CAM     │  │  │  │  │
                    │                 │  │  │  │  │
           5V  <────┼─ 5V             │  │  │  │  │
          GND  <────┼─ GND            │  │  │  │  │
          RX   <────┼─ U0R            │◄─┘  │  │  │
          TX   ─────┼─ U0T            │────►│  │  │
                    │                 │     │  │  │
                    │  IO0 ───────────┼─────┼──┼──┼── GND (flash mode)
                    │                 │     │  │  │
                    └─────────────────┘     │  │  │
                                            │  │  │
                    (Disconnect IO0 from    │  │  │
                     GND after flashing)    │  │  │
```

---

## Firmware Upload

### Step 1: Wire the ESP32-CAM

1. Connect FTDI programmer to ESP32-CAM as shown above
2. **Connect IO0 to GND** (critical for flash mode)
3. Plug FTDI into computer USB

### Step 2: Select Port

1. Go to **Tools** → **Port**
2. Select the FTDI COM port

### Step 3: Upload

1. Open `RoverProject/cam_firmware/cam_firmware.ino`
2. Click **Verify** to compile
3. Click **Upload**
4. Wait for "Done uploading"

### Troubleshooting Upload

**"Failed to connect to ESP32-CAM"**
- Ensure IO0 is connected to GND
- Press and hold the small RST button on ESP32-CAM
- Try a different baud rate (115200 or 921600)
- Check FTDI drivers are installed

**"Brownout detector triggered"**
- Use a better 5V power supply
- The ESP32-CAM needs more current during flash
- Add a 100µF capacitor between 5V and GND

**"Invalid header" or "Wrong boot mode"**
- IO0 must be connected to GND during upload
- Disconnect camera module during flashing (optional)

---

## Post-Flash Setup

### Step 1: Disconnect IO0 from GND

After successful upload:
1. Disconnect the wire from IO0 to GND
2. Press the RST button or repower

### Step 2: Verify Operation

1. Open Serial Monitor at **115200** baud
2. You should see: `CAM:READY`
3. If you see `CAM:ERROR`, check camera ribbon cable connection

---

## Camera Module Connection

### OV2640 Camera Ribbon Cable

The camera ribbon cable must be inserted correctly:

1. **Unlock** the connector by pulling the black tab up slightly
2. Insert the ribbon cable with **contacts facing down** (toward the board)
3. **Lock** the connector by pushing the black tab down

```
        ┌──────────────────┐
        │   ESP32-CAM      │
        │                  │
        │  ┌────────────┐  │
        │  │ CAMERA     │  │
        │  │ CONNECTOR  │  │
        │  │   ║║║║║║║   │  │  ← Ribbon cable inserted here
        │  └────────────┘  │
        │                  │
        └──────────────────┘
```

---

## Pin Connections Reference

### ESP32-CAM to ESP32-S3 (Rover Main)

| ESP32-CAM Pin | ESP32-S3 GPIO | Function |
|---------------|---------------|----------|
| U0T (TX) | GPIO 17 (RX) | UART TX to S3 |
| U0R (RX) | GPIO 18 (TX) | UART RX from S3 |
| 5V | 5V (Buck Converter) | Power |
| GND | GND | Common Ground |

### ESP32-CAM Camera Pins (Internal)

| Pin | GPIO | Function |
|-----|------|----------|
| D0 | 5 | Camera Data Bit 0 |
| D1 | 18 | Camera Data Bit 1 |
| D2 | 19 | Camera Data Bit 2 |
| D3 | 21 | Camera Data Bit 3 |
| D4 | 36 | Camera Data Bit 4 |
| D5 | 39 | Camera Data Bit 5 |
| D6 | 34 | Camera Data Bit 6 |
| D7 | 35 | Camera Data Bit 7 |
| XCLK | 0 | Camera Clock |
| PCLK | 22 | Pixel Clock |
| VSYNC | 25 | Vertical Sync |
| HREF | 23 | Horizontal Reference |
| SDA | 26 | I2C Data |
| SCL | 27 | I2C Clock |

---

## Communication Protocol

### Commands from ESP32-S3

| Command | Code | Description |
|---------|------|-------------|
| CAPTURE | 0x01 | Take single image and return detection result |
| STREAM_ON | 0x02 | Start continuous frame streaming |
| STREAM_OFF | 0x03 | Stop streaming |
| DETECT | 0x04 | Run detection and return result |
| NIGHT_MODE | 0x05 | Toggle night vision mode |
| SET_SENSITIVITY | 0x06 | Set detection sensitivity (0-100) |
| SET_ZONES | 0x07 | Configure detection zones |
| GET_STATUS | 0x08 | Return module status |
| CALIBRATE | 0x09 | Run sensor calibration |
| SET_RESOLUTION | 0x0A | Change camera resolution |
| ENABLE_FLASH | 0x0B | Enable flash LED |
| DISABLE_FLASH | 0x0C | Disable flash LED |
| GET_TEMPERATURE | 0x0D | Get module temperature |
| RESET_COUNTERS | 0x0E | Reset detection counters |
| GET_FRAME | 0x0F | Get raw frame data |
| SET_EXPOSURE | 0x10 | Set exposure level |
| SET_GAIN | 0x11 | Set gain level |
| SAVE_CONFIG | 0x12 | Save configuration to EEPROM |
| LOAD_CONFIG | 0x13 | Load configuration from EEPROM |
| FACTORY_RESET | 0x14 | Reset all settings to defaults |

### Response Format

The ESP32-CAM sends back a `DetectionResult` struct:

```cpp
struct DetectionResult {
  bool    fire;              // Fire detected
  bool    motion;            // Motion detected
  bool    human;             // Human detected
  bool    smoke;             // Smoke detected
  bool    animal;            // Animal detected
  float   confidence;        // Overall detection confidence (0-100)
  float   fireConfidence;    // Fire detection confidence
  float   motionConfidence;  // Motion detection confidence
  float   humanConfidence;   // Human detection confidence
  float   smokeConfidence;   // Smoke detection confidence
  int16_t fireCenterX;       // Fire center X coordinate
  int16_t fireCenterY;       // Fire center Y coordinate
  int16_t motionCenterX;     // Motion center X coordinate
  int16_t motionCenterY;     // Motion center Y coordinate
  int16_t humanCenterX;      // Human center X coordinate
  int16_t humanCenterY;      // Human center Y coordinate
  int16_t smokeCenterX;      // Smoke center X coordinate
  int16_t smokeCenterY;      // Smoke center Y coordinate
  char    description[32];   // Human-readable description
  uint32_t frameCount;       // Frame counter
  uint16_t processingTime;   // Processing time in ms
  uint8_t  detectionFlags;   // Bit flags for detections
  uint16_t firePixelCount;   // Fire pixel count
  uint16_t motionPixelCount; // Motion pixel count
  uint16_t humanPixelCount;  // Human pixel count
  uint16_t smokePixelCount;  // Smoke pixel count
};
```

### Text Responses

- `HAZARD:FIRE` - Fire detected
- `HAZARD:MOTION` - Motion detected
- `HAZARD:HUMAN` - Human detected
- `HAZARD:SMOKE` - Smoke detected
- `HAZARD:ANIMAL` - Animal detected
- `CLEAR` - No hazards detected
- `CAM:READY` - Camera initialized successfully
- `CAM:ERROR` - Camera initialization failed

### JSON Response Format

Each detection also outputs detailed JSON:

```json
{
  "fire": 0,
  "motion": 1,
  "human": 0,
  "smoke": 0,
  "animal": 0,
  "confidence": 45.2,
  "fireX": -1,
  "fireY": -1,
  "motionX": 160,
  "motionY": 120,
  "humanX": -1,
  "humanY": -1,
  "frameCount": 1234,
  "processTime": 15,
  "pixels": {
    "fire": 0,
    "motion": 45,
    "human": 0,
    "smoke": 0
  }
}
```

---

## Detection Features

### Fire Detection
- Multi-algorithm approach using color and pattern analysis
- RGB565 color space analysis for fire colors (high red, medium green, low blue)
- Center point calculation for fire location
- Confidence scoring based on pixel count

### Motion Detection
- Frame differencing with adaptive threshold
- Previous frame comparison for movement detection
- Configurable sensitivity levels (0-100)
- Motion center point tracking

### Human Detection
- Skin-tone color analysis in RGB565 space
- Simplified pattern recognition
- Higher threshold to reduce false positives

### Smoke Detection
- Gray-scale analysis for haze detection
- Brightness variance calculation
- Combined with fire detection for enhanced alerts

### Animal Detection
- Movement pattern analysis
- Differentiated from human detection
- Combined with motion detection

---

## Detection Zones

The firmware supports up to 8 configurable detection zones:

```cpp
struct DetectionZone {
  uint16_t x1, y1;  // Top-left corner
  uint16_t x2, y2;  // Bottom-right corner
  bool     enabled;
  uint8_t  sensitivity;  // 0-100
  uint8_t  detectionMask; // Bit flags for which detections to run
  char     name[16];
};
```

Use command `0x07` (SET_ZONES) to configure zones via serial.

---

## Night Vision Mode

The firmware includes a night vision mode:

- Increased brightness setting
- Higher gain levels
- Extended exposure time
- Optional IR LED control (if connected)

Toggle with command `0x05` (NIGHT_MODE).

---

## Testing After Upload

### Serial Monitor Test

1. Open Serial Monitor at **115200** baud
2. Power on ESP32-CAM
3. Look for `CAM:READY` message

### Manual Command Test

Send these bytes via Serial Monitor (set to "HEX" mode):
- Send `01` (CAPTURE) - should return detection result
- Send `02` (STREAM_ON) - should return `STREAM:ON`
- Send `03` (STREAM_OFF) - should return `STREAM:OFF`
- Send `08` (GET_STATUS) - should return JSON status

### LED Indicators

| LED State | Meaning |
|-----------|---------|
| 3 blinks at startup | Camera initializing |
| Solid on briefly | Camera ready |
| Continuous fast blink | Camera error |
| Brief flash | Image captured |

---

## Power Requirements

| State | Current |
|-------|---------|
| Idle | ~80mA |
| Streaming | ~150mA |
| Flash LED on | ~300mA+ |

**Note**: The built-in flash LED can draw significant current. Use sparingly for battery operation.

---

## Troubleshooting

### Camera Not Initializing (CAM:ERROR)

1. Check ribbon cable connection
2. Ensure ribbon cable contacts face down
3. Try reseating the ribbon cable
4. Check for damaged ribbon cable

### Poor Image Quality

1. Clean the camera lens
2. Adjust lighting conditions
3. The OV2640 struggles in low light
4. Try night mode for low-light conditions

### Detection Not Working

1. Ensure adequate lighting
2. Fire detection needs bright orange/red colors
3. Motion detection needs contrast in scene
4. Adjust sensitivity with SET_SENSITIVITY command

### High Temperature Warning

If you see `WARN:HIGH_TEMP`:
1. Ensure adequate ventilation
2. Reduce streaming frequency
3. Disable flash LED if not needed

---

## Alternative: Espressif Flash Tool

If Arduino IDE doesn't work, you can use the official Espressif Flash Download Tool:

### Step 1: Compile to Binary

1. In Arduino IDE, enable "Show verbose output during compilation"
2. Compile the sketch
3. Find the `.bin` file path in the output

### Step 2: Download Flash Tool

Download from: https://www.espressif.com/en/support/download/other-tools

### Step 3: Configure

| Setting | Value |
|---------|-------|
| Chip Type | ESP32 |
| Work Mode | Develop |
| Load Address | 0x10000 |
| SPI Speed | 40MHz |
| SPI Mode | DIO |

### Step 4: Flash

1. Select the compiled `.bin` file
2. Set COM port
3. Click **START**

---

## Next Steps

After flashing the ESP32-CAM:
1. Connect to ESP32-S3 via UART
2. Flash the [ESP32-S3 rover](./ESP32-S3_INSTRUCTIONS.md)
3. Test camera commands from the rover
4. Review the complete [wiring diagram](./WIRING_DIAGRAMS.md)

---

*Document Version: 2.0*  
*Last Updated: February 2026*  
*Updated for expanded firmware with smoke/animal detection, night mode, and detection zones*
