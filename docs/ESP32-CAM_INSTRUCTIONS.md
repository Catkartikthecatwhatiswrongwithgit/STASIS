# ESP32-CAM Vision Module - Flashing Instructions

## Overview

The ESP32-CAM handles all vision-related tasks for the rover:
- Image capture and processing
- Fire detection (color + brightness analysis)
- Motion detection (frame differencing)
- Human detection (skin-tone pattern analysis)
- Frame streaming on demand

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

### ESP32-CAM Camera Pins (Internal - AI Thinker)

| Pin | GPIO | Function |
|-----|------|----------|
| D0 | 5 | Camera Data Bit 0 (Y2) |
| D1 | 18 | Camera Data Bit 1 (Y3) |
| D2 | 19 | Camera Data Bit 2 (Y4) |
| D3 | 21 | Camera Data Bit 3 (Y5) |
| D4 | 36 | Camera Data Bit 4 (Y6) |
| D5 | 39 | Camera Data Bit 5 (Y7) |
| D6 | 34 | Camera Data Bit 6 (Y8) |
| D7 | 35 | Camera Data Bit 7 (Y9) |
| XCLK | 0 | Camera Clock |
| PCLK | 22 | Pixel Clock |
| VSYNC | 25 | Vertical Sync |
| HREF | 23 | Horizontal Reference |
| SDA | 26 | I2C Data (SIOD) |
| SCL | 27 | I2C Clock (SIOC) |
| PWDN | 32 | Power Down |
| RESET | -1 | Reset (not connected) |

### Status LED

| Pin | GPIO | Function |
|-----|------|----------|
| LED | 4 | Status LED (built-in) |

---

## Communication Protocol

### Commands from ESP32-S3

| Command | Code | Description |
|---------|------|-------------|
| `CMD_CAPTURE` | 0x01 | Take single image and return detection result |
| `CMD_STREAM_ON` | 0x02 | Start continuous frame streaming |
| `CMD_STREAM_OFF` | 0x03 | Stop streaming |
| `CMD_DETECT` | 0x04 | Run detection and return result |

### Response Format

The ESP32-CAM sends back a `DetectionResult` struct:

```cpp
typedef struct __attribute__((packed)) {
  bool  fire;        // Fire detected
  bool  motion;      // Motion detected
  bool  human;       // Human detected
  float confidence;  // Detection confidence (0-100)
  char  description[24];  // Human-readable description
} DetectionResult;
```

### Text Responses

- `HAZARD:FIRE` - Fire detected
- `HAZARD:MOTION` - Motion detected
- `HAZARD:HUMAN` - Human detected
- `CLEAR` - No hazards detected
- `CAM:READY` - Camera initialized successfully
- `CAM:ERROR` - Camera initialization failed

---

## Detection Features

### Fire Detection
- RGB565 color space analysis for fire colors
- Thresholds:
  - Red channel minimum: 25
  - Green channel minimum: 10
  - Blue channel maximum: 8
- Minimum fire pixels to trigger: 15

### Motion Detection
- Frame differencing with adaptive threshold
- Previous frame comparison for movement detection
- Motion sensitivity threshold: 4000 (pixel difference)
- Minimum motion pixels to trigger: 30

### Human Detection
- Skin-tone color analysis in RGB565 space
- Simplified pattern recognition
- Higher threshold to reduce false positives

---

## Detection Thresholds (Configurable)

```cpp
#define FIRE_PIXEL_THRESHOLD    15    // Minimum fire pixels to trigger
#define MOTION_PIXEL_THRESHOLD  30    // Minimum motion pixels to trigger
#define MOTION_SENSITIVITY      4000  // Pixel difference threshold
#define FIRE_R_MIN              25    // Red channel minimum for fire
#define FIRE_G_MIN              10    // Green channel minimum for fire
#define FIRE_B_MAX              8     // Blue channel maximum for fire
```

---

## Camera Configuration

The camera is configured for optimal detection performance:

| Setting | Value | Description |
|---------|-------|-------------|
| Pixel Format | RGB565 | Efficient for color analysis |
| Frame Size | QVGA (320x240) | Faster processing |
| JPEG Quality | 12 | Compression quality |
| Frame Buffer Count | 2 | Double buffer for streaming |
| XCLK Frequency | 20MHz | Camera clock speed |

---

## Testing After Upload

### Serial Monitor Test

1. Open Serial Monitor at **115200** baud
2. Power on ESP32-CAM
3. Look for `CAM:READY` message

### Manual Command Test

Send these bytes via Serial Monitor (set to "HEX" mode):
- Send `01` (CAPTURE) - should return detection result
- Send `02` (STREAM_ON) - should start streaming
- Send `03` (STREAM_OFF) - should stop streaming
- Send `04` (DETECT) - should return detection result

### LED Indicators

| LED State | Meaning |
|-----------|---------|
| Blinks at startup | Camera initializing |
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
4. Ensure adequate lighting for detection

### Detection Not Working

1. Ensure adequate lighting
2. Fire detection needs bright orange/red colors
3. Motion detection needs contrast in scene
4. Adjust thresholds in firmware if needed

### Brownout During Operation

1. Ensure stable 5V power supply
2. Add decoupling capacitor
3. Check current capacity of power source

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
1. Connect to ESP32-S3 via UART (GPIO 17/18)
2. Flash the [ESP32-S3 rover](./ESP32-S3_INSTRUCTIONS.md)
3. Test camera commands from the rover
4. Review the complete [wiring diagram](./WIRING_DIAGRAMS.md)

---

*Document Version: 2.1*  
*Last Updated: February 2026*
