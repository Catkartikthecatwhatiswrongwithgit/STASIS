# ESP32-C3 Base Station Bridge - Flashing Instructions

## Overview

The ESP32-C3 serves as the communication bridge between the rover and the base station. It handles:
- ESP-NOW wireless communication with the ESP32-S3 rover
- UART serial communication with the Raspberry Pi Zero
- Message queuing and reliability
- WiFi Access Point for status indication
- LED status indicators

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

---

## Pin Connections Reference

| Component | Pin | ESP32-C3 GPIO | Notes |
|-----------|-----|---------------|-------|
| Raspberry Pi Zero | TX | GPIO 21 | UART to Pi RX (Pin 10) |
| Raspberry Pi Zero | RX | GPIO 20 | UART from Pi TX (Pin 8) |
| Built-in LED | - | GPIO 8 | Status indicator |
| Power | 5V | - | From Pi 5V pin |
| Ground | GND | - | Common ground with Pi |

---

## Wiring to Raspberry Pi Zero

### Connection Diagram

```
ESP32-C3                    Raspberry Pi Zero (GPIO Header)
--------                    ------------------------------
  GPIO 21 (TX)  ──────────>  Pin 10 (GPIO 15 RXD)
  GPIO 20 (RX)  <──────────  Pin 8  (GPIO 14 TXD)
  5V / 3.3V     <──────────  Pin 1 or Pin 2 (3.3V or 5V)
  GND           ──────────>  Pin 6 (GND)
```

### Important Notes

1. **Power**: The ESP32-C3 can be powered from the Pi's 5V or 3.3V pin
2. **Level Shifting**: Not required - both ESP32-C3 and Pi are 3.3V logic
3. **Common Ground**: Essential for UART communication

---

## LED Status Indicators

| Pattern | Meaning |
|---------|---------|
| 3 blinks at startup | Initialization successful |
| Continuous fast blink | ESP-NOW initialization failed |
| 1 blink | Data received from rover |
| 2 blinks | Command sent successfully |
| 5 blinks | ESP-NOW reconnected |

---

## Testing After Upload

### Serial Monitor Test

1. Open Serial Monitor at **115200** baud
2. You should see: `Bridge ready`

### WiFi AP Test

1. On your phone/laptop, scan for WiFi networks
2. Connect to `AeroSentinel-Base` (password: `sentinel123`)
3. Open browser and go to `http://192.168.4.1`
4. You should see the status page

### UART Test with Pi

1. Connect ESP32-C3 to Raspberry Pi Zero
2. On the Pi, run: `python3 -c "import serial; s=serial.Serial('/dev/serial0', 115200); print(s.readline())"`
3. You should see heartbeat messages every 5 seconds

---

## Troubleshooting

### ESP-NOW Not Initializing
- Check WiFi is not in AP mode during ESP-NOW init
- Try resetting the board
- Check for interference from other 2.4GHz devices

### UART Communication Issues
- Verify TX/RX are not swapped
- Check baud rate matches (115200)
- Ensure common ground connection

### WiFi AP Not Visible
- Wait 30 seconds after boot for AP to start
- Check if ESP-NOW is using the same channel
- Try restarting the ESP32-C3

---

## Next Steps

After flashing the ESP32-C3:
1. Connect to Raspberry Pi Zero via UART
2. Set up the [Raspberry Pi Zero software](./RPI0_INSTRUCTIONS.md)
3. Flash the [ESP32-S3 rover](./ESP32-S3_INSTRUCTIONS.md) with the correct MAC address
4. Test end-to-end communication

---

*Document Version: 1.0*  
*Last Updated: February 2026*

## Overview

The ESP32-C3 serves as the communication bridge between the rover and the base station. It handles:
- ESP-NOW wireless communication with the ESP32-S3 rover
- UART serial communication with the Raspberry Pi Zero
- Message queuing and reliability
- WiFi Access Point for status indication
- LED status indicators

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

---

## Pin Connections Reference

| Component | Pin | ESP32-C3 GPIO | Notes |
|-----------|-----|---------------|-------|
| Raspberry Pi Zero | TX | GPIO 21 | UART to Pi RX (Pin 10) |
| Raspberry Pi Zero | RX | GPIO 20 | UART from Pi TX (Pin 8) |
| Built-in LED | - | GPIO 8 | Status indicator |
| Power | 5V | - | From Pi 5V pin |
| Ground | GND | - | Common ground with Pi |

---

## Wiring to Raspberry Pi Zero

### Connection Diagram

```
ESP32-C3                    Raspberry Pi Zero (GPIO Header)
--------                    ------------------------------
  GPIO 21 (TX)  ──────────>  Pin 10 (GPIO 15 RXD)
  GPIO 20 (RX)  <──────────  Pin 8  (GPIO 14 TXD)
  5V / 3.3V     <──────────  Pin 1 or Pin 2 (3.3V or 5V)
  GND           ──────────>  Pin 6 (GND)
```

### Important Notes

1. **Power**: The ESP32-C3 can be powered from the Pi's 5V or 3.3V pin
2. **Level Shifting**: Not required - both ESP32-C3 and Pi are 3.3V logic
3. **Common Ground**: Essential for UART communication

---

## LED Status Indicators

| Pattern | Meaning |
|---------|---------|
| 3 blinks at startup | Initialization successful |
| Continuous fast blink | ESP-NOW initialization failed |
| 1 blink | Data received from rover |
| 2 blinks | Command sent successfully |
| 5 blinks | ESP-NOW reconnected |

---

## Testing After Upload

### Serial Monitor Test

1. Open Serial Monitor at **115200** baud
2. You should see: `Bridge ready`

### WiFi AP Test

1. On your phone/laptop, scan for WiFi networks
2. Connect to `AeroSentinel-Base` (password: `sentinel123`)
3. Open browser and go to `http://192.168.4.1`
4. You should see the status page

### UART Test with Pi

1. Connect ESP32-C3 to Raspberry Pi Zero
2. On the Pi, run: `python3 -c "import serial; s=serial.Serial('/dev/serial0', 115200); print(s.readline())"`
3. You should see heartbeat messages every 5 seconds

---

## Troubleshooting

### ESP-NOW Not Initializing
- Check WiFi is not in AP mode during ESP-NOW init
- Try resetting the board
- Check for interference from other 2.4GHz devices

### UART Communication Issues
- Verify TX/RX are not swapped
- Check baud rate matches (115200)
- Ensure common ground connection

### WiFi AP Not Visible
- Wait 30 seconds after boot for AP to start
- Check if ESP-NOW is using the same channel
- Try restarting the ESP32-C3

---

## Next Steps

After flashing the ESP32-C3:
1. Connect to Raspberry Pi Zero via UART
2. Set up the [Raspberry Pi Zero software](./RPI0_INSTRUCTIONS.md)
3. Flash the [ESP32-S3 rover](./ESP32-S3_INSTRUCTIONS.md) with the correct MAC address
4. Test end-to-end communication

---

*Document Version: 1.0*  
*Last Updated: February 2026*

