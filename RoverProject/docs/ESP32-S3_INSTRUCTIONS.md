# ESP32-S3 Rover Main Controller - Flashing Instructions

## Overview

The ESP32-S3 is the main controller for the rover unit. It handles:
- Navigation and motor control
- Sensor data collection (GPS, temperature, IMU, ultrasonic)
- Communication with ESP32-CAM (vision module)
- GSM/SMS alerts via SIM800
- ESP-NOW communication with base station
- State machine logic (Patrol, Research, Alert, Return Base, Docked)

---

## Hardware Requirements

### Components
- ESP32-S3 Development Board (ESP32-S3-DEVKITC or similar)
- USB-C cable for programming
- Computer with Arduino IDE installed

### Board Specifications
- Flash Memory: 8MB or 16MB (recommended)
- PSRAM: 8MB (recommended for image processing)
- USB: Native USB-C (no need for external USB-TTL adapter)

---

## Software Setup - Arduino IDE

### Step 1: Install Arduino IDE

Download and install Arduino IDE from:
- **Windows/Mac/Linux**: https://www.arduino.cc/en/software

### Step 2: Add ESP32 Board Support

1. Open Arduino IDE
2. Go to **File** → **Preferences**
3. In "Additional Boards Manager URLs", add:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
4. Click **OK**

5. Go to **Tools** → **Board** → **Boards Manager**
6. Search for "esp32"
7. Install **esp32 by Espressif Systems** (version 2.0.11 or later)

### Step 3: Select ESP32-S3 Board

1. Go to **Tools** → **Board** → **esp32**
2. Select **ESP32S3 Dev Module**

### Step 4: Configure Board Settings

Set the following options in **Tools** menu:

| Setting | Value |
|---------|-------|
| Board | ESP32S3 Dev Module |
| Upload Speed | 921600 |
| USB Mode | Hardware CDC and JTAG |
| USB CDC On Boot | Enabled |
| USB Firmware MSC On Boot | Disabled |
| Partition Scheme | Huge App (3MB No OTA/1MB SPIFFS) |
| CPU Frequency | 240MHz (WiFi/BT) |
| Flash Mode | QIO 80MHz |
| Flash Size | 8MB (or match your board) |
| Core Debug Level | None |
| PSRAM | OPI PSRAM (if available) |

---

## Required Libraries

### Install via Library Manager

Go to **Sketch** → **Include Library** → **Manage Libraries** and install:

| Library | Version | Purpose |
|---------|---------|---------|
| Adafruit MPU6050 | 2.x | IMU sensor |
| Adafruit Unified Sensor | 1.x | Sensor abstraction |
| TinyGPSPlus | 1.0.3 | GPS parsing |
| OneWire | 2.3.8 | DS18B20 temperature sensor |
| DallasTemperature | 3.9.0 | DS18B20 driver |
| ArduinoJson | 6.x or 7.x | JSON parsing |

### Libraries Included with ESP32 Core (No installation needed)
- WiFi
- ESP-NOW
- WebServer
- EEPROM
- HardwareSerial
- SoftwareSerial

---

## Firmware Upload

### Step 1: Connect ESP32-S3

1. Connect ESP32-S3 to computer via USB-C cable
2. Wait for drivers to install (Windows may need a moment)

### Step 2: Select Port

1. Go to **Tools** → **Port**
2. Select the COM port (e.g., COM3 on Windows, /dev/ttyUSB0 on Linux)

### Step 3: Open Firmware File

1. Go to **File** → **Open**
2. Navigate to `RoverProject/rover_main/rover_main.ino`

### Step 4: Upload

1. Click **Verify** (checkmark icon) to compile
2. If no errors, click **Upload** (arrow icon)
3. Wait for "Done uploading" message

### Troubleshooting Upload Issues

**Problem: "Failed to connect to ESP32-S3"**
- Hold the **BOOT** button on the ESP32-S3 while clicking Upload
- Release BOOT when "Connecting..." appears
- Some boards require this manual boot mode trigger

**Problem: "No port found"**
- Install CP210x or CH340 drivers (depending on your board's USB chip)
- Windows: Check Device Manager → Ports (COM & LPT)
- Linux: Run `sudo usermod -a -G dialout $USER` and reboot

**Problem: "Brownout detector triggered"**
- Use a better USB cable
- Connect external 5V power supply
- Disable brownout detection in code (not recommended)

---

## Configuration Before Upload

### Required Changes in Code

Open `rover_main.ino` and modify these values:

```cpp
// Line ~194: Set your base station MAC address
uint8_t baseMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 
// Replace with actual ESP32-C3 MAC address

// Lines ~206-211: Set default configuration
strcpy(config.phone1, "+1234567890");  // Your phone number
strcpy(config.phone2, "");              // Secondary number (optional)
config.baseLat = 12.9716;               // Base station latitude
config.baseLng = 77.5946;               // Base station longitude
config.alertTempHigh = 55.0;            // Temperature alert threshold
```

### Finding ESP32-C3 MAC Address

To find the MAC address of your ESP32-C3 (base station bridge):

1. Flash the ESP32-C3 with base_bridge.ino
2. Open Serial Monitor at 115200 baud
3. The MAC address will be printed on startup
4. Update the `baseMac[]` array in rover_main.ino with this address

---

## Pin Connections Reference

| Component | Pin | ESP32-S3 GPIO | Notes |
|-----------|-----|---------------|-------|
| L9110S Motor Driver | B-IA | GPIO 4 | PWM Speed - Left |
| L9110S Motor Driver | B-IB | GPIO 5 | Direction - Left |
| L9110S Motor Driver | A-IA | GPIO 6 | PWM Speed - Right |
| L9110S Motor Driver | A-IB | GPIO 7 | Direction - Right |
| ESP32-CAM | TX | GPIO 17 (RX) | UART1 |
| ESP32-CAM | RX | GPIO 18 (TX) | UART1 |
| SIM800L | TXD | GPIO 16 (RX) | UART2 |
| SIM800L | RXD | GPIO 15 (TX) | UART2 |
| Neo-6M GPS | TX | GPIO 8 (RX) | SoftwareSerial |
| Neo-6M GPS | RX | GPIO 19 (TX) | SoftwareSerial |
| MPU6050 | SDA | GPIO 42 | I2C |
| MPU6050 | SCL | GPIO 2 | I2C |
| Ultrasonic | Trig | GPIO 12 | - |
| Ultrasonic | Echo | GPIO 13 | Needs voltage divider |
| DS18B20 | Data | GPIO 21 | Needs 4.7k pull-up |
| Buzzer | + | GPIO 48 | Active buzzer |
| Battery Monitor | + | GPIO 1 (ADC) | Needs voltage divider |

---

## Testing After Upload

### Serial Monitor Test

1. Open Serial Monitor (**Tools** → **Serial Monitor**)
2. Set baud rate to **115200**
3. You should see: `STASIS Rover initialized`

### LED Indicators

- **Built-in LED**: Blinks on startup
- **No LED activity after init**: Normal operation

### Common Test Commands

Send these commands via ESP-NOW from base station:
- `FORWARD` - Move forward
- `BACKWARD` - Move backward
- `LEFT` - Turn left
- `RIGHT` - Turn right
- `STOP` - Stop motors

---

## Power Requirements

| Component | Voltage | Current (Peak) |
|-----------|---------|----------------|
| ESP32-S3 | 3.3V / 5V | 500mA |
| ESP32-CAM | 5V | 500mA (peak 2A with flash) |
| SIM800L | 3.4V-4.4V | 2A (peak) |
| Motors (4x) | 6V-12V | 2A (stall) |
| GPS | 3.3V-5V | 50mA |
| MPU6050 | 3.3V | 10mA |
| DS18B20 | 3.3V-5V | 5mA |

**Total estimated peak current: 5-6A**

Use a buck converter to step down battery voltage to 5V for ESP32 and peripherals.

---

## Next Steps

After flashing the ESP32-S3:
1. Flash the ESP32-CAM with [cam_firmware.ino](./ESP32-CAM_INSTRUCTIONS.md)
2. Flash the ESP32-C3 with [base_bridge.ino](./ESP32-C3_INSTRUCTIONS.md)
3. Set up the Raspberry Pi Zero with [station_monitor.py](./RPI0_INSTRUCTIONS.md)
4. Review the complete [wiring diagram](./WIRING_DIAGRAMS.md)

---

## Troubleshooting

### GPS Not Getting Fix
- Ensure GPS has clear view of sky
- Wait 2-5 minutes for cold start
- Check antenna connection

### MPU6050 Not Detected
- Check I2C connections (SDA, SCL)
- Verify 3.3V power supply
- Run I2C scanner sketch

### Motors Not Moving
- Check L9110S power supply
- Verify motor connections
- Test with simple PWM sketch

### SIM800 Not Responding
- Ensure adequate power supply (2A peak)
- Check SIM card is inserted
- Verify UART connections

---

*Document Version: 1.0*  
*Last Updated: February 2026*

## Overview

The ESP32-S3 is the main controller for the rover unit. It handles:
- Navigation and motor control
- Sensor data collection (GPS, temperature, IMU, ultrasonic)
- Communication with ESP32-CAM (vision module)
- GSM/SMS alerts via SIM800
- ESP-NOW communication with base station
- State machine logic (Patrol, Research, Alert, Return Base, Docked)

---

## Hardware Requirements

### Components
- ESP32-S3 Development Board (ESP32-S3-DEVKITC or similar)
- USB-C cable for programming
- Computer with Arduino IDE installed

### Board Specifications
- Flash Memory: 8MB or 16MB (recommended)
- PSRAM: 8MB (recommended for image processing)
- USB: Native USB-C (no need for external USB-TTL adapter)

---

## Software Setup - Arduino IDE

### Step 1: Install Arduino IDE

Download and install Arduino IDE from:
- **Windows/Mac/Linux**: https://www.arduino.cc/en/software

### Step 2: Add ESP32 Board Support

1. Open Arduino IDE
2. Go to **File** → **Preferences**
3. In "Additional Boards Manager URLs", add:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
4. Click **OK**

5. Go to **Tools** → **Board** → **Boards Manager**
6. Search for "esp32"
7. Install **esp32 by Espressif Systems** (version 2.0.11 or later)

### Step 3: Select ESP32-S3 Board

1. Go to **Tools** → **Board** → **esp32**
2. Select **ESP32S3 Dev Module**

### Step 4: Configure Board Settings

Set the following options in **Tools** menu:

| Setting | Value |
|---------|-------|
| Board | ESP32S3 Dev Module |
| Upload Speed | 921600 |
| USB Mode | Hardware CDC and JTAG |
| USB CDC On Boot | Enabled |
| USB Firmware MSC On Boot | Disabled |
| Partition Scheme | Huge App (3MB No OTA/1MB SPIFFS) |
| CPU Frequency | 240MHz (WiFi/BT) |
| Flash Mode | QIO 80MHz |
| Flash Size | 8MB (or match your board) |
| Core Debug Level | None |
| PSRAM | OPI PSRAM (if available) |

---

## Required Libraries

### Install via Library Manager

Go to **Sketch** → **Include Library** → **Manage Libraries** and install:

| Library | Version | Purpose |
|---------|---------|---------|
| Adafruit MPU6050 | 2.x | IMU sensor |
| Adafruit Unified Sensor | 1.x | Sensor abstraction |
| TinyGPSPlus | 1.0.3 | GPS parsing |
| OneWire | 2.3.8 | DS18B20 temperature sensor |
| DallasTemperature | 3.9.0 | DS18B20 driver |
| ArduinoJson | 6.x or 7.x | JSON parsing |

### Libraries Included with ESP32 Core (No installation needed)
- WiFi
- ESP-NOW
- WebServer
- EEPROM
- HardwareSerial
- SoftwareSerial

---

## Firmware Upload

### Step 1: Connect ESP32-S3

1. Connect ESP32-S3 to computer via USB-C cable
2. Wait for drivers to install (Windows may need a moment)

### Step 2: Select Port

1. Go to **Tools** → **Port**
2. Select the COM port (e.g., COM3 on Windows, /dev/ttyUSB0 on Linux)

### Step 3: Open Firmware File

1. Go to **File** → **Open**
2. Navigate to `RoverProject/rover_main/rover_main.ino`

### Step 4: Upload

1. Click **Verify** (checkmark icon) to compile
2. If no errors, click **Upload** (arrow icon)
3. Wait for "Done uploading" message

### Troubleshooting Upload Issues

**Problem: "Failed to connect to ESP32-S3"**
- Hold the **BOOT** button on the ESP32-S3 while clicking Upload
- Release BOOT when "Connecting..." appears
- Some boards require this manual boot mode trigger

**Problem: "No port found"**
- Install CP210x or CH340 drivers (depending on your board's USB chip)
- Windows: Check Device Manager → Ports (COM & LPT)
- Linux: Run `sudo usermod -a -G dialout $USER` and reboot

**Problem: "Brownout detector triggered"**
- Use a better USB cable
- Connect external 5V power supply
- Disable brownout detection in code (not recommended)

---

## Configuration Before Upload

### Required Changes in Code

Open `rover_main.ino` and modify these values:

```cpp
// Line ~194: Set your base station MAC address
uint8_t baseMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 
// Replace with actual ESP32-C3 MAC address

// Lines ~206-211: Set default configuration
strcpy(config.phone1, "+1234567890");  // Your phone number
strcpy(config.phone2, "");              // Secondary number (optional)
config.baseLat = 12.9716;               // Base station latitude
config.baseLng = 77.5946;               // Base station longitude
config.alertTempHigh = 55.0;            // Temperature alert threshold
```

### Finding ESP32-C3 MAC Address

To find the MAC address of your ESP32-C3 (base station bridge):

1. Flash the ESP32-C3 with base_bridge.ino
2. Open Serial Monitor at 115200 baud
3. The MAC address will be printed on startup
4. Update the `baseMac[]` array in rover_main.ino with this address

---

## Pin Connections Reference

| Component | Pin | ESP32-S3 GPIO | Notes |
|-----------|-----|---------------|-------|
| L9110S Motor Driver | B-IA | GPIO 4 | PWM Speed - Left |
| L9110S Motor Driver | B-IB | GPIO 5 | Direction - Left |
| L9110S Motor Driver | A-IA | GPIO 6 | PWM Speed - Right |
| L9110S Motor Driver | A-IB | GPIO 7 | Direction - Right |
| ESP32-CAM | TX | GPIO 17 (RX) | UART1 |
| ESP32-CAM | RX | GPIO 18 (TX) | UART1 |
| SIM800L | TXD | GPIO 16 (RX) | UART2 |
| SIM800L | RXD | GPIO 15 (TX) | UART2 |
| Neo-6M GPS | TX | GPIO 8 (RX) | SoftwareSerial |
| Neo-6M GPS | RX | GPIO 19 (TX) | SoftwareSerial |
| MPU6050 | SDA | GPIO 42 | I2C |
| MPU6050 | SCL | GPIO 2 | I2C |
| Ultrasonic | Trig | GPIO 12 | - |
| Ultrasonic | Echo | GPIO 13 | Needs voltage divider |
| DS18B20 | Data | GPIO 21 | Needs 4.7k pull-up |
| Buzzer | + | GPIO 48 | Active buzzer |
| Battery Monitor | + | GPIO 1 (ADC) | Needs voltage divider |

---

## Testing After Upload

### Serial Monitor Test

1. Open Serial Monitor (**Tools** → **Serial Monitor**)
2. Set baud rate to **115200**
3. You should see: `STASIS Rover initialized`

### LED Indicators

- **Built-in LED**: Blinks on startup
- **No LED activity after init**: Normal operation

### Common Test Commands

Send these commands via ESP-NOW from base station:
- `FORWARD` - Move forward
- `BACKWARD` - Move backward
- `LEFT` - Turn left
- `RIGHT` - Turn right
- `STOP` - Stop motors

---

## Power Requirements

| Component | Voltage | Current (Peak) |
|-----------|---------|----------------|
| ESP32-S3 | 3.3V / 5V | 500mA |
| ESP32-CAM | 5V | 500mA (peak 2A with flash) |
| SIM800L | 3.4V-4.4V | 2A (peak) |
| Motors (4x) | 6V-12V | 2A (stall) |
| GPS | 3.3V-5V | 50mA |
| MPU6050 | 3.3V | 10mA |
| DS18B20 | 3.3V-5V | 5mA |

**Total estimated peak current: 5-6A**

Use a buck converter to step down battery voltage to 5V for ESP32 and peripherals.

---

## Next Steps

After flashing the ESP32-S3:
1. Flash the ESP32-CAM with [cam_firmware.ino](./ESP32-CAM_INSTRUCTIONS.md)
2. Flash the ESP32-C3 with [base_bridge.ino](./ESP32-C3_INSTRUCTIONS.md)
3. Set up the Raspberry Pi Zero with [station_monitor.py](./RPI0_INSTRUCTIONS.md)
4. Review the complete [wiring diagram](./WIRING_DIAGRAMS.md)

---

## Troubleshooting

### GPS Not Getting Fix
- Ensure GPS has clear view of sky
- Wait 2-5 minutes for cold start
- Check antenna connection

### MPU6050 Not Detected
- Check I2C connections (SDA, SCL)
- Verify 3.3V power supply
- Run I2C scanner sketch

### Motors Not Moving
- Check L9110S power supply
- Verify motor connections
- Test with simple PWM sketch

### SIM800 Not Responding
- Ensure adequate power supply (2A peak)
- Check SIM card is inserted
- Verify UART connections

---

*Document Version: 1.0*  
*Last Updated: February 2026*

## Overview

The ESP32-S3 is the main controller for the rover unit. It handles:
- Navigation and motor control
- Sensor data collection (GPS, temperature, IMU, ultrasonic)
- Communication with ESP32-CAM (vision module)
- GSM/SMS alerts via SIM800
- ESP-NOW communication with base station
- State machine logic (Patrol, Research, Alert, Return Base, Docked)

---

## Hardware Requirements

### Components
- ESP32-S3 Development Board (ESP32-S3-DEVKITC or similar)
- USB-C cable for programming
- Computer with Arduino IDE installed

### Board Specifications
- Flash Memory: 8MB or 16MB (recommended)
- PSRAM: 8MB (recommended for image processing)
- USB: Native USB-C (no need for external USB-TTL adapter)

---

## Software Setup - Arduino IDE

### Step 1: Install Arduino IDE

Download and install Arduino IDE from:
- **Windows/Mac/Linux**: https://www.arduino.cc/en/software

### Step 2: Add ESP32 Board Support

1. Open Arduino IDE
2. Go to **File** → **Preferences**
3. In "Additional Boards Manager URLs", add:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
4. Click **OK**

5. Go to **Tools** → **Board** → **Boards Manager**
6. Search for "esp32"
7. Install **esp32 by Espressif Systems** (version 2.0.11 or later)

### Step 3: Select ESP32-S3 Board

1. Go to **Tools** → **Board** → **esp32**
2. Select **ESP32S3 Dev Module**

### Step 4: Configure Board Settings

Set the following options in **Tools** menu:

| Setting | Value |
|---------|-------|
| Board | ESP32S3 Dev Module |
| Upload Speed | 921600 |
| USB Mode | Hardware CDC and JTAG |
| USB CDC On Boot | Enabled |
| USB Firmware MSC On Boot | Disabled |
| Partition Scheme | Huge App (3MB No OTA/1MB SPIFFS) |
| CPU Frequency | 240MHz (WiFi/BT) |
| Flash Mode | QIO 80MHz |
| Flash Size | 8MB (or match your board) |
| Core Debug Level | None |
| PSRAM | OPI PSRAM (if available) |

---

## Required Libraries

### Install via Library Manager

Go to **Sketch** → **Include Library** → **Manage Libraries** and install:

| Library | Version | Purpose |
|---------|---------|---------|
| Adafruit MPU6050 | 2.x | IMU sensor |
| Adafruit Unified Sensor | 1.x | Sensor abstraction |
| TinyGPSPlus | 1.0.3 | GPS parsing |
| OneWire | 2.3.8 | DS18B20 temperature sensor |
| DallasTemperature | 3.9.0 | DS18B20 driver |
| ArduinoJson | 6.x or 7.x | JSON parsing |

### Libraries Included with ESP32 Core (No installation needed)
- WiFi
- ESP-NOW
- WebServer
- EEPROM
- HardwareSerial
- SoftwareSerial

---

## Firmware Upload

### Step 1: Connect ESP32-S3

1. Connect ESP32-S3 to computer via USB-C cable
2. Wait for drivers to install (Windows may need a moment)

### Step 2: Select Port

1. Go to **Tools** → **Port**
2. Select the COM port (e.g., COM3 on Windows, /dev/ttyUSB0 on Linux)

### Step 3: Open Firmware File

1. Go to **File** → **Open**
2. Navigate to `RoverProject/rover_main/rover_main.ino`

### Step 4: Upload

1. Click **Verify** (checkmark icon) to compile
2. If no errors, click **Upload** (arrow icon)
3. Wait for "Done uploading" message

### Troubleshooting Upload Issues

**Problem: "Failed to connect to ESP32-S3"**
- Hold the **BOOT** button on the ESP32-S3 while clicking Upload
- Release BOOT when "Connecting..." appears
- Some boards require this manual boot mode trigger

**Problem: "No port found"**
- Install CP210x or CH340 drivers (depending on your board's USB chip)
- Windows: Check Device Manager → Ports (COM & LPT)
- Linux: Run `sudo usermod -a -G dialout $USER` and reboot

**Problem: "Brownout detector triggered"**
- Use a better USB cable
- Connect external 5V power supply
- Disable brownout detection in code (not recommended)

---

## Configuration Before Upload

### Required Changes in Code

Open `rover_main.ino` and modify these values:

```cpp
// Line ~194: Set your base station MAC address
uint8_t baseMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 
// Replace with actual ESP32-C3 MAC address

// Lines ~206-211: Set default configuration
strcpy(config.phone1, "+1234567890");  // Your phone number
strcpy(config.phone2, "");              // Secondary number (optional)
config.baseLat = 12.9716;               // Base station latitude
config.baseLng = 77.5946;               // Base station longitude
config.alertTempHigh = 55.0;            // Temperature alert threshold
```

### Finding ESP32-C3 MAC Address

To find the MAC address of your ESP32-C3 (base station bridge):

1. Flash the ESP32-C3 with base_bridge.ino
2. Open Serial Monitor at 115200 baud
3. The MAC address will be printed on startup
4. Update the `baseMac[]` array in rover_main.ino with this address

---

## Pin Connections Reference

| Component | Pin | ESP32-S3 GPIO | Notes |
|-----------|-----|---------------|-------|
| L9110S Motor Driver | B-IA | GPIO 4 | PWM Speed - Left |
| L9110S Motor Driver | B-IB | GPIO 5 | Direction - Left |
| L9110S Motor Driver | A-IA | GPIO 6 | PWM Speed - Right |
| L9110S Motor Driver | A-IB | GPIO 7 | Direction - Right |
| ESP32-CAM | TX | GPIO 17 (RX) | UART1 |
| ESP32-CAM | RX | GPIO 18 (TX) | UART1 |
| SIM800L | TXD | GPIO 16 (RX) | UART2 |
| SIM800L | RXD | GPIO 15 (TX) | UART2 |
| Neo-6M GPS | TX | GPIO 8 (RX) | SoftwareSerial |
| Neo-6M GPS | RX | GPIO 19 (TX) | SoftwareSerial |
| MPU6050 | SDA | GPIO 42 | I2C |
| MPU6050 | SCL | GPIO 2 | I2C |
| Ultrasonic | Trig | GPIO 12 | - |
| Ultrasonic | Echo | GPIO 13 | Needs voltage divider |
| DS18B20 | Data | GPIO 21 | Needs 4.7k pull-up |
| Buzzer | + | GPIO 48 | Active buzzer |
| Battery Monitor | + | GPIO 1 (ADC) | Needs voltage divider |

---

## Testing After Upload

### Serial Monitor Test

1. Open Serial Monitor (**Tools** → **Serial Monitor**)
2. Set baud rate to **115200**
3. You should see: `STASIS Rover initialized`

### LED Indicators

- **Built-in LED**: Blinks on startup
- **No LED activity after init**: Normal operation

### Common Test Commands

Send these commands via ESP-NOW from base station:
- `FORWARD` - Move forward
- `BACKWARD` - Move backward
- `LEFT` - Turn left
- `RIGHT` - Turn right
- `STOP` - Stop motors

---

## Power Requirements

| Component | Voltage | Current (Peak) |
|-----------|---------|----------------|
| ESP32-S3 | 3.3V / 5V | 500mA |
| ESP32-CAM | 5V | 500mA (peak 2A with flash) |
| SIM800L | 3.4V-4.4V | 2A (peak) |
| Motors (4x) | 6V-12V | 2A (stall) |
| GPS | 3.3V-5V | 50mA |
| MPU6050 | 3.3V | 10mA |
| DS18B20 | 3.3V-5V | 5mA |

**Total estimated peak current: 5-6A**

Use a buck converter to step down battery voltage to 5V for ESP32 and peripherals.

---

## Next Steps

After flashing the ESP32-S3:
1. Flash the ESP32-CAM with [cam_firmware.ino](./ESP32-CAM_INSTRUCTIONS.md)
2. Flash the ESP32-C3 with [base_bridge.ino](./ESP32-C3_INSTRUCTIONS.md)
3. Set up the Raspberry Pi Zero with [station_monitor.py](./RPI0_INSTRUCTIONS.md)
4. Review the complete [wiring diagram](./WIRING_DIAGRAMS.md)

---

## Troubleshooting

### GPS Not Getting Fix
- Ensure GPS has clear view of sky
- Wait 2-5 minutes for cold start
- Check antenna connection

### MPU6050 Not Detected
- Check I2C connections (SDA, SCL)
- Verify 3.3V power supply
- Run I2C scanner sketch

### Motors Not Moving
- Check L9110S power supply
- Verify motor connections
- Test with simple PWM sketch

### SIM800 Not Responding
- Ensure adequate power supply (2A peak)
- Check SIM card is inserted
- Verify UART connections

---

*Document Version: 1.0*  
*Last Updated: February 2026*

## Overview

The ESP32-S3 is the main controller for the rover unit. It handles:
- Navigation and motor control
- Sensor data collection (GPS, temperature, IMU, ultrasonic)
- Communication with ESP32-CAM (vision module)
- GSM/SMS alerts via SIM800
- ESP-NOW communication with base station
- State machine logic (Patrol, Research, Alert, Return Base, Docked)

---

## Hardware Requirements

### Components
- ESP32-S3 Development Board (ESP32-S3-DEVKITC or similar)
- USB-C cable for programming
- Computer with Arduino IDE installed

### Board Specifications
- Flash Memory: 8MB or 16MB (recommended)
- PSRAM: 8MB (recommended for image processing)
- USB: Native USB-C (no need for external USB-TTL adapter)

---

## Software Setup - Arduino IDE

### Step 1: Install Arduino IDE

Download and install Arduino IDE from:
- **Windows/Mac/Linux**: https://www.arduino.cc/en/software

### Step 2: Add ESP32 Board Support

1. Open Arduino IDE
2. Go to **File** → **Preferences**
3. In "Additional Boards Manager URLs", add:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
4. Click **OK**

5. Go to **Tools** → **Board** → **Boards Manager**
6. Search for "esp32"
7. Install **esp32 by Espressif Systems** (version 2.0.11 or later)

### Step 3: Select ESP32-S3 Board

1. Go to **Tools** → **Board** → **esp32**
2. Select **ESP32S3 Dev Module**

### Step 4: Configure Board Settings

Set the following options in **Tools** menu:

| Setting | Value |
|---------|-------|
| Board | ESP32S3 Dev Module |
| Upload Speed | 921600 |
| USB Mode | Hardware CDC and JTAG |
| USB CDC On Boot | Enabled |
| USB Firmware MSC On Boot | Disabled |
| Partition Scheme | Huge App (3MB No OTA/1MB SPIFFS) |
| CPU Frequency | 240MHz (WiFi/BT) |
| Flash Mode | QIO 80MHz |
| Flash Size | 8MB (or match your board) |
| Core Debug Level | None |
| PSRAM | OPI PSRAM (if available) |

---

## Required Libraries

### Install via Library Manager

Go to **Sketch** → **Include Library** → **Manage Libraries** and install:

| Library | Version | Purpose |
|---------|---------|---------|
| Adafruit MPU6050 | 2.x | IMU sensor |
| Adafruit Unified Sensor | 1.x | Sensor abstraction |
| TinyGPSPlus | 1.0.3 | GPS parsing |
| OneWire | 2.3.8 | DS18B20 temperature sensor |
| DallasTemperature | 3.9.0 | DS18B20 driver |
| ArduinoJson | 6.x or 7.x | JSON parsing |

### Libraries Included with ESP32 Core (No installation needed)
- WiFi
- ESP-NOW
- WebServer
- EEPROM
- HardwareSerial
- SoftwareSerial

---

## Firmware Upload

### Step 1: Connect ESP32-S3

1. Connect ESP32-S3 to computer via USB-C cable
2. Wait for drivers to install (Windows may need a moment)

### Step 2: Select Port

1. Go to **Tools** → **Port**
2. Select the COM port (e.g., COM3 on Windows, /dev/ttyUSB0 on Linux)

### Step 3: Open Firmware File

1. Go to **File** → **Open**
2. Navigate to `RoverProject/rover_main/rover_main.ino`

### Step 4: Upload

1. Click **Verify** (checkmark icon) to compile
2. If no errors, click **Upload** (arrow icon)
3. Wait for "Done uploading" message

### Troubleshooting Upload Issues

**Problem: "Failed to connect to ESP32-S3"**
- Hold the **BOOT** button on the ESP32-S3 while clicking Upload
- Release BOOT when "Connecting..." appears
- Some boards require this manual boot mode trigger

**Problem: "No port found"**
- Install CP210x or CH340 drivers (depending on your board's USB chip)
- Windows: Check Device Manager → Ports (COM & LPT)
- Linux: Run `sudo usermod -a -G dialout $USER` and reboot

**Problem: "Brownout detector triggered"**
- Use a better USB cable
- Connect external 5V power supply
- Disable brownout detection in code (not recommended)

---

## Configuration Before Upload

### Required Changes in Code

Open `rover_main.ino` and modify these values:

```cpp
// Line ~194: Set your base station MAC address
uint8_t baseMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 
// Replace with actual ESP32-C3 MAC address

// Lines ~206-211: Set default configuration
strcpy(config.phone1, "+1234567890");  // Your phone number
strcpy(config.phone2, "");              // Secondary number (optional)
config.baseLat = 12.9716;               // Base station latitude
config.baseLng = 77.5946;               // Base station longitude
config.alertTempHigh = 55.0;            // Temperature alert threshold
```

### Finding ESP32-C3 MAC Address

To find the MAC address of your ESP32-C3 (base station bridge):

1. Flash the ESP32-C3 with base_bridge.ino
2. Open Serial Monitor at 115200 baud
3. The MAC address will be printed on startup
4. Update the `baseMac[]` array in rover_main.ino with this address

---

## Pin Connections Reference

| Component | Pin | ESP32-S3 GPIO | Notes |
|-----------|-----|---------------|-------|
| L9110S Motor Driver | B-IA | GPIO 4 | PWM Speed - Left |
| L9110S Motor Driver | B-IB | GPIO 5 | Direction - Left |
| L9110S Motor Driver | A-IA | GPIO 6 | PWM Speed - Right |
| L9110S Motor Driver | A-IB | GPIO 7 | Direction - Right |
| ESP32-CAM | TX | GPIO 17 (RX) | UART1 |
| ESP32-CAM | RX | GPIO 18 (TX) | UART1 |
| SIM800L | TXD | GPIO 16 (RX) | UART2 |
| SIM800L | RXD | GPIO 15 (TX) | UART2 |
| Neo-6M GPS | TX | GPIO 8 (RX) | SoftwareSerial |
| Neo-6M GPS | RX | GPIO 19 (TX) | SoftwareSerial |
| MPU6050 | SDA | GPIO 42 | I2C |
| MPU6050 | SCL | GPIO 2 | I2C |
| Ultrasonic | Trig | GPIO 12 | - |
| Ultrasonic | Echo | GPIO 13 | Needs voltage divider |
| DS18B20 | Data | GPIO 21 | Needs 4.7k pull-up |
| Buzzer | + | GPIO 48 | Active buzzer |
| Battery Monitor | + | GPIO 1 (ADC) | Needs voltage divider |

---

## Testing After Upload

### Serial Monitor Test

1. Open Serial Monitor (**Tools** → **Serial Monitor**)
2. Set baud rate to **115200**
3. You should see: `STASIS Rover initialized`

### LED Indicators

- **Built-in LED**: Blinks on startup
- **No LED activity after init**: Normal operation

### Common Test Commands

Send these commands via ESP-NOW from base station:
- `FORWARD` - Move forward
- `BACKWARD` - Move backward
- `LEFT` - Turn left
- `RIGHT` - Turn right
- `STOP` - Stop motors

---

## Power Requirements

| Component | Voltage | Current (Peak) |
|-----------|---------|----------------|
| ESP32-S3 | 3.3V / 5V | 500mA |
| ESP32-CAM | 5V | 500mA (peak 2A with flash) |
| SIM800L | 3.4V-4.4V | 2A (peak) |
| Motors (4x) | 6V-12V | 2A (stall) |
| GPS | 3.3V-5V | 50mA |
| MPU6050 | 3.3V | 10mA |
| DS18B20 | 3.3V-5V | 5mA |

**Total estimated peak current: 5-6A**

Use a buck converter to step down battery voltage to 5V for ESP32 and peripherals.

---

## Next Steps

After flashing the ESP32-S3:
1. Flash the ESP32-CAM with [`cam_firmware.ino`](./ESP32-CAM_INSTRUCTIONS.md)
2. Flash the ESP32-C3 with [`base_bridge.ino`](./ESP32-C3_INSTRUCTIONS.md)
3. Set up the Raspberry Pi Zero with [`station_monitor.py`](./RPI0_INSTRUCTIONS.md)
4. Review the complete [wiring diagram](./WIRING_DIAGRAMS.md)

---

## Troubleshooting

### GPS Not Getting Fix
- Ensure GPS has clear view of sky
- Wait 2-5 minutes for cold start
- Check antenna connection

### MPU6050 Not Detected
- Check I2C connections (SDA, SCL)
- Verify 3.3V power supply
- Run I2C scanner sketch

### Motors Not Moving
- Check L9110S power supply
- Verify motor connections
- Test with simple PWM sketch

### SIM800 Not Responding
- Ensure adequate power supply (2A peak)
- Check SIM card is inserted
- Verify UART connections

---

*Document Version: 1.0*
*Last Updated: February 2026*


## Overview

The ESP32-S3 is the main controller for the rover unit. It handles:
- Navigation and motor control
- Sensor data collection (GPS, temperature, IMU, ultrasonic)
- Communication with ESP32-CAM (vision module)
- GSM/SMS alerts via SIM800
- ESP-NOW communication with base station
- State machine logic (Patrol, Research, Alert, Return Base, Docked)

---

## Hardware Requirements

### Components
- ESP32-S3 Development Board (ESP32-S3-DEVKITC or similar)
- USB-C cable for programming
- Computer with Arduino IDE installed

### Board Specifications
- Flash Memory: 8MB or 16MB (recommended)
- PSRAM: 8MB (recommended for image processing)
- USB: Native USB-C (no need for external USB-TTL adapter)

---

## Software Setup - Arduino IDE

### Step 1: Install Arduino IDE

Download and install Arduino IDE from:
- **Windows/Mac/Linux**: https://www.arduino.cc/en/software

### Step 2: Add ESP32 Board Support

1. Open Arduino IDE
2. Go to **File** → **Preferences**
3. In "Additional Boards Manager URLs", add:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
4. Click **OK**

5. Go to **Tools** → **Board** → **Boards Manager**
6. Search for "esp32"
7. Install **esp32 by Espressif Systems** (version 2.0.11 or later)

### Step 3: Select ESP32-S3 Board

1. Go to **Tools** → **Board** → **esp32**
2. Select **ESP32S3 Dev Module**

### Step 4: Configure Board Settings

Set the following options in **Tools** menu:

| Setting | Value |
|---------|-------|
| Board | ESP32S3 Dev Module |
| Upload Speed | 921600 |
| USB Mode | Hardware CDC and JTAG |
| USB CDC On Boot | Enabled |
| USB Firmware MSC On Boot | Disabled |
| Partition Scheme | Huge App (3MB No OTA/1MB SPIFFS) |
| CPU Frequency | 240MHz (WiFi/BT) |
| Flash Mode | QIO 80MHz |
| Flash Size | 8MB (or match your board) |
| Partition Scheme | Huge App (3MB No OTA) |
| Core Debug Level | None |
| PSRAM | OPI PSRAM (if available) |

---

## Required Libraries

### Install via Library Manager

Go to **Sketch** → **Include Library** → **Manage Libraries** and install:

| Library | Version | Purpose |
|---------|---------|---------|
| Adafruit MPU6050 | 2.x | IMU sensor |
| Adafruit Unified Sensor | 1.x | Sensor abstraction |
| TinyGPSPlus | 1.0.3 | GPS parsing |
| OneWire | 2.3.8 | DS18B20 temperature sensor |
| DallasTemperature | 3.9.0 | DS18B20 driver |
| ArduinoJson | 6.x or 7.x | JSON parsing |

### Libraries Included with ESP32 Core (No installation needed)
- WiFi
- ESP-NOW
- WebServer
- EEPROM
- HardwareSerial
- SoftwareSerial

---

## Firmware Upload

### Step 1: Connect ESP32-S3

1. Connect ESP32-S3 to computer via USB-C cable
2. Wait for drivers to install (Windows may need a moment)

### Step 2: Select Port

1. Go to **Tools** → **Port**
2. Select the COM port (e.g., COM3 on Windows, /dev/ttyUSB0 on Linux)

### Step 3: Open Firmware File

1. Go to **File** → **Open**
2. Navigate to `RoverProject/rover_main/rover_main.ino`

### Step 4: Upload

1. Click **Verify** (checkmark icon) to compile
2. If no errors, click **Upload** (arrow icon)
3. Wait for "Done uploading" message

### Troubleshooting Upload Issues

**Problem: "Failed to connect to ESP32-S3"**
- Hold the **BOOT** button on the ESP32-S3 while clicking Upload
- Release BOOT when "Connecting..." appears
- Some boards require this manual boot mode trigger

**Problem: "No port found"**
- Install CP210x or CH340 drivers (depending on your board's USB chip)
- Windows: Check Device Manager → Ports (COM & LPT)
- Linux: Run `sudo usermod -a -G dialout $USER` and reboot

**Problem: "Brownout detector triggered"**
- Use a better USB cable
- Connect external 5V power supply
- Disable brownout detection in code (not recommended)

---

## Configuration Before Upload

### Required Changes in Code

Open `rover_main.ino` and modify these values:

```cpp
// Line ~194: Set your base station MAC address
uint8_t baseMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 
// Replace with actual ESP32-C3 MAC address

// Lines ~206-211: Set default configuration
strcpy(config.phone1, "+1234567890");  // Your phone number
strcpy(config.phone2, "");              // Secondary number (optional)
config.baseLat = 12.9716;               // Base station latitude
config.baseLng = 77.5946;               // Base station longitude
config.alertTempHigh = 55.0;            // Temperature alert threshold
```

### Finding ESP32-C3 MAC Address

To find the MAC address of your ESP32-C3 (base station bridge):

1. Flash the ESP32-C3 with base_bridge.ino
2. Open Serial Monitor at 115200 baud
3. The MAC address will be printed on startup
4. Update the `baseMac[]` array in rover_main.ino with this address

---

## Pin Connections Reference

| Component | Pin | ESP32-S3 GPIO | Notes |
|-----------|-----|---------------|-------|
| L9110S Motor Driver | B-IA | GPIO 4 | PWM Speed - Left |
| L9110S Motor Driver | B-IB | GPIO 5 | Direction - Left |
| L9110S Motor Driver | A-IA | GPIO 6 | PWM Speed - Right |
| L9110S Motor Driver | A-IB | GPIO 7 | Direction - Right |
| ESP32-CAM | TX | GPIO 17 (RX) | UART1 |
| ESP32-CAM | RX | GPIO 18 (TX) | UART1 |
| SIM800L | TXD | GPIO 16 (RX) | UART2 |
| SIM800L | RXD | GPIO 15 (TX) | UART2 |
| Neo-6M GPS | TX | GPIO 8 (RX) | SoftwareSerial |
| Neo-6M GPS | RX | GPIO 19 (TX) | SoftwareSerial |
| MPU6050 | SDA | GPIO 42 | I2C |
| MPU6050 | SCL | GPIO 2 | I2C |
| Ultrasonic | Trig | GPIO 12 | - |
| Ultrasonic | Echo | GPIO 13 | Needs voltage divider |
| DS18B20 | Data | GPIO 21 | Needs 4.7k pull-up |
| Buzzer | + | GPIO 48 | Active buzzer |
| Battery Monitor | + | GPIO 1 (ADC) | Needs voltage divider |

---

## Testing After Upload

### Serial Monitor Test

1. Open Serial Monitor (**Tools** → **Serial Monitor**)
2. Set baud rate to **115200**
3. You should see: `STASIS Rover initialized`

### LED Indicators

- **Built-in LED**: Blinks on startup
- **No LED activity after init**: Normal operation

### Common Test Commands

Send these commands via ESP-NOW from base station:
- `FORWARD` - Move forward
- `BACKWARD` - Move backward
- `LEFT` - Turn left
- `RIGHT` - Turn right
- `STOP` - Stop motors

---

## Power Requirements

| Component | Voltage | Current (Peak) |
|-----------|---------|----------------|
| ESP32-S3 | 3.3V / 5V | 500mA |
| ESP32-CAM | 5V | 500mA (peak 2A with flash) |
| SIM800L | 3.4V-4.4V | 2A (peak) |
| Motors (4x) | 6V-12V | 2A (stall) |
| GPS | 3.3V-5V | 50mA |
| MPU6050 | 3.3V | 10mA |
| DS18B20 | 3.3V-5V | 5mA |

**Total estimated peak current: 5-6A**

Use a buck converter to step down battery voltage to 5V for ESP32 and peripherals.

---

## Next Steps

After flashing the ESP32-S3:
1. Flash the ESP32-CAM with [`cam_firmware.ino`](./ESP32-CAM_INSTRUCTIONS.md)
2. Flash the ESP32-C3 with [`base_bridge.ino`](./ESP32-C3_INSTRUCTIONS.md)
3. Set up the Raspberry Pi Zero with [`station_monitor.py`](./RPI0_INSTRUCTIONS.md)
4. Review the complete [wiring diagram](./WIRING_DIAGRAMS.md)

---

## Troubleshooting

### GPS Not Getting Fix
- Ensure GPS has clear view of sky
- Wait 2-5 minutes for cold start
- Check antenna connection

### MPU6050 Not Detected
- Check I2C connections (SDA, SCL)
- Verify 3.3V power supply
- Run I2C scanner sketch

### Motors Not Moving
- Check L9110S power supply
- Verify motor connections
- Test with simple PWM sketch

### SIM800 Not Responding
- Ensure adequate power supply (2A peak)
- Check SIM card is inserted
- Verify UART connections

---

*Document Version: 1.0*
*Last Updated: February 2026*

