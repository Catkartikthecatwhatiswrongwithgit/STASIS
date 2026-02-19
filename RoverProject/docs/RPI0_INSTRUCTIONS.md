# Raspberry Pi Zero - Auto-Boot Setup Instructions

## Overview

This guide sets up the Raspberry Pi Zero to:
- Automatically run the station monitor Python script on power-up
- Connect to the ESP32-C3 via UART for rover communication
- Create a WiFi hotspot through the ESP32-C3 for status indication
- Operate in headless mode (no monitor/keyboard required)

---

## Hardware Requirements

### Components
- Raspberry Pi Zero (any version: Pi Zero, Pi Zero W, or Pi Zero 2 W)
- MicroSD Card (8GB minimum, 16GB recommended)
- 5V Power Supply (2.5A recommended)
- ESP32-C3 (connected via UART)

### Optional
- USB to TTL adapter (for initial headless setup)
- HDMI adapter and monitor (for debugging)

---

## Operating System Setup

### Step 1: Download Raspberry Pi OS

Download **Raspberry Pi OS Lite** (no desktop environment needed):
- URL: https://www.raspberrypi.com/software/operating-systems/
- Choose: **Raspberry Pi OS Lite (64-bit or 32-bit)**

### Step 2: Flash the SD Card

Use **Raspberry Pi Imager** or **balenaEtcher**:

#### Using Raspberry Pi Imager (Recommended)

1. Download from: https://www.raspberrypi.com/software/
2. Insert SD card into computer
3. Open Raspberry Pi Imager
4. Select OS: **Raspberry Pi OS (other)** → **Raspberry Pi OS Lite**
5. Select your SD card
6. Click the **gear icon** (Advanced options):
   - Set hostname: `aerosentinel-base`
   - Enable SSH: **Yes**
   - Set username/password: `pi` / `sentinel2024`
   - Configure wireless LAN: (optional, for internet access)
   - Set locale settings: Your timezone
7. Click **Save** then **Write**

#### Using balenaEtcher

1. Download from: https://www.balena.io/etcher/
2. Flash the downloaded `.img` file to SD card
3. After flashing, open the `boot` partition
4. Create an empty file named `ssh` (no extension) to enable SSH
5. Create `wpa_supplicant.conf` for WiFi (optional):

```conf
country=US
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1

network={
    ssid="YourWiFiName"
    psk="YourWiFiPassword"
}
```

---

## Initial Boot and Configuration

### Step 1: Boot the Pi

1. Insert SD card into Raspberry Pi Zero
2. Connect ESP32-C3 to GPIO pins (see wiring below)
3. Connect power (micro-USB)
4. Wait 2-3 minutes for first boot

### Step 2: Connect via SSH

```bash
ssh pi@aerosentinel-base.local
# Password: sentinel2024
```

Or use IP address if hostname doesn't resolve:
```bash
ssh pi@192.168.1.xxx
```

### Step 3: Update System

```bash
sudo apt update
sudo apt upgrade -y
```

### Step 4: Enable Serial Port

```bash
sudo raspi-config
```

Navigate to:
1. **Interface Options** → **Serial Port**
2. Login shell on serial: **No**
3. Serial port hardware enabled: **Yes**
4. Reboot when prompted

---

## Install Required Packages

### Step 1: Install Python Dependencies

```bash
sudo apt install -y python3-pip python3-venv
sudo pip3 install flask flask-cors pyserial fpdf2
```

### Step 2: Create Project Directory

```bash
mkdir -p ~/aerosentinel
cd ~/aerosentinel
```

### Step 3: Copy Station Monitor Script

Copy the `station_monitor.py` file to the Pi:

**From your computer:**
```bash
scp RoverProject/base_station/station_monitor.py pi@aerosentinel-base.local:~/aerosentinel/
```

**Or create directly on Pi:**
```bash
nano ~/aerosentinel/station_monitor.py
# Paste the contents
```

---

## Auto-Start Configuration

### Method 1: systemd Service (Recommended)

Create a systemd service to auto-start the script:

#### Step 1: Create Service File

```bash
sudo nano /etc/systemd/system/aerosentinel.service
```

#### Step 2: Add Service Configuration

```ini
[Unit]
Description=Aero Sentinel Base Station Monitor
After=network.target serial-getty@serial0.service
Wants=network-online.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/aerosentinel
ExecStart=/usr/bin/python3 /home/pi/aerosentinel/station_monitor.py
Restart=always
RestartSec=10
StandardOutput=journal
StandardError=journal

# Environment variables
Environment="SERIAL_PORT=/dev/serial0"
Environment="BAUD_RATE=115200"
Environment="FLASK_HOST=0.0.0.0"
Environment="FLASK_PORT=5000"

[Install]
WantedBy=multi-user.target
```

#### Step 3: Enable and Start Service

```bash
sudo systemctl daemon-reload
sudo systemctl enable aerosentinel.service
sudo systemctl start aerosentinel.service
```

#### Step 4: Check Status

```bash
sudo systemctl status aerosentinel.service
```

#### View Logs

```bash
journalctl -u aerosentinel.service -f
```

### Method 2: crontab (Alternative)

```bash
crontab -e
```

Add this line:
```
@reboot sleep 30 && /usr/bin/python3 /home/pi/aerosentinel/station_monitor.py &
```

### Method 3: rc.local (Legacy)

```bash
sudo nano /etc/rc.local
```

Add before `exit 0`:
```bash
# Wait for network and serial to be ready
sleep 20
su - pi -c "/usr/bin/python3 /home/pi/aerosentinel/station_monitor.py &"
```

---

## Wiring: ESP32-C3 to Raspberry Pi Zero

### Connection Table

| ESP32-C3 | Raspberry Pi Zero GPIO | Function |
|----------|------------------------|----------|
| GPIO 21 (TX) | Pin 10 (GPIO 15 RXD) | UART TX → RX |
| GPIO 20 (RX) | Pin 8 (GPIO 14 TXD) | UART RX ← TX |
| 5V | Pin 2 or Pin 4 (5V) | Power |
| GND | Pin 6 (GND) | Common Ground |

### GPIO Pinout Reference

```
Raspberry Pi Zero GPIO Header (Top View)
─────────────────────────────────────────
 3.3V  ─── [01] [02] ─── 5V
 SDA   ─── [03] [04] ─── 5V
 SCL   ─── [05] [06] ─── GND
 GPIO4 ─── [07] [08] ─── TXD (GPIO 14) ──► ESP32-C3 RX
 GND   ─── [09] [10] ─── RXD (GPIO 15) ◄── ESP32-C3 TX
 GPIO17── [11] [12] ─── GPIO18
 GPIO27── [13] [14] ─── GND
 GPIO22── [15] [16] ─── GPIO23
 3.3V  ─── [17] [18] ─── GPIO24
 MOSI  ─── [19] [20] ─── GND
 MISO  ─── [21] [22] ─── GPIO25
 SCLK  ─── [23] [24] ─── CE0
 GND   ─── [25] [26] ─── CE1
─────────────────────────────────────────
```

---

## WiFi Status Indication via ESP32-C3

The ESP32-C3 creates a WiFi Access Point for status indication:

### Connection Details

| Setting | Value |
|---------|-------|
| SSID | `AeroSentinel-Base` |
| Password | `sentinel123` |
| IP | `192.168.4.1` |

### Accessing the Status Page

1. Connect your phone/laptop to `AeroSentinel-Base` WiFi
2. Open browser: `http://192.168.4.1`
3. View real-time status of:
   - ESP-NOW connection
   - Packets received
   - Commands sent
   - Last telemetry data

---

## Testing the Setup

### Test 1: Check Serial Communication

```bash
# Install minicom for testing
sudo apt install minicom

# Connect to serial port
minicom -D /dev/serial0 -b 115200

# You should see heartbeat messages from ESP32-C3 every 5 seconds
# Press Ctrl+A then X to exit
```

### Test 2: Check Flask API

```bash
# On another device on the same network
curl http://aerosentinel-base.local:5000/api/health

# Should return:
# {"status": "healthy", "uptime": 123, "packets_received": 0}
```

### Test 3: Check Service Status

```bash
sudo systemctl status aerosentinel.service
```

Should show: `Active: active (running)`

---

## Troubleshooting

### Serial Port Not Working

```bash
# Check if serial port exists
ls -la /dev/serial0

# Should show: /dev/serial0 -> ttyAMA0

# Check for conflicts
sudo cat /proc/cmdline
# Should NOT contain console=serial0,115200
```

### Service Not Starting

```bash
# Check logs
journalctl -u aerosentinel.service -n 50

# Check Python syntax
python3 -m py_compile /home/pi/aerosentinel/station_monitor.py

# Run manually to debug
cd /home/pi/aerosentinel
python3 station_monitor.py
```

### Permission Denied on Serial Port

```bash
# Add pi user to dialout group
sudo usermod -a -G dialout pi

# Logout and login again
```

### WiFi AP Not Visible

1. Check ESP32-C3 is powered on
2. Wait 30 seconds after Pi boots
3. Verify ESP32-C3 firmware has AP mode enabled

---

## Creating a Custom OS Image (Optional)

To create a pre-configured image for easy deployment:

### Step 1: Configure the System

Complete all setup steps above on a test SD card.

### Step 2: Shrink the Filesystem

```bash
sudo apt clean
sudo apt autoremove
sudo zerofree /dev/mmcblk0p2
# (Run from another system with the SD card mounted)
```

### Step 3: Create Image

On a Linux computer:
```bash
# Insert SD card and find device
lsblk

# Create image (replace sdX with your device)
sudo dd if=/dev/sdX of=aerosentinel-base.img bs=4M status=progress
```

### Step 4: Compress for Distribution

```bash
gzip aerosentinel-base.img
```

---

## Power-On Behavior Summary

When power is connected to the Raspberry Pi Zero:

1. **Boot** (30-60 seconds)
   - Raspberry Pi OS loads
   - Network interfaces initialize
   - Serial port becomes available

2. **Service Start** (after boot)
   - `aerosentinel.service` starts automatically
   - Python script begins execution
   - Serial connection to ESP32-C3 established

3. **Normal Operation**
   - Listens for telemetry from ESP32-C3
   - Serves REST API on port 5000
   - Stores data in SQLite database
   - Generates reports when rover docks

4. **Status Indication**
   - ESP32-C3 creates WiFi AP: `AeroSentinel-Base`
   - Status page available at `http://192.168.4.1`

---

## Next Steps

After setting up the Raspberry Pi Zero:
1. Verify ESP32-C3 connection with `minicom`
2. Test the API endpoints
3. Flash the [ESP32-C3 bridge](./ESP32-C3_INSTRUCTIONS.md)
4. Flash the [ESP32-S3 rover](./ESP32-S3_INSTRUCTIONS.md)
5. Test end-to-end communication

---

*Document Version: 1.0*  
*Last Updated: February 2026*

## Overview

This guide sets up the Raspberry Pi Zero to:
- Automatically run the station monitor Python script on power-up
- Connect to the ESP32-C3 via UART for rover communication
- Create a WiFi hotspot through the ESP32-C3 for status indication
- Operate in headless mode (no monitor/keyboard required)

---

## Hardware Requirements

### Components
- Raspberry Pi Zero (any version: Pi Zero, Pi Zero W, or Pi Zero 2 W)
- MicroSD Card (8GB minimum, 16GB recommended)
- 5V Power Supply (2.5A recommended)
- ESP32-C3 (connected via UART)

### Optional
- USB to TTL adapter (for initial headless setup)
- HDMI adapter and monitor (for debugging)

---

## Operating System Setup

### Step 1: Download Raspberry Pi OS

Download **Raspberry Pi OS Lite** (no desktop environment needed):
- URL: https://www.raspberrypi.com/software/operating-systems/
- Choose: **Raspberry Pi OS Lite (64-bit or 32-bit)**

### Step 2: Flash the SD Card

Use **Raspberry Pi Imager** or **balenaEtcher**:

#### Using Raspberry Pi Imager (Recommended)

1. Download from: https://www.raspberrypi.com/software/
2. Insert SD card into computer
3. Open Raspberry Pi Imager
4. Select OS: **Raspberry Pi OS (other)** → **Raspberry Pi OS Lite**
5. Select your SD card
6. Click the **gear icon** (Advanced options):
   - Set hostname: `aerosentinel-base`
   - Enable SSH: **Yes**
   - Set username/password: `pi` / `sentinel2024`
   - Configure wireless LAN: (optional, for internet access)
   - Set locale settings: Your timezone
7. Click **Save** then **Write**

#### Using balenaEtcher

1. Download from: https://www.balena.io/etcher/
2. Flash the downloaded `.img` file to SD card
3. After flashing, open the `boot` partition
4. Create an empty file named `ssh` (no extension) to enable SSH
5. Create `wpa_supplicant.conf` for WiFi (optional):

```conf
country=US
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1

network={
    ssid="YourWiFiName"
    psk="YourWiFiPassword"
}
```

---

## Initial Boot and Configuration

### Step 1: Boot the Pi

1. Insert SD card into Raspberry Pi Zero
2. Connect ESP32-C3 to GPIO pins (see wiring below)
3. Connect power (micro-USB)
4. Wait 2-3 minutes for first boot

### Step 2: Connect via SSH

```bash
ssh pi@aerosentinel-base.local
# Password: sentinel2024
```

Or use IP address if hostname doesn't resolve:
```bash
ssh pi@192.168.1.xxx
```

### Step 3: Update System

```bash
sudo apt update
sudo apt upgrade -y
```

### Step 4: Enable Serial Port

```bash
sudo raspi-config
```

Navigate to:
1. **Interface Options** → **Serial Port**
2. Login shell on serial: **No**
3. Serial port hardware enabled: **Yes**
4. Reboot when prompted

---

## Install Required Packages

### Step 1: Install Python Dependencies

```bash
sudo apt install -y python3-pip python3-venv
sudo pip3 install flask flask-cors pyserial fpdf2
```

### Step 2: Create Project Directory

```bash
mkdir -p ~/aerosentinel
cd ~/aerosentinel
```

### Step 3: Copy Station Monitor Script

Copy the `station_monitor.py` file to the Pi:

**From your computer:**
```bash
scp RoverProject/base_station/station_monitor.py pi@aerosentinel-base.local:~/aerosentinel/
```

**Or create directly on Pi:**
```bash
nano ~/aerosentinel/station_monitor.py
# Paste the contents
```

---

## Auto-Start Configuration

### Method 1: systemd Service (Recommended)

Create a systemd service to auto-start the script:

#### Step 1: Create Service File

```bash
sudo nano /etc/systemd/system/aerosentinel.service
```

#### Step 2: Add Service Configuration

```ini
[Unit]
Description=Aero Sentinel Base Station Monitor
After=network.target serial-getty@serial0.service
Wants=network-online.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/aerosentinel
ExecStart=/usr/bin/python3 /home/pi/aerosentinel/station_monitor.py
Restart=always
RestartSec=10
StandardOutput=journal
StandardError=journal

# Environment variables
Environment="SERIAL_PORT=/dev/serial0"
Environment="BAUD_RATE=115200"
Environment="FLASK_HOST=0.0.0.0"
Environment="FLASK_PORT=5000"

[Install]
WantedBy=multi-user.target
```

#### Step 3: Enable and Start Service

```bash
sudo systemctl daemon-reload
sudo systemctl enable aerosentinel.service
sudo systemctl start aerosentinel.service
```

#### Step 4: Check Status

```bash
sudo systemctl status aerosentinel.service
```

#### View Logs

```bash
journalctl -u aerosentinel.service -f
```

### Method 2: crontab (Alternative)

```bash
crontab -e
```

Add this line:
```
@reboot sleep 30 && /usr/bin/python3 /home/pi/aerosentinel/station_monitor.py &
```

### Method 3: rc.local (Legacy)

```bash
sudo nano /etc/rc.local
```

Add before `exit 0`:
```bash
# Wait for network and serial to be ready
sleep 20
su - pi -c "/usr/bin/python3 /home/pi/aerosentinel/station_monitor.py &"
```

---

## Wiring: ESP32-C3 to Raspberry Pi Zero

### Connection Table

| ESP32-C3 | Raspberry Pi Zero GPIO | Function |
|----------|------------------------|----------|
| GPIO 21 (TX) | Pin 10 (GPIO 15 RXD) | UART TX → RX |
| GPIO 20 (RX) | Pin 8 (GPIO 14 TXD) | UART RX ← TX |
| 5V | Pin 2 or Pin 4 (5V) | Power |
| GND | Pin 6 (GND) | Common Ground |

### GPIO Pinout Reference

```
Raspberry Pi Zero GPIO Header (Top View)
─────────────────────────────────────────
 3.3V  ─── [01] [02] ─── 5V
 SDA   ─── [03] [04] ─── 5V
 SCL   ─── [05] [06] ─── GND
 GPIO4 ─── [07] [08] ─── TXD (GPIO 14) ──► ESP32-C3 RX
 GND   ─── [09] [10] ─── RXD (GPIO 15) ◄── ESP32-C3 TX
 GPIO17── [11] [12] ─── GPIO18
 GPIO27── [13] [14] ─── GND
 GPIO22── [15] [16] ─── GPIO23
 3.3V  ─── [17] [18] ─── GPIO24
 MOSI  ─── [19] [20] ─── GND
 MISO  ─── [21] [22] ─── GPIO25
 SCLK  ─── [23] [24] ─── CE0
 GND   ─── [25] [26] ─── CE1
─────────────────────────────────────────
```

---

## WiFi Status Indication via ESP32-C3

The ESP32-C3 creates a WiFi Access Point for status indication:

### Connection Details

| Setting | Value |
|---------|-------|
| SSID | `AeroSentinel-Base` |
| Password | `sentinel123` |
| IP | `192.168.4.1` |

### Accessing the Status Page

1. Connect your phone/laptop to `AeroSentinel-Base` WiFi
2. Open browser: `http://192.168.4.1`
3. View real-time status of:
   - ESP-NOW connection
   - Packets received
   - Commands sent
   - Last telemetry data

---

## Testing the Setup

### Test 1: Check Serial Communication

```bash
# Install minicom for testing
sudo apt install minicom

# Connect to serial port
minicom -D /dev/serial0 -b 115200

# You should see heartbeat messages from ESP32-C3 every 5 seconds
# Press Ctrl+A then X to exit
```

### Test 2: Check Flask API

```bash
# On another device on the same network
curl http://aerosentinel-base.local:5000/api/health

# Should return:
# {"status": "healthy", "uptime": 123, "packets_received": 0}
```

### Test 3: Check Service Status

```bash
sudo systemctl status aerosentinel.service
```

Should show: `Active: active (running)`

---

## Troubleshooting

### Serial Port Not Working

```bash
# Check if serial port exists
ls -la /dev/serial0

# Should show: /dev/serial0 -> ttyAMA0

# Check for conflicts
sudo cat /proc/cmdline
# Should NOT contain console=serial0,115200
```

### Service Not Starting

```bash
# Check logs
journalctl -u aerosentinel.service -n 50

# Check Python syntax
python3 -m py_compile /home/pi/aerosentinel/station_monitor.py

# Run manually to debug
cd /home/pi/aerosentinel
python3 station_monitor.py
```

### Permission Denied on Serial Port

```bash
# Add pi user to dialout group
sudo usermod -a -G dialout pi

# Logout and login again
```

### WiFi AP Not Visible

1. Check ESP32-C3 is powered on
2. Wait 30 seconds after Pi boots
3. Verify ESP32-C3 firmware has AP mode enabled

---

## Creating a Custom OS Image (Optional)

To create a pre-configured image for easy deployment:

### Step 1: Configure the System

Complete all setup steps above on a test SD card.

### Step 2: Shrink the Filesystem

```bash
sudo apt clean
sudo apt autoremove
sudo zerofree /dev/mmcblk0p2
# (Run from another system with the SD card mounted)
```

### Step 3: Create Image

On a Linux computer:
```bash
# Insert SD card and find device
lsblk

# Create image (replace sdX with your device)
sudo dd if=/dev/sdX of=aerosentinel-base.img bs=4M status=progress
```

### Step 4: Compress for Distribution

```bash
gzip aerosentinel-base.img
```

---

## Power-On Behavior Summary

When power is connected to the Raspberry Pi Zero:

1. **Boot** (30-60 seconds)
   - Raspberry Pi OS loads
   - Network interfaces initialize
   - Serial port becomes available

2. **Service Start** (after boot)
   - `aerosentinel.service` starts automatically
   - Python script begins execution
   - Serial connection to ESP32-C3 established

3. **Normal Operation**
   - Listens for telemetry from ESP32-C3
   - Serves REST API on port 5000
   - Stores data in SQLite database
   - Generates reports when rover docks

4. **Status Indication**
   - ESP32-C3 creates WiFi AP: `AeroSentinel-Base`
   - Status page available at `http://192.168.4.1`

---

## Next Steps

After setting up the Raspberry Pi Zero:
1. Verify ESP32-C3 connection with `minicom`
2. Test the API endpoints
3. Flash the [ESP32-C3 bridge](./ESP32-C3_INSTRUCTIONS.md)
4. Flash the [ESP32-S3 rover](./ESP32-S3_INSTRUCTIONS.md)
5. Test end-to-end communication

---

*Document Version: 1.0*  
*Last Updated: February 2026*

## Overview

This guide sets up the Raspberry Pi Zero to:
- Automatically run the station monitor Python script on power-up
- Connect to the ESP32-C3 via UART for rover communication
- Create a WiFi hotspot through the ESP32-C3 for status indication
- Operate in headless mode (no monitor/keyboard required)

---

## Hardware Requirements

### Components
- Raspberry Pi Zero (any version: Pi Zero, Pi Zero W, or Pi Zero 2 W)
- MicroSD Card (8GB minimum, 16GB recommended)
- 5V Power Supply (2.5A recommended)
- ESP32-C3 (connected via UART)

### Optional
- USB to TTL adapter (for initial headless setup)
- HDMI adapter and monitor (for debugging)

---

## Operating System Setup

### Step 1: Download Raspberry Pi OS

Download **Raspberry Pi OS Lite** (no desktop environment needed):
- URL: https://www.raspberrypi.com/software/operating-systems/
- Choose: **Raspberry Pi OS Lite (64-bit or 32-bit)**

### Step 2: Flash the SD Card

Use **Raspberry Pi Imager** or **balenaEtcher**:

#### Using Raspberry Pi Imager (Recommended)

1. Download from: https://www.raspberrypi.com/software/
2. Insert SD card into computer
3. Open Raspberry Pi Imager
4. Select OS: **Raspberry Pi OS (other)** → **Raspberry Pi OS Lite**
5. Select your SD card
6. Click the **gear icon** (Advanced options):
   - Set hostname: `aerosentinel-base`
   - Enable SSH: **Yes**
   - Set username/password: `pi` / `sentinel2024`
   - Configure wireless LAN: (optional, for internet access)
   - Set locale settings: Your timezone
7. Click **Save** then **Write**

#### Using balenaEtcher

1. Download from: https://www.balena.io/etcher/
2. Flash the downloaded `.img` file to SD card
3. After flashing, open the `boot` partition
4. Create an empty file named `ssh` (no extension) to enable SSH
5. Create `wpa_supplicant.conf` for WiFi (optional):

```conf
country=US
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1

network={
    ssid="YourWiFiName"
    psk="YourWiFiPassword"
}
```

---

## Initial Boot and Configuration

### Step 1: Boot the Pi

1. Insert SD card into Raspberry Pi Zero
2. Connect ESP32-C3 to GPIO pins (see wiring below)
3. Connect power (micro-USB)
4. Wait 2-3 minutes for first boot

### Step 2: Connect via SSH

```bash
ssh pi@aerosentinel-base.local
# Password: sentinel2024
```

Or use IP address if hostname doesn't resolve:
```bash
ssh pi@192.168.1.xxx
```

### Step 3: Update System

```bash
sudo apt update
sudo apt upgrade -y
```

### Step 4: Enable Serial Port

```bash
sudo raspi-config
```

Navigate to:
1. **Interface Options** → **Serial Port**
2. Login shell on serial: **No**
3. Serial port hardware enabled: **Yes**
4. Reboot when prompted

---

## Install Required Packages

### Step 1: Install Python Dependencies

```bash
sudo apt install -y python3-pip python3-venv
sudo pip3 install flask flask-cors pyserial fpdf2
```

### Step 2: Create Project Directory

```bash
mkdir -p ~/aerosentinel
cd ~/aerosentinel
```

### Step 3: Copy Station Monitor Script

Copy the `station_monitor.py` file to the Pi:

**From your computer:**
```bash
scp RoverProject/base_station/station_monitor.py pi@aerosentinel-base.local:~/aerosentinel/
```

**Or create directly on Pi:**
```bash
nano ~/aerosentinel/station_monitor.py
# Paste the contents
```

---

## Auto-Start Configuration

### Method 1: systemd Service (Recommended)

Create a systemd service to auto-start the script:

#### Step 1: Create Service File

```bash
sudo nano /etc/systemd/system/aerosentinel.service
```

#### Step 2: Add Service Configuration

```ini
[Unit]
Description=Aero Sentinel Base Station Monitor
After=network.target serial-getty@serial0.service
Wants=network-online.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/aerosentinel
ExecStart=/usr/bin/python3 /home/pi/aerosentinel/station_monitor.py
Restart=always
RestartSec=10
StandardOutput=journal
StandardError=journal

# Environment variables
Environment="SERIAL_PORT=/dev/serial0"
Environment="BAUD_RATE=115200"
Environment="FLASK_HOST=0.0.0.0"
Environment="FLASK_PORT=5000"

[Install]
WantedBy=multi-user.target
```

#### Step 3: Enable and Start Service

```bash
sudo systemctl daemon-reload
sudo systemctl enable aerosentinel.service
sudo systemctl start aerosentinel.service
```

#### Step 4: Check Status

```bash
sudo systemctl status aerosentinel.service
```

#### View Logs

```bash
journalctl -u aerosentinel.service -f
```

### Method 2: crontab (Alternative)

```bash
crontab -e
```

Add this line:
```
@reboot sleep 30 && /usr/bin/python3 /home/pi/aerosentinel/station_monitor.py &
```

### Method 3: rc.local (Legacy)

```bash
sudo nano /etc/rc.local
```

Add before `exit 0`:
```bash
# Wait for network and serial to be ready
sleep 20
su - pi -c "/usr/bin/python3 /home/pi/aerosentinel/station_monitor.py &"
```

---

## Wiring: ESP32-C3 to Raspberry Pi Zero

### Connection Table

| ESP32-C3 | Raspberry Pi Zero GPIO | Function |
|----------|------------------------|----------|
| GPIO 21 (TX) | Pin 10 (GPIO 15 RXD) | UART TX → RX |
| GPIO 20 (RX) | Pin 8 (GPIO 14 TXD) | UART RX ← TX |
| 5V | Pin 2 or Pin 4 (5V) | Power |
| GND | Pin 6 (GND) | Common Ground |

### GPIO Pinout Reference

```
Raspberry Pi Zero GPIO Header (Top View)
─────────────────────────────────────────
 3.3V  ─── [01] [02] ─── 5V
 SDA   ─── [03] [04] ─── 5V
 SCL   ─── [05] [06] ─── GND
 GPIO4 ─── [07] [08] ─── TXD (GPIO 14) ──► ESP32-C3 RX
 GND   ─── [09] [10] ─── RXD (GPIO 15) ◄── ESP32-C3 TX
 GPIO17── [11] [12] ─── GPIO18
 GPIO27── [13] [14] ─── GND
 GPIO22── [15] [16] ─── GPIO23
 3.3V  ─── [17] [18] ─── GPIO24
 MOSI  ─── [19] [20] ─── GND
 MISO  ─── [21] [22] ─── GPIO25
 SCLK  ─── [23] [24] ─── CE0
 GND   ─── [25] [26] ─── CE1
─────────────────────────────────────────
```

---

## WiFi Status Indication via ESP32-C3

The ESP32-C3 creates a WiFi Access Point for status indication:

### Connection Details

| Setting | Value |
|---------|-------|
| SSID | `AeroSentinel-Base` |
| Password | `sentinel123` |
| IP | `192.168.4.1` |

### Accessing the Status Page

1. Connect your phone/laptop to `AeroSentinel-Base` WiFi
2. Open browser: `http://192.168.4.1`
3. View real-time status of:
   - ESP-NOW connection
   - Packets received
   - Commands sent
   - Last telemetry data

---

## Testing the Setup

### Test 1: Check Serial Communication

```bash
# Install minicom for testing
sudo apt install minicom

# Connect to serial port
minicom -D /dev/serial0 -b 115200

# You should see heartbeat messages from ESP32-C3 every 5 seconds
# Press Ctrl+A then X to exit
```

### Test 2: Check Flask API

```bash
# On another device on the same network
curl http://aerosentinel-base.local:5000/api/health

# Should return:
# {"status": "healthy", "uptime": 123, "packets_received": 0}
```

### Test 3: Check Service Status

```bash
sudo systemctl status aerosentinel.service
```

Should show: `Active: active (running)`

---

## Troubleshooting

### Serial Port Not Working

```bash
# Check if serial port exists
ls -la /dev/serial0

# Should show: /dev/serial0 -> ttyAMA0

# Check for conflicts
sudo cat /proc/cmdline
# Should NOT contain console=serial0,115200
```

### Service Not Starting

```bash
# Check logs
journalctl -u aerosentinel.service -n 50

# Check Python syntax
python3 -m py_compile /home/pi/aerosentinel/station_monitor.py

# Run manually to debug
cd /home/pi/aerosentinel
python3 station_monitor.py
```

### Permission Denied on Serial Port

```bash
# Add pi user to dialout group
sudo usermod -a -G dialout pi

# Logout and login again
```

### WiFi AP Not Visible

1. Check ESP32-C3 is powered on
2. Wait 30 seconds after Pi boots
3. Verify ESP32-C3 firmware has AP mode enabled

---

## Creating a Custom OS Image (Optional)

To create a pre-configured image for easy deployment:

### Step 1: Configure the System

Complete all setup steps above on a test SD card.

### Step 2: Shrink the Filesystem

```bash
sudo apt clean
sudo apt autoremove
sudo zerofree /dev/mmcblk0p2
# (Run from another system with the SD card mounted)
```

### Step 3: Create Image

On a Linux computer:
```bash
# Insert SD card and find device
lsblk

# Create image (replace sdX with your device)
sudo dd if=/dev/sdX of=aerosentinel-base.img bs=4M status=progress
```

### Step 4: Compress for Distribution

```bash
gzip aerosentinel-base.img
```

---

## Power-On Behavior Summary

When power is connected to the Raspberry Pi Zero:

1. **Boot** (30-60 seconds)
   - Raspberry Pi OS loads
   - Network interfaces initialize
   - Serial port becomes available

2. **Service Start** (after boot)
   - `aerosentinel.service` starts automatically
   - Python script begins execution
   - Serial connection to ESP32-C3 established

3. **Normal Operation**
   - Listens for telemetry from ESP32-C3
   - Serves REST API on port 5000
   - Stores data in SQLite database
   - Generates reports when rover docks

4. **Status Indication**
   - ESP32-C3 creates WiFi AP: `AeroSentinel-Base`
   - Status page available at `http://192.168.4.1`

---

## Next Steps

After setting up the Raspberry Pi Zero:
1. Verify ESP32-C3 connection with `minicom`
2. Test the API endpoints
3. Flash the [ESP32-C3 bridge](./ESP32-C3_INSTRUCTIONS.md)
4. Flash the [ESP32-S3 rover](./ESP32-S3_INSTRUCTIONS.md)
5. Test end-to-end communication

---

*Document Version: 1.0*  
*Last Updated: February 2026*

