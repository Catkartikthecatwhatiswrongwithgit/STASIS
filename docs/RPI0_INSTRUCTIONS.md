# Raspberry Pi Zero - Base Station Setup Instructions

## Overview

This guide sets up the Raspberry Pi Zero to:
- Automatically run the station monitor Python script on power-up
- Connect to the ESP32-C3 via UART for rover communication
- Serve the web dashboard and REST API
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

Use **Raspberry Pi Imager**:

1. Download from: https://www.raspberrypi.com/software/
2. Insert SD card into computer
3. Open Raspberry Pi Imager
4. Select OS: **Raspberry Pi OS (other)** → **Raspberry Pi OS Lite**
5. Select your SD card
6. Click the **gear icon** (Advanced options):
   - Set hostname: `stasis-base`
   - Enable SSH: **Yes**
   - Set username/password: `pi` / `sentinel2024`
   - Configure wireless LAN: (optional, for internet access)
   - Set locale settings: Your timezone
7. Click **Save** then **Write**

### Step 3: Enable Serial Port

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
sudo pip3 install flask flask-cors flask-socketio pyserial fpdf2 eventlet
```

### Step 2: Copy Station Monitor Script

```bash
mkdir -p ~/Stasis
cd ~/Stasis
scp -r base_station pi@stasis-base.local:~/Stasis/
```

---

## Auto-Start Configuration

### systemd Service (Recommended)

Create a systemd service:

```bash
sudo nano /etc/systemd/system/stasis.service
```

Add:

```ini
[Unit]
Description=Stasis Base Station Monitor
After=network.target serial-getty@serial0.service

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/Stasis
ExecStart=/usr/bin/python3 /home/pi/Stasis/base_station/station_monitor.py
Restart=always
RestartSec=10
StandardOutput=journal
StandardError=journal

Environment="SERIAL_PORT=/dev/serial0"
Environment="BAUD_RATE=115200"
Environment="FLASK_HOST=0.0.0.0"
Environment="FLASK_PORT=5000"

[Install]
WantedBy=multi-user.target
```

Enable:

```bash
sudo systemctl daemon-reload
sudo systemctl enable stasis.service
sudo systemctl start stasis.service
```

---

## Wiring: ESP32-C3 to Raspberry Pi Zero

| ESP32-C3 | Raspberry Pi Zero | Function |
|----------|------------------|----------|
| GPIO 21 (TX) | Pin 10 (GPIO 15 RXD) | UART TX → RX |
| GPIO 20 (RX) | Pin 8 (GPIO 14 TXD) | UART RX ← TX |
| 5V | Pin 2 or 4 (5V) | Power |
| GND | Pin 6 (GND) | Common Ground |

---

## Testing the Setup

### Test 1: Check Serial Communication

```bash
sudo apt install minicom
minicom -D /dev/serial0 -b 115200
```

You should see telemetry JSON from ESP32-C3. Press Ctrl+A then X to exit.

### Test 2: Check Flask API

```bash
# Public endpoint (no auth required)
curl http://stasis-base.local:5000/api/health

# Authenticated endpoints (default key: stasis-2024)
curl -H "X-API-Key: stasis-2024" http://stasis-base.local:5000/api/status
```

### Test 3: Access Web Dashboard

Open browser:
```
http://stasis-base.local:5000/stasis_app/index.html
```

---

## API Usage

### Send Command to Rover

```bash
curl -X POST http://stasis-base.local:5000/api/command \
  -H "Content-Type: application/json" \
  -H "X-API-Key: stasis-2024" \
  -d '{"cmd":"PATROL"}'
```

Available commands: STOP, FORWARD, BACKWARD, LEFT, RIGHT, PATROL, HOME, DOCKING, RESEARCH, RESET

### Get Telemetry

```bash
curl http://stasis-base.local:5000/api/status \
  -H "X-API-Key: stasis-2024"
```

### Get Alerts

```bash
curl http://stasis-base.local:5000/api/alerts \
  -H "X-API-Key: stasis-2024"
```

---

## Troubleshooting

### Serial Port Not Found

```bash
ls -la /dev/serial0
# Should show: /dev/serial0 -> ttyAMA0
```

### Service Not Starting

```bash
journalctl -u stasis.service -n 50
python3 -m py_compile ~/Stasis/base_station/station_monitor.py
```

### Permission Denied

```bash
sudo usermod -a -G dialout pi
# Logout and login again
```

---

## Power-On Behavior

1. **Boot** (30-60 seconds) - Raspberry Pi OS loads
2. **Service Start** - stasis.service starts automatically
3. **Normal Operation**:
   - Listens for telemetry from ESP32-C3
   - Serves REST API on port 5000
   - Stores data in SQLite database
   - Generates PDF/TXT reports when rover docks
   - WebSocket for real-time updates

---

## Next Steps

After setting up the Raspberry Pi Zero:
1. Verify ESP32-C3 connection with `minicom`
2. Flash the [ESP32-C3 bridge](./ESP32-C3_INSTRUCTIONS.md)
3. Flash the [ESP32-S3 rover](./ESP32-S3_INSTRUCTIONS.md)
4. Test end-to-end communication

---

*Document Version: 2.0*  
*Last Updated: February 2026*
