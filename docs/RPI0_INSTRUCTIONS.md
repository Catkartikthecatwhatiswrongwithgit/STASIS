# Raspberry Pi Zero — Base Station Setup

## Overview

The Pi Zero runs `station_monitor.py`, which handles:
- Reading JSON telemetry from the ESP32-C3 bridge over UART
- Serving the REST API and web dashboard on port 5000
- Storing telemetry in a SQLite database
- Generating PDF mission reports when the rover docks
- Running the AprilTag visual docking control loop

---

## OS Setup

### 1. Flash Raspberry Pi OS Lite

Use **Raspberry Pi Imager** (https://www.raspberrypi.com/software/):

- OS: **Raspberry Pi OS Lite (32-bit)**
- Click the gear icon before writing to pre-configure:
  - Hostname: `stasis-base`
  - Enable SSH: Yes
  - Username / password: `pi` / `sentinel2024`
  - Locale: your timezone

### 2. Enable the serial port (disable serial console)

```bash
sudo raspi-config
# Interface Options → Serial Port
#   Login shell over serial: No
#   Serial port hardware enabled: Yes
# Reboot
```

---

## Install Dependencies

```bash
sudo apt update
sudo apt install -y python3-pip

# Core station monitor
pip install flask flask-cors flask-socketio pyserial fpdf2 eventlet

# AprilTag docking (optional but recommended)
pip install pupil-apriltags opencv-python-headless

# Cache Leaflet maps for offline use (run once with internet access)
mkdir -p ~/stasis/static
wget -O ~/stasis/static/leaflet.css https://unpkg.com/leaflet@1.9.4/dist/leaflet.css
wget -O ~/stasis/static/leaflet.js  https://unpkg.com/leaflet@1.9.4/dist/leaflet.js
```

---

## Copy the Project

```bash
# From your computer (replace with actual path)
scp -r STASIS/ pi@stasis-base.local:~/stasis/
```

---

## Auto-Start with systemd

```bash
sudo nano /etc/systemd/system/stasis.service
```

```ini
[Unit]
Description=STASIS Base Station Monitor
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/stasis
ExecStart=/usr/bin/python3 /home/pi/stasis/base_station/station_monitor.py
Restart=always
RestartSec=10
StandardOutput=journal
StandardError=journal

Environment="SERIAL_PORT=/dev/serial0"
Environment="BAUD_RATE=115200"
Environment="FLASK_HOST=0.0.0.0"
Environment="FLASK_PORT=5000"
Environment="CAM_STREAM_URL=http://192.168.4.1:81/stream"

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl daemon-reload
sudo systemctl enable stasis
sudo systemctl start stasis
```

---

## Wiring (ESP32-C3 → Pi Zero)

| ESP32-C3 | Raspberry Pi Zero | Function |
|----------|------------------|----------|
| GPIO 21 (TX) | Pin 10 (GPIO 15 / RXD) | UART data to Pi |
| GPIO 20 (RX) | Pin 8 (GPIO 14 / TXD) | UART data from Pi |
| 5V | Pin 2 (5V) | Power from Pi |
| GND | Pin 6 (GND) | Common ground |

> Only the bottom row of the Pi GPIO header is used (even-numbered pins: 2, 4, 6, 8, 10).

---

## Testing

### Verify serial comms

```bash
sudo apt install minicom
minicom -D /dev/serial0 -b 115200
# Should see JSON telemetry lines scrolling
# Press Ctrl+A then X to exit
```

### Verify the API

```bash
# Public health check
curl http://localhost:5000/api/health

# Authenticated status
curl -H "X-API-Key: stasis-2024" http://localhost:5000/api/status
```

### Access the dashboard

Connect to the `Stasis-Base` WiFi AP, then open:
```
http://192.168.4.1:5000/stasis_app/index.html
```

### Check service logs

```bash
journalctl -u stasis -f
```

### Send a test command

```bash
curl -X POST http://localhost:5000/api/command \
  -H "Content-Type: application/json" \
  -H "X-API-Key: stasis-2024" \
  -d '{"cmd": "STOP"}'
```

---

## Boot Sequence

1. Pi OS boots (~30–60 s)
2. `stasis.service` starts automatically
3. Serial listener thread opens `/dev/serial0`
4. Flask API starts on `0.0.0.0:5000`
5. AprilTag thread starts (waits for `STATE_DOCKING` telemetry)
6. Dashboard is accessible from any device on `Stasis-Base` WiFi

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| `/dev/serial0` not found | Run `raspi-config` → Interface Options → Serial Port → disable login shell, enable hardware |
| Permission denied on serial | `sudo usermod -a -G dialout pi` then logout/login |
| Service not starting | `journalctl -u stasis -n 50` to see the error |
| AprilTag thread disabled | `pip install pupil-apriltags opencv-python-headless` |
| Camera stream not opening | Check `CAM_STREAM_URL` in service env; verify ESP32-CAM is on AP |
| Dashboard map broken | Run the Leaflet cache `wget` commands (needs internet once) |

---

*Last updated: March 2026 — v2*
