# SSH into Raspberry Pi Zero - Wired Connection Guide

This guide covers how to SSH into your Raspberry Pi Zero from your computer using a wired USB connection (no WiFi required).

---

## Method 1: USB Ethernet (Recommended)

This method uses the Pi's Micro-USB port to create an Ethernet over USB connection.

### On Raspberry Pi Zero (One-Time Setup)

1. **Enable Ethernet over USB:**
   
   Add these lines to `/boot/config.txt` (on the SD card):
   ```
   dtoverlay=dwc2
   ```
   
   Add this to `/boot/cmdline.txt` after `rootwait`:
   ```
   modules=loop,snd-usb-audio,dwc2,g_ether
   ```

2. **Enable SSH:**
   
   Create an empty file named `ssh` in the boot partition:
   ```
   touch /boot/ssh
   ```

3. **Set a static hostname (optional):**
   
   Edit `/etc/hostname`:
   ```
   stasis-base
   ```

4. **Set up a username/password:**
   
   The default is `pi` / `raspberry`. Change it:
   ```bash
   sudo raspi-config
   # System Options → Password
   ```

### On Your Computer

#### Windows:
1. Install RNDIS/Ethernet gadget driver:
   - Download from: https://github.com/pbatard/libwdi/wiki/USB-RNDIS-Drivers
   - Or use Zadig: https://zadig.akeo.ie/

2. Connect USB cable to **USB** port (not PWR) on Pi Zero

3. Wait for "USB Ethernet/RNDIS" to appear in Network Adapters

4. SSH using:
   ```
   ssh pi@raspberrypi.local
   ```
   Or if you set a hostname:
   ```
   ssh pi@stasis-base.local
   ```

#### macOS:
1. Connect USB cable to **USB** port (not PWR) on Pi Zero

2. Open Terminal and SSH:
   ```bash
   ssh pi@raspberrypi.local
   ```
   Or:
   ```bash
   ssh pi@stasis-base.local
   ```

#### Linux:
1. Connect USB cable to **USB** port (not PWR) on Pi Zero

2. SSH:
   ```bash
   ssh pi@raspberrypi.local
   ```

---

## Method 2: Serial Console (TTL Adapter)

If you have a USB-to-TTL adapter (like FTDI), you can access the Pi's console.

### Wiring

| USB-TTL Adapter | Raspberry Pi Zero |
|-----------------|-------------------|
| RX (Green) | Pin 8 (TXD/GPIO 14) |
| TX (White) | Pin 10 (RXD/GPIO 15) |
| 5V (Red) | Pin 2 or 4 (5V) - **ONLY IF POWERING FROM ADAPTER** |
| GND (Black) | Pin 6 (GND) |

**⚠️ IMPORTANT: Don't connect 5V if Pi is powered separately!**

### Connection

1. Connect adapter to your computer
2. Use a terminal program:
   - **Windows:** PuTTY, set COM port at 115200 baud
   - **macOS/Linux:** 
     ```bash
     screen /dev/ttyUSB0 115200
     ```
     Or:
     ```bash
     minicom -D /dev/ttyUSB0 -b 115200
     ```

3. Login with: `pi` / `raspberry`

---

## Method 3: Ethernet Hat / USB Hub

If you have an Ethernet USB hub or Ethernet HAT:

1. Connect Ethernet cable from your router/switch to the USB Ethernet adapter
2. Find the IP address:
   ```bash
   # From your router's admin page
   # Or scan your network:
   nmap -sn 192.168.1.0/24
   ```
3. SSH:
   ```bash
   ssh pi@192.168.1.xxx
   ```

---

## Quick Reference

| Method | Port to Use | Hostname | Speed |
|--------|-------------|----------|-------|
| USB Ethernet | N/A | `raspberrypi.local` | Fast |
| Serial (TTL) | COMx or /dev/ttyUSB0 | N/A | Slow |
| Ethernet | N/A | IP address | Fastest |

---

## Troubleshooting

### Can't connect via USB Ethernet?

**Windows:**
1. Check Device Manager for "USB Ethernet/RNDIS Gadget"
2. If not showing, install driver manually

**All OS:**
1. Try pinging the hostname:
   ```bash
   ping raspberrypi.local
   ```
2. Check if Pi is getting power (LED should be on)

### SSH Connection Refused?

1. Make sure the `ssh` file exists in boot partition
2. Try:
   ```bash
   sudo systemctl enable ssh
   sudo systemctl start ssh
   ```

### Wrong Password?

The default password is `raspberry`. If changed and forgotten:
1. Re-flash SD card
2. Or edit `/etc/shadow` from Linux computer

---

## SSH Key Setup (Recommended)

To avoid typing password each time:

```bash
# On your computer, generate key:
ssh-keygen -t ed25519

# Copy to Pi:
ssh-copy-id pi@raspberrypi.local
```

---

## After Connecting - First Time Setup

```bash
# Update system
sudo apt update
sudo apt upgrade -y

# Set up WiFi (optional)
sudo raspi-config
# System Options → Wireless LAN

# Configure serial port for ESP32-C3
sudo raspi-config
# Interface Options → Serial Port
# Login shell: No
# Serial port: Yes

# Install dependencies
sudo apt install -y python3-pip
pip3 install flask flask-cors flask-socketio pyserial fpdf2 eventlet

# Copy station_monitor.py
mkdir -p ~/Stasis
# Copy base_station folder to ~/Stasis/
```

---

## Connect to Your STASIS Base Station

Once set up:

```bash
# SSH into Pi
ssh pi@stasis-base.local

# Check if station_monitor is running
sudo systemctl status stasis

# View logs
journalctl -u stasis -f

# Restart service
sudo systemctl restart stasis
```

Access the dashboard:
```
http://stasis-base.local:5000/stasis_app/index.html
```

---

*Document Version: 1.0*
*Last Updated: February 2026*
