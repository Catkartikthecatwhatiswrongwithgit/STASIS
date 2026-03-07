# STASIS — Quick Reference Card

---

## Commands

| Command | Action |
|---------|--------|
| `STOP` | Emergency stop → SAFE_IDLE |
| `FORWARD` | Drive forward |
| `BACKWARD` | Drive in reverse |
| `LEFT` / `RIGHT` | Turn |
| `PATROL` | Start autonomous patrol |
| `STOP_PATROL` | Stop patrol → SAFE_IDLE |
| `HOME` / `RETURN_BASE` | Return to dock |
| `DOCKING` | Enable AprilTag docking |
| `DISABLE_DOCKING` | Cancel docking |
| `SETHOME` | Set current GPS as home (requires fix) |
| `RESET` | Reset all flags and state |

---

## States

| State | LED | Buzzer |
|-------|-----|--------|
| `SAFE_IDLE` | 1 Hz slow blink | Off |
| `PATROL` | Solid | Off |
| `ALERT` | 5 Hz fast blink | 2 Hz |
| `RETURN_TO_BASE` | 2 Hz medium blink | Off |
| `DOCKING` | Off | Off |
| `DOCKED_CHARGING` | Solid | Off |

---

## Pin Map (ESP32-S3)

```
Motors:      2, 4, 5, 6, 7, 15, 16, 18   (L9110S 4WD)
Ultrasonic:  9 (ECHO), 10 (TRIG)          (HC-SR04)
GPS:         8 (RX), 19 (TX)              (NEO-6M, Serial2)
SIM800:      12 (RX), 13 (TX)             (Serial1 — NOT Serial2)
DS18B20:     21                            (OneWire + 4.7k pull-up)
Battery ADC: 1                             (30k/10k divider)
Buzzer:      14
Status LED:  45
UART to Pi:  43 (TX), 44 (RX)
```

> Serial2 = GPS only. Serial1 = SIM800 only. They must not be swapped.

---

## Safety Thresholds

| Parameter | Value |
|-----------|-------|
| Obstacle stop | < 25 cm |
| Geofence radius | 50 m |
| GPS staleness halt | > 5 s (after 90 s cold-start grace) |
| Dock confirm voltage | +0.4 V above baseline |
| Dock debounce | 20 frames (~2 s) |
| Low battery speed limit | 20 % → 60 % speed cap |
| Hard voltage cutoff | 6.2 V |
| Charge complete | 8.4 V or 10 min |
| SMS cooldown | 60 s |

---

## AprilTag Docking Checklist

- [ ] Tag family: tag36h11, ID 0, size 10 cm
- [ ] Tag mounted perpendicular to approach path
- [ ] Consistent lighting on tag face
- [ ] `pip install pupil-apriltags opencv-python-headless` on Pi
- [ ] `CAM_STREAM_URL` set correctly (default: `http://192.168.4.1:81/stream`)
- [ ] Camera stream reachable: `curl -I http://192.168.4.1:81/stream`
- [ ] Rover reports `STATE_DOCKING` in telemetry to activate thread

---

## Dashboard Access

Connect to WiFi AP: **Stasis-Base** (password: `stasis123`)

```
Dashboard:  http://192.168.4.1:5000/stasis_app/index.html
API status: http://192.168.4.1:5000/api/health
API key:    stasis-2024
```

Keyboard shortcuts in dashboard:

| Key | Action |
|-----|--------|
| `1`–`6` | Switch panel |
| `Space` | STOP |
| `P` | PATROL |
| `H` | HOME |
| `F` | Fencing panel |

---

## Common Problems

| Symptom | Fix |
|---------|-----|
| Rover won't stop in emergency | Flash v2 firmware — `estop()` was broken |
| BACKWARD drives forward | Flash v2 firmware — speed sign was wrong |
| GPS halts rover on boot | Flash v2 firmware — 90 s grace period added |
| SMS sends but GPS garbled | SIM800 must be on Serial1, GPS on Serial2 |
| Docking doesn't steer | Check `binaryMode` flag in `uartTask` |
| `L is not defined` on dashboard | Cache Leaflet locally on Pi |
| Chat not working | Socket.IO required — not raw WebSocket |
| All dashboard icons blank | Inline SVG fix in v2 dashboard |
| Map shows in wrong place | Check GPS home fix, verify lat/lng sign |

---

## Useful Pi Commands

```bash
# Check service status
sudo systemctl status stasis

# View live logs
journalctl -u stasis -f

# Restart service
sudo systemctl restart stasis

# Test serial comms
minicom -D /dev/serial0 -b 115200

# Send a manual command
curl -X POST http://localhost:5000/api/command \
  -H "Content-Type: application/json" \
  -H "X-API-Key: stasis-2024" \
  -d '{"cmd":"STOP"}'
```

---

*Last updated: March 2026 — v2*
