# AprilTag Docking Guide

## Overview

STASIS uses AprilTag visual markers to guide the rover into precise contact with the charging dock. The Raspberry Pi Zero detects the tag from the ESP32-CAM's live stream and runs a proportional steering controller, sending binary commands to the rover in real time.

---

## How It Works

```
ESP32-CAM  →  MJPEG stream  →  Pi Zero (pupil-apriltags)
                                       │
                              Detect tag36h11 ID 0
                              Measure X error (px) and Z depth (m)
                                       │
                              Proportional controller:
                                turn  = clamp(err_x × 0.6, -100, +100)
                                speed = clamp(dist_m × 40, 10, 40)
                                dead-band: if |err_x| < 15 px → turn = 0
                                           if dist_m  < 0.25 m → DOCKED
                                       │
                              6-byte binary DockCommand
                              [0xAA | turn | speed | docked | XOR | 0x55]
                                       │
                              UART → ESP32-C3 → ESP-NOW → ESP32-S3 → motors
```

The docking thread in `station_monitor.py` starts automatically when the rover's telemetry reports `STATE_DOCKING`. It stops when the rover leaves that state or confirms a dock.

---

## Hardware Setup

### Tag Preparation

- **Family:** tag36h11
- **ID:** 0 (primary dock)
- **Printed size:** 10 cm × 10 cm minimum
- **Paper:** Matte, high contrast, black on white
- **Laminate** for durability and glare reduction
- **White border** of at least one tag-width around the printed marker

Download the tag image: https://github.com/AprilRobotics/apriltag-imgs

### Tag Placement

Mount the tag on the docking station face-on to the rover's approach path:

- Height: 20–40 cm from ground (centred with the camera)
- Angle: perpendicular to the approach — do not tilt the tag
- Lighting: ensure consistent, direct light — avoid backlighting

### Camera Mounting

The ESP32-CAM should be mounted at the front of the rover:

- Facing forward, level with the tag height when the rover is at the dock
- Fixed — no flex or vibration in the mount
- Unobstructed field of view

---

## Software Setup

### Install dependencies (Pi Zero, once)

```bash
pip install pupil-apriltags opencv-python-headless
```

### Set the camera stream URL

The docking thread reads `CAM_STREAM_URL` from the environment. Set it in the systemd service or shell:

```bash
export CAM_STREAM_URL=http://192.168.4.1:81/stream
```

Default if not set: `http://192.168.4.1:81/stream`

### Verify the stream is reachable

```bash
curl -I http://192.168.4.1:81/stream
# Should return HTTP/1.1 200 OK
```

---

## Configuration Parameters

These constants are near the top of the AprilTag section in `station_monitor.py`:

| Constant | Default | Description |
|----------|---------|-------------|
| `APRILTAG_ID` | 0 | Tag ID to dock to |
| `APRILTAG_SIZE_M` | 0.10 | Physical tag side length in metres |
| `CAM_FX`, `CAM_FY` | 320.0 | Camera focal length in pixels |
| `CAM_CX`, `CAM_CY` | 160.0, 120.0 | Principal point (QVGA centre) |
| `DOCK_CENTRE_TOL` | 15 | X dead-band in pixels before steering |
| `DOCK_CLOSE_DIST` | 0.25 | Distance in metres to declare docked |

> **Calibrating focal length:** For accurate distance estimation, measure the tag at a known distance and adjust `CAM_FX`/`CAM_FY` so the reported Z depth matches reality.

---

## Docking Sequence

### Phase 1 — Command
Send `DOCKING` command from dashboard or API. Rover transitions to `STATE_DOCKING`.

### Phase 2 — Search
If the tag is not immediately visible, the rover rotates slowly (turn=30, speed=15) until the tag enters the camera frame.

### Phase 3 — Approach
Once the tag is detected:
- Steering corrects X alignment proportionally
- Speed decreases as distance closes
- Dead-band of ±15 px prevents steering oscillation when nearly centred

### Phase 4 — Contact
When `dist_m < 0.25 m`, a final `docked=1` packet is sent to halt the rover. The rover's dock detection (battery voltage rise) confirms actual electrical contact and transitions to `STATE_DOCKED_CHARGING`.

---

## Dashboard Integration

When the docking thread detects a tag, it emits a Socket.IO event to all connected dashboard clients:

```json
{ "found": true,  "tag_id": 0, "err_x": -12.3, "dist_m": 0.87 }
{ "found": false }
```

The dashboard displays docking status in real time on the Dashboard and Map panels.

---

## Troubleshooting

| Problem | Cause | Fix |
|---------|-------|-----|
| Tag never detected | Poor lighting or tag too small | Improve lighting; check 10 cm print size |
| Rover oscillates left/right | Dead-band too small | Increase `DOCK_CENTRE_TOL` from 15 to 25 |
| Rover overshoots the dock | Speed too high near contact | Decrease the speed multiplier (`dist_m × 40`) |
| Wrong distance reported | Focal length not calibrated | Measure actual distance vs reported; scale `CAM_FX`/`CAM_FY` |
| `APRILTAG_AVAILABLE = False` | Library not installed | `pip install pupil-apriltags opencv-python-headless` |
| Camera stream not opening | Wrong IP or ESP32-CAM offline | Verify `CAM_STREAM_URL`; check camera AP is up |
| Docking thread doesn't activate | Rover not in DOCKING state | Check telemetry `status` field contains `DOCKING` |

---

## Testing Without the Rover

Print the tag and hold it in front of the camera at arm's length. Watch the Pi logs:

```
AprilTag: rover in DOCKING state — opening camera stream
AprilTag: id=0 err_x=23.5 dist=0.94m
AprilTag: id=0 err_x=2.1  dist=0.41m
AprilTag: DOCKED (dist=0.22m) — stopping motors
```

You can also monitor the dashboard's Socket.IO `apriltag` events in the browser console.

---

*Last updated: March 2026 — v2*
