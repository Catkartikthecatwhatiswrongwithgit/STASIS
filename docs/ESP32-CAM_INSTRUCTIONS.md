# ESP32-CAM — Vision Module

## Overview

The ESP32-CAM provides vision capabilities for the STASIS rover:

- Fire detection (RGB colour analysis)
- Motion detection (frame differencing)
- Human detection (skin-tone heuristic)
- MJPEG streaming for AprilTag processing on the Pi

All hazard output is **plain text only** — no binary structs are written to Serial.

---

## Hardware

- **Board:** AI Thinker ESP32-CAM
- **Camera:** OV2640, QVGA (320×240), RGB565
- **Serial:** UART0 (`U0T`/`U0R`) — no native USB, requires FTDI programmer for flashing
- **Status LED:** GPIO 4 (active low)

---

## Flashing

### Wiring (FTDI programmer required)

| ESP32-CAM | FTDI Programmer |
|-----------|-----------------|
| U0R (RX) | TX |
| U0T (TX) | RX |
| GND | GND |
| 5V | 5V |
| **IO0** | **GND** ← Flash mode only |

### Steps

1. Connect IO0 → GND
2. Open `cam_firmware/cam_firmware.ino` in Arduino IDE
3. Board: **AI Thinker ESP32-CAM** | Upload speed: **115200**
4. Click Upload
5. Wait for "Done uploading"
6. **Disconnect IO0 from GND**
7. Press RST or power-cycle

### Verify

Open Serial Monitor at 115200 baud. You should see: `CAM:READY`

If you see `CAM:ERROR`, reseat the camera ribbon cable (contacts face down).

---

## Pin Reference

| ESP32-CAM | ESP32-S3 | Function |
|-----------|----------|----------|
| U0T (TX) | Serial RX (cameraTask) | Hazard text messages |
| U0R (RX) | Serial TX | Command bytes |
| 5V | 5V rail | Power |
| GND | GND | Ground |

---

## Commands (from ESP32-S3)

Single-byte commands:

| Byte | Name | Effect |
|------|------|--------|
| `0x01` | CAPTURE | Capture one frame, run detection, return result |
| `0x02` | STREAM_ON | Start continuous MJPEG stream |
| `0x03` | STREAM_OFF | Stop streaming |
| `0x04` | DETECT | Run detection only (no stream) |
| `0x05` | APRIL_STREAM | Switch to JPEG mode and start streaming (AprilTag) |

---

## Output Messages (to ESP32-S3)

All output is plain text, one message per line:

| Message | Meaning |
|---------|---------|
| `CAM:READY` | Camera initialised |
| `CAM:ERROR` | Camera failed to initialise |
| `HAZARD:FIRE` | Fire pixels detected |
| `HAZARD:MOTION` | Motion detected |
| `HAZARD:HUMAN` | Human detected |
| `CLEAR` | No hazards in current frame |

> **v2 change:** Raw binary `DetectionResult` struct writes were removed from `captureAndSend()` and `detectOnly()`. The binary bytes were corrupting the S3's text-line parser. Text-only output is unambiguous and sufficient.

---

## Detection Algorithm

The camera processes every Nth pixel (sample step = 50) in RGB565 format.

**Fire detection** — looks for high red, moderate green, low blue:
```
R > 25  AND  G > 10  AND  B < 8
→ fire pixel confirmed
→ trigger if firePixels > 15
```

**Motion detection** — compares against downsampled previous frame:
```
|current_pixel - previous_pixel| > 4000
→ motion pixel confirmed
→ trigger if motionPixels > 30
```

**Human detection** — skin tone heuristic in RGB565 5/6/5 bit space:
```
R > 15  AND  8 < G < 20  AND  5 < B < 15
→ human pixel confirmed
→ trigger if humanPixels > 30  (higher threshold than fire)
```

---

## Camera Settings

| Setting | Value | Reason |
|---------|-------|--------|
| Frame size | QVGA (320×240) | Fast processing on ESP32-CAM |
| Pixel format | RGB565 | Efficient for colour analysis |
| Frame buffers | 2 | Double-buffered for streaming |
| XCLK | 20 MHz | Standard speed |

---

## Streaming for AprilTag

When `CMD_APRIL_STREAM (0x05)` is sent, the camera switches to JPEG format and starts streaming. The Pi Zero reads this MJPEG stream at `CAM_STREAM_URL` for AprilTag detection.

The default stream endpoint served by the ESP32-CAM: `http://192.168.4.1:81/stream`

---

## LED Behaviour

| Pattern | Meaning |
|---------|---------|
| 3 quick blinks at boot | Starting up |
| Solid 500 ms | Camera ready |
| Continuous fast blink | Camera error |
| Brief flash on capture | Capture in progress |

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| `CAM:ERROR` | Reseat ribbon cable; contacts must face down toward board |
| Upload fails | Ensure IO0 is grounded; try different baud rate |
| Brownout during flash | Use a 100 µF capacitor between 5V and GND |
| Poor detection | Improve lighting; adjust detection thresholds in firmware |
| MJPEG stream not opening | Check AP connection; verify `http://192.168.4.1:81/stream` |

---

*Last updated: March 2026 — v2*
