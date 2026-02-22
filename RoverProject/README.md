# STASIS

STASIS is an autonomous environmental monitoring rover designed to patrol an area, detect hazards, and return to a charging station without human intervention.

The project focuses on building a **reliable real-world system** using affordable hardware rather than a simulation-only demo. The rover combines onboard sensing, wireless communication, and vision-assisted docking to operate for extended periods with minimal supervision.

---

## What STASIS Does

* Patrols an area while avoiding obstacles
* Monitors environmental conditions
* Detects potential fire events using sensor fusion
* Sends alerts through GSM (SIM800)
* Returns to base automatically when battery is low
* Uses AprilTag visual alignment for final docking
* Syncs collected data to a base station

The goal is not just movement, but **autonomous decision-making under constraints**.

---

## System Overview

STASIS is split into two main parts: the rover and the base station.

**Rover (ESP32-S3 based)**
Handles navigation, sensor reading, state machine logic, and motor control.

**ESP32-CAM**
Runs lightweight vision tasks and provides detection flags.

**Base Station (Pi Zero + ESP32-C3)**
Processes AprilTag detection, receives rover data, and generates reports.

This separation keeps real-time control on the rover while heavier processing stays at the base.

---

## Rover State Machine

The rover operates using explicit states:

* PATROL
* RESEARCH
* ALERT
* RETURN_TO_BASE
* DOCKING
* DOCKED

Transitions are watchdog-protected to prevent lockups in the field.

---

## Fire Detection Approach

To reduce false alarms, STASIS does **not** rely on a single sensor.

A fire event is only triggered when:

* Temperature crosses threshold (DS18B20)
  **and**
* ESP32-CAM reports a fire-like visual signature

This was added after early testing showed single-sensor triggers were unreliable.

---

## Power Strategy

When battery drops below 30%:

* motor speed is reduced
* sensor polling is lowered
* rover switches to Return-to-Base mode

The system is designed to fail safe rather than run until brownout.

---

## AprilTag Docking

Docking uses a 200 mm tag36h11 marker at the charging station.

Behavior is two-stage:

1. Long-range acquisition (up to ~4 m)
2. Slow precision alignment near the dock

If the tag is lost, the rover enters a controlled search instead of continuing blindly.

---

## Repository Layout

```
/rover_s3_firmware
/esp32_cam_firmware
/base_station_c3
/pi_zero_station
/cad
/docs
```

---

## Hardware Used

**Rover**

* ESP32-S3
* ESP32-CAM
* L9110S motor driver (4WD)
* Neo-6M GPS
* MPU6050
* DS18B20
* Ultrasonic sensor
* SIM800
* LiPo battery

**Base Station**

* Raspberry Pi Zero
* ESP32-C3
* I2C LCD
* AprilTag marker

---

## Current Status

The core architecture is working. Docking and field reliability are under active tuning.

---

## Known Limitations

* Docking accuracy depends heavily on camera calibration
* ESP32-CAM performance varies with lighting
* GPS return is coarse in dense environments
* Mechanical design is in early revision

---

## Why This Project Exists

This started as a simple rover idea and gradually evolved into a full multi-node system. The focus throughout has been on building something that behaves predictably on real hardware, not just in ideal conditions.

---

## License

MIT
