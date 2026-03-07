# STASIS Autonomous Rover — Team EDGECASE

STASIS is a compact autonomous hazard-monitoring rover designed for reliable real-world operation under tight budget constraints.

This project focuses on robustness, deterministic behavior, and competition-ready autonomy rather than experimental features.

---

## Core Capabilities

* Autonomous patrol
* Obstacle avoidance (HC-SR04)
* Environmental sensing (DS18B20, LDR)
* GPS tracking with virtual geofence (NEO-6M)
* GSM hazard alerts (SIM800)
* AprilTag-assisted precision docking
* Passive contact-pad charging
* Multi-MCU distributed architecture
* Raspberry Pi mission logging
* Live telemetry dashboard

---

## System Architecture

### Rover (ESP32-S3)

Primary responsibilities:

* state machine control
* motor control (L9110S)
* sensor fusion
* dock detection
* return-to-base logic
* GSM alert triggering

### Vision Node (ESP32-CAM)

* AprilTag detection
* snapshot capture
* hazard flag signaling

### Base Bridge (ESP32-C3)

* wireless/serial relay
* packet forwarding
* base communications

### Supervisor (Raspberry Pi Zero W)

* mission logging
* evidence storage
* dashboard hosting
* system monitoring

---

## Charging System

STASIS uses passive contact-pad charging:

Dock side:

* regulated 5V supply
* exposed screw-head contacts
* mechanical alignment funnel

Rover side:

* spring metal wipers
* onboard Li-ion charger

Dock detection is performed via battery voltage rise with hysteresis filtering.

---

## Rover State Machine

Primary states:

* PATROL
* ALERT
* RETURN_TO_BASE
* DOCKED_CHARGING
* SAFE_IDLE

All transitions are non-blocking and millis()-driven.

---

## Repository Structure

* rover_s3_firmware/ — main rover firmware
* cam_firmware/ — ESP32-CAM code
* base_bridge/ — ESP32-C3 bridge
* base_station/ — Raspberry Pi services
* stasis_app/ — web dashboard
* docking/ — AprilTag utilities
* docs/ — system documentation

---

## Design Philosophy

STASIS prioritizes:

* deterministic behavior
* electrical tolerance
* mechanical forgiveness
* low-cost components
* competition reliability

This is a field-robust student platform, not a lab-only prototype.

---

## Status

Active development.
Focused on competition reliability and system hardening.
