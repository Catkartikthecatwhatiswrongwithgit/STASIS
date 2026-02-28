# Complete Wiring Diagrams - Stasis Rover System

This document provides comprehensive wiring diagrams for all components of the Stasis system.

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Rover Unit (ESP32-S3)](#2-rover-unit-esp32-s3)
3. [Vision Module (ESP32-CAM)](#3-vision-module-esp32-cam)
4. [Base Station (ESP32-C3 + RPi Zero)](#4-base-station-esp32-c3--rpi-zero)
5. [Power Distribution](#5-power-distribution)
6. [Complete Pin Reference](#6-complete-pin-reference)

---

## 1. System Overview

### Communication Flow

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           ROVER UNIT                                        │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐                   │
│  │ ESP32-S3    │◄──►│ ESP32-CAM   │    │   Motors    │                   │
│  │ (Main Ctrl) │    │  (Vision)   │    │  L9110S     │                   │
│  │             │    │             │    │             │                   │
│  │ UART1: Pi   │    │ Serial: S3  │    │ GPIO 4-7    │                   │
│  │ GPIO 43,44  │    │             │    │             │                   │
│  └──────┬──────┘    └──────┬──────┘    └─────────────┘                   │
│         │                  │                                                   │
│         │ ESP-NOW          │                                                   │
└─────────┼──────────────────┼───────────────────────────────────────────────────┘
          │                  │
          ▼                  ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                          BASE STATION                                       │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐                    │
│  │ ESP32-C3   │◄──►│ Raspberry   │    │   LCD       │                    │
│  │  (Bridge)  │    │   Pi Zero   │    │  16x2 I2C   │                    │
│  │             │    │             │    │             │                    │
│  │ UART: Pi   │    │ Flask API   │    │ I2C         │                    │
│  │ GPIO 20,21 │    │ Port 5000   │    │             │                    │
│  └─────────────┘    └──────┬──────┘    └─────────────┘                    │
└─────────────────────────────┼───────────────────────────────────────────────┘
                              │
                              │ HTTP
                              ▼
                    ┌─────────────────┐
                    │  Web Dashboard │
                    │ stasis_app/    │
                    └─────────────────┘
```

### Connection Summary

| Link | Protocol | Pins |
|------|----------|------|
| Rover ↔ Camera | Serial (UART0) | RX/GPIO1, TX/GPIO3 |
| Rover ↔ Pi | UART1 | GPIO 43 (TX), GPIO 44 (RX) |
| Rover ↔ Base | ESP-NOW | Wireless |
| Base ↔ Pi | UART | GPIO 20 (TX), GPIO 21 (RX) |

---

## 2. Rover Unit (ESP32-S3)

### Complete Pin Map

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           ESP32-S3 DEVKIT                                  │
│                                                                             │
│  ┌─────┐                                                                     │
│  │ USB │                                                                     │
│  └──┬──┘                                                                     │
│     │                                                                        │
│  ┌──┴────────────────────────────────────────────────────────────────────┐  │
│  │                                                                       │  │
│  │  GPIO 1  ◄──── Battery Monitor (via voltage divider)                 │  │
│  │  GPIO 2  ◄──── MPU6050 SCL (I2C Clock)                               │  │
│  │  GPIO 3  ◄──── ESP32-CAM TX (Serial)                                │  │
│  │  GPIO 4  ────► L9110S B-IA (Left Motor PWM)                          │  │
│  │  GPIO 5  ────► L9110S B-IB (Left Motor Direction)                    │  │
│  │  GPIO 6  ────► L9110S A-IA (Right Motor PWM)                         │  │
│  │  GPIO 7  ────► L9110S A-IB (Right Motor Direction)                  │  │
│  │  GPIO 8  ◄──── GPS TX (SoftwareSerial RX)                            │  │
│  │  GPIO 9  ◄──── Ultrasonic Echo                                        │  │
│  │  GPIO 10 ────► Ultrasonic Trig                                        │  │
│  │  GPIO 12 ────► SIM800 RXD (optional)                                 │  │
│  │  GPIO 13 ◄──── SIM800 TXD (optional)                                 │  │
│  │  GPIO 21 ────► DS18B20 Data (with 4.7k pull-up)                     │  │
│  │  GPIO 42 ◄──── MPU6050 SDA (I2C Data)                               │  │
│  │  GPIO 43 ────► Pi RX (UART1 TX)                                      │  │
│  │  GPIO 44 ◄──── Pi TX (UART1 RX)                                      │  │
│  │  GPIO 45 ────► Status LED                                             │  │
│  │  GPIO 48 ────► Buzzer (+)                                             │  │
│  │                                                                       │  │
│  │  3.3V   ────► MPU6050 VCC, GPS VCC (if 3.3V compatible)            │  │
│  │  5V     ◄──── Buck Converter Output                                   │  │
│  │  GND    ────► Common Ground (ALL components)                         │  │
│  │                                                                       │  │
│  └───────────────────────────────────────────────────────────────────────┘  │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### UART Connections

```
ESP32-S3                        Raspberry Pi (via ESP32-C3)
┌─────────────┐                ┌─────────────┐
│             │                │             │
│ GPIO 43 (TX)│────────────────│ GPIO 20     │  ESP32-C3 RX
│             │                │             │
│ GPIO 44 (RX)│◄──────────────│ GPIO 21     │  ESP32-C3 TX
│             │                │             │
└─────────────┘                └─────────────┘

Note: Connection is through ESP32-C3 bridge, not direct to Pi
```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           ESP32-S3 DEVKIT                                    │
│                                                                             │
│  ┌─────┐                                                                     │
│  │ USB │                                                                     │
│  └──┬──┘                                                                     │
│     │                                                                        │
│  ┌──┴────────────────────────────────────────────────────────────────────┐  │
│  │                                                                       │  │
│  │  GPIO 1  ◄──── Battery Monitor (via voltage divider)                 │  │
│  │  GPIO 2  ◄──── MPU6050 SCL (I2C Clock)                               │  │
│  │  GPIO 4  ────► L9110S B-IA (Left Motor PWM)                          │  │
│  │  GPIO 5  ────► L9110S B-IB (Left Motor Direction)                    │  │
│  │  GPIO 6  ────► L9110S A-IA (Right Motor PWM)                         │  │
│  │  GPIO 7  ────► L9110S A-IB (Right Motor Direction)                   │  │
│  │  GPIO 8  ◄──── GPS TX (SoftwareSerial RX)                            │  │
│  │  GPIO 12 ────► Ultrasonic Trig                                        │  │
│  │  GPIO 13 ◄──── Ultrasonic Echo (via voltage divider)                 │  │
│  │  GPIO 15 ────► SIM800 RXD                                             │  │
│  │  GPIO 16 ◄──── SIM800 TXD                                             │  │
│  │  GPIO 17 ◄──── ESP32-CAM TX (UART1 RX)                               │  │
│  │  GPIO 18 ────► ESP32-CAM RX (UART1 TX)                               │  │
│  │  GPIO 19 ────► GPS RX (SoftwareSerial TX)                            │  │
│  │  GPIO 21 ────► DS18B20 Data (with 4.7k pull-up)                      │  │
│  │  GPIO 42 ◄──── MPU6050 SDA (I2C Data)                                │  │
│  │  GPIO 45 ────► Status LED                                             │  │
│  │  GPIO 48 ────► Buzzer (+)                                             │  │
│  │                                                                       │  │
│  │  3.3V   ────► MPU6050 VCC, GPS VCC (if 3.3V compatible)              │  │
│  │  5V     ◄──── Buck Converter Output                                   │  │
│  │  GND    ────► Common Ground (ALL components)                          │  │
│  │                                                                       │  │
│  └───────────────────────────────────────────────────────────────────────┘  │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Motor Driver (L9110S) Connections

```
                    L9110S Motor Driver
                   ┌─────────────────────┐
                   │                     │
    GPIO 4 ───────►│ B-IA   ││   A-IA ◄─────── GPIO 6
    GPIO 5 ───────►│ B-IB   ││   A-IB ◄─────── GPIO 7
                   │        ││                    │
    Left Motors ──►│ OUTB   ││   OUTA ◄─────────── Right Motors
                   │        ││                    │
    Buck 5V   ────►│ VCC    ││   GND ◄─────────── Common GND
                   │                     │
                   └─────────────────────┘

   LEFT SIDE (B Channel)          RIGHT SIDE (A Channel)
   ┌───────────────────┐          ┌───────────────────┐
   │   Front Left      │          │   Front Right     │
   │   Motor           │          │   Motor           │
   │   ───┬───         │          │   ───┬───         │
   │      │            │          │      │            │
   │   Rear Left       │          │   Rear Right      │
   │   Motor           │          │   Motor           │
   └───────────────────┘          └───────────────────┘
         ││                              ││
    Connect both to              Connect both to
    B screw terminals            A screw terminals
```

### Sensor Connections

#### MPU6050 (IMU)

```
    MPU6050                ESP32-S3
    ┌─────────┐           ┌─────────┐
    │  VCC    │──────────►│  3.3V   │
    │  GND    │──────────►│  GND    │
    │  SCL    │──────────►│ GPIO 2  │
    │  SDA    │──────────►│ GPIO 42 │
    │  XDA    │           │         │
    │  XCL    │           │         │
    │  AD0    │──────────►│  GND    │ (Address: 0x68)
    │  INT    │           │         │
    └─────────┘           └─────────┘
```

#### DS18B20 (Temperature)

```
                    4.7kΩ Resistor
    DS18B20            ┌───┐
    ┌─────────┐        │   │
    │  VCC    │────────┴───┴──────► 3.3V
    │  DATA   │────────────────────► GPIO 21
    │  GND    │────────────────────► GND
    └─────────┘
    
    Note: The 4.7kΩ pull-up resistor connects DATA to VCC
```

#### Ultrasonic Sensor (HC-SR04)

```
    HC-SR04              ESP32-S3
    ┌─────────┐         ┌─────────┐
    │  VCC    │────────►│  5V     │
    │  Trig   │────────►│ GPIO 12 │
    │  Echo   │──┬─────►│ GPIO 13 │
    │  GND    │  │      │         │
    └─────────┘  │      └─────────┘
                 │
            Voltage Divider (5V → 3.3V)
            ┌─────────────────────┐
            │                     │
            │   1kΩ      2kΩ     │
            │  ┌───┐    ┌───┐    │
    Echo ───┴──┤   ├──┬─┤   ├──┴──► GND
                  └───┘  │  └───┘
                         │
                         └──────────────► GPIO 13
```

#### GPS Module (Neo-6M)

```
    Neo-6M               ESP32-S3
    ┌─────────┐         ┌─────────┐
    │  VCC    │────────►│  3.3V   │
    │  GND    │────────►│  GND    │
    │  TX     │────────►│ GPIO 8  │ (SoftwareSerial RX)
    │  RX     │◄────────│ GPIO 19 │ (SoftwareSerial TX)
    └─────────┘         └─────────┘
```

#### SIM800L (GSM Module)

```
    SIM800L              ESP32-S3
    ┌─────────┐         ┌─────────┐
    │  VCC    │◄────────│ 4.0V Supply (NOT from ESP32!)
    │  GND    │────────►│  GND    │
    │  TXD    │────────►│ GPIO 16 │ (UART2 RX)
    │  RXD    │◄────────│ GPIO 15 │ (UART2 TX)
    │  RST    │         │         │
    └─────────┘         └─────────┘
    
    ⚠️ WARNING: SIM800L requires 3.4V-4.4V @ 2A peak!
    Use dedicated power supply or LiPo battery directly.
```

#### Battery Monitor

```
    LiPo Battery (12.6V max)
         │
         ├──────────────────────────────────────┐
         │                                      │
         │   Voltage Divider                    │
         │   ┌─────────────────┐                │
         │   │                 │                │
         │   │   30kΩ    10kΩ  │                │
         ├───┤  ┌───┐   ┌───┐  │                │
         │   │  │   │   │   │  │                │
         │   │  └───┴───┴───┴──┼────────────────┤
         │   │         │       │                │
         │   │         └───────┼────► GPIO 1    │
         │   │                 │    (ADC)       │
         │   └─────────────────┼────────────────┤
         │                     │                │
         └─────────────────────┴────► GND       │
                                                │
    Formula: ADC_Voltage = Battery_V × 10/(30+10)
             ADC_Voltage = Battery_V × 0.25
             Battery_V = ADC_Voltage × 4
```

---

## 3. Vision Module (ESP32-CAM)

### ESP32-CAM to ESP32-S3 Connection

The ESP32-CAM uses **Serial (UART0)** to communicate with the ESP32-S3 rover:

```
    ESP32-CAM                    ESP32-S3
    ┌─────────────┐             ┌─────────────┐
    │             │             │             │
    │  U0T (TX)   │────────────►│ GPIO 1 (RX) │  Serial
    │  U0R (RX)   │◄────────────│ GPIO 3 (TX) │  Serial
    │             │             │             │
    │  5V         │◄────────────│ 5V          │
    │  GND        │────────────►│ GND         │
    │             │             │             │
    └─────────────┘             └─────────────┘
    
    Note: Uses Serial (UART0), not UART1
```

### Communication Protocol

| From Rover | From Camera | Description |
|------------|-------------|-------------|
| Commands: 0x01-0x04 | `HAZARD:FIRE` | Fire detected |
| | `HAZARD:MOTION` | Motion detected |
| | `HAZARD:HUMAN` | Human detected |
| | `CAM:READY` | Camera initialized |
| | `CAM:ERROR` | Camera error |
| | `CLEAR` | No hazards |
    ESP32-CAM                    ESP32-S3
    ┌─────────────┐             ┌─────────────┐
    │             │             │             │
    │  U0T (TX)   │────────────►│ GPIO 17 (RX)│
    │  U0R (RX)   │◄────────────│ GPIO 18 (TX)│
    │             │             │             │
    │  5V         │◄────────────│ 5V          │
    │  GND        │────────────►│ GND         │
    │             │             │             │
    └─────────────┘             └─────────────┘
    
    Note: Both boards share the same 5V supply from Buck Converter
```

### ESP32-CAM Camera Connection

```
    ESP32-CAM Board
    ┌─────────────────────────────────┐
    │                                 │
    │   ┌─────────────────────────┐   │
    │   │   Camera Connector      │   │
    │   │   ┌─────────────────┐   │   │
    │   │   │ ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ │   │   │
    │   │   │  Ribbon Cable   │   │   │
    │   │   │  (contacts down)│   │   │
    │   │   └─────────────────┘   │   │
    │   └─────────────────────────┘   │
    │                                 │
    │   OV2640 Camera Module          │
    │   ┌─────────────────────┐       │
    │   │  ┌───────────────┐  │       │
    │   │  │   Lens        │  │       │
    │   │  └───────────────┘  │       │
    │   └─────────────────────┘       │
    │                                 │
    └─────────────────────────────────┘
```

---

## 4. Base Station (ESP32-C3 + RPi Zero)

### ESP32-C3 to Raspberry Pi Zero

```
    ESP32-C3                    Raspberry Pi Zero
    ┌─────────────┐             ┌─────────────────────┐
    │             │             │                     │
    │  GPIO 21    │────────────►│ Pin 10 (GPIO 15 RX)│  UART TX → RPi RX
    │  (TX)       │             │                     │
    │             │             │                     │
    │  GPIO 20    │◄────────────│ Pin 8 (GPIO 14 TX) │  UART RX ← RPi TX
    │  (RX)       │             │                     │
    │             │             │                     │
    │  5V         │◄────────────│ Pin 2 (5V)         │  Power from RPi
    │             │             │                     │
    │  GND        │────────────►│ Pin 6 (GND)        │  Common Ground
    │             │             │                     │
    └─────────────┘             └─────────────────────┘
```

### Raspberry Pi Zero GPIO Header

```
    40-pin GPIO Header
    ┌────────────────────────────────────────┐
    │ 3.3V  [01] [02] 5V  ───► ESP32-C3    │  BOTTOM ROW (Even pins)
    │ SDA   [03] [04] 5V                    │  
    │ SCL   [05] [06] GND ───► ESP32-C3    │  
    │ GPIO4 [07] [08] TXD ───► ESP32-C3 RX  │  
    │ GND   [09] [10] RXD ◄─── ESP32-C3 TX  │  
    │ GPIO17[11] [12] GPIO18                │  TOP ROW - Available
    │ GPIO27[13] [14] GND                   │  for LCD, sensors
    │ GPIO22[15] [16] GPIO23                │  
    │ 3.3V  [17] [18] GPIO24                │  
    │ MOSI  [19] [20] GND                   │  
    │ MISO  [21] [22] GPIO25                │  
    │ SCLK  [23] [24] CE0                   │  
    │ GND   [25] [26] CE1                   │  
    └────────────────────────────────────────┘
```
    └────────────────────────────────────────┘
```

### LCD Display Connection (Controlled by ESP32-C3)

The LCD display is connected to the ESP32-C3, NOT the RPi Zero directly. The C3 receives display commands from RPi0W via UART and controls the LCD.

```
    LCD 16x2 I2C            ESP32-C3
    ┌─────────────┐         ┌─────────────────┐
    │  VCC        │────────►│ 5V (from RPi)   │
    │  GND        │────────►│ GND             │
    │  SDA        │────────►│ GPIO 4 (SDA)    │
    │  SCL        │────────►│ GPIO 5 (SCL)    │
    └─────────────┘         └─────────────────┘
    
    Note: LCD I2C Address typically 0x27 or 0x3F
```

### Complete Base Station Wiring Diagram

```
    ┌─────────────────────────────────────────────────────────────────────────┐
    │                         BASE STATION ARCHITECTURE                        │
    │                                                                          │
    │   ┌─────────────────┐         UART (Pins 8,10)        ┌───────────────┐ │
    │   │  USB Wall       │                                 │   ESP32-C3    │ │
    │   │  Charger (5V)   │    ┌───────────────────────┐    │               │ │
    │   └────────┬────────┘    │                       │    │  ┌─────────┐  │ │
    │            │             │   Raspberry Pi Zero   │    │  │ LCD     │  │ │
    │            ▼             │                       │    │  │ 16x2    │  │ │
    │   ┌─────────────────┐    │   ┌───────────────┐   │    │  │ I2C     │  │ │
    │   │  RPi Zero       │◄───┼───┤ UART RX (P10) │◄──┼────┤  │ Display │  │ │
    │   │  (Main Brain)   │    │   │ UART TX (P8)  │───┼───►│  └─────────┘  │ │
    │   │                 │    │   └───────────────┘   │    │               │ │
    │   │  - Processes    │    │                       │    │  GPIO 4 ─────┼─┼─► LCD SDA
    │   │    AprilTag     │    │   5V ─────────────────┼───►│  GPIO 5 ─────┼─┼─► LCD SCL
    │   │  - Generates    │    │   GND ────────────────┼───►│               │ │
    │   │    reports      │    │                       │    │  GPIO 21 ─────┼─┼─► RPi RX
    │   │  - Controls     │    └───────────────────────┘    │  GPIO 20 ─────┼─┼─► RPi TX
    │   │    display      │                                 │               │ │
    │   └─────────────────┘                                 │  ESP-NOW      │ │
    │                                                        │  (Wireless)   │ │
    │                                                        └───────┬───────┘ │
    │                                                                │         │
    │                                                                │         │
    │                                                        ESP-NOW │         │
    │                                                        (2.4GHz)│         │
    │                                                                ▼         │
    │                                                        ┌───────────────┐ │
    │                                                        │  Rover (S3)   │ │
    │                                                        │  + ESP32-CAM  │ │
    │                                                        └───────────────┘ │
    │                                                                          │
    └─────────────────────────────────────────────────────────────────────────┘
```

### Charging Docking Data Flow

```
    CHARGING STATION APPROACH SEQUENCE:
    
    1. ROVER APPROACH (GPS Navigation):
       ┌──────────┐    GPS Distance     ┌──────────┐
       │ Rover S3 │ ──────────────────► │ Base C3  │
       │          │    (ESP-NOW)        │          │
       └──────────┘                     └──────────┘
    
    2. APRILTAG DETECTION (Precise Alignment):
       ┌──────────┐   AprilTag Data    ┌──────────┐   AprilTag Data    ┌──────────┐
       │ ESP32-CAM│ ──────────────────►│ Rover S3 │ ──────────────────►│ Base C3  │
       │ (on S3)  │    (UART)          │          │    (ESP-NOW)       │          │
       └──────────┘                    └──────────┘                    └──────────┘
                                                                            │
                                                                            ▼
       ┌──────────┐  Alignment Cmds    ┌──────────┐  Processed Data    ┌──────────┐
       │ Rover S3 │ ◄───────────────── │ Base C3  │ ◄───────────────── │ RPi Zero │
       │          │    (ESP-NOW)       │          │    (UART)          │          │
       └──────────┘                    └──────────┘                    └──────────┘
    
    3. DOCKING & CHARGING:
       ┌──────────┐   Charging Status  ┌──────────┐   Display Cmds     ┌──────────┐
       │ Rover S3 │ ──────────────────►│ Base C3  │ ◄───────────────── │ RPi Zero │
       │          │    (ESP-NOW)       │          │    (UART)          │          │
       └──────────┘                    └──────────┘                    └──────────┘
                                             │
                                             ▼
                                       ┌──────────┐
                                       │ LCD 16x2 │
                                       │ Display  │
                                       └──────────┘
    
    4. REPORT GENERATION:
       ┌──────────┐   All Log Data     ┌──────────┐   Raw Data         ┌──────────┐
       │ Rover S3 │ ──────────────────►│ Base C3  │ ──────────────────►│ RPi Zero │
       │          │    (ESP-NOW)       │          │    (UART)          │          │
       └──────────┘                    └──────────┘                    └──────────┘
                                                                             │
                                                                             ▼
                                                                       ┌──────────┐
                                                                       │ Daily    │
                                                                       │ Report   │
                                                                       │ (PDF)    │
                                                                       └──────────┘
```

### ESP32-C3 Pin Map (Complete)

```
    ESP32-C3 DEVKIT
    ┌─────────────────────────────────────────────────────────────────────────┐
    │                                                                         │
    │  GPIO 4  ◄────► LCD SDA (I2C Data)                                     │
    │  GPIO 5  ◄────► LCD SCL (I2C Clock)                                    │
    │  GPIO 8  ────► Built-in LED (Status)                                   │
    │  GPIO 20 ◄────► RPi Zero TXD (Pin 8) - UART RX                         │
    │  GPIO 21 ────► RPi Zero RXD (Pin 10) - UART TX                         │
    │                                                                         │
    │  5V     ◄──── RPi Zero 5V (Pin 2)                                      │
    │  GND    ────► Common Ground                                            │
    │                                                                         │
    │  ESP-NOW Antenna (Built-in) ──── Wireless to Rover S3                  │
    │                                                                         │
    └─────────────────────────────────────────────────────────────────────────┘
```

---

## 4. Power Distribution

### Rover Power System

```
    LiPo Battery (3S, 11.1V - 12.6V)
    ┌─────────────────────────────────────────────────────────────┐
    │                                                             │
    │   ┌─────────────────────────────────────────────────────┐   │
    │   │                 Buck Converter                       │   │
    │   │               (12V → 5V, 5A max)                     │   │
    │   │                                                      │   │
    │──►│  IN+     IN-     OUT+    OUT-                       │   │
    │   │  │       │       │       │                          │   │
    │   └──┼───────┼───────┼───────┼──────────────────────────┘   │
    │      │       │       │       │                              │
    │      │       │       │       │                              │
    │      │       │       │       └──────────────────► GND       │
    │      │       │       │                                      │
    │      │       │       └───────────────────────► 5V Rail      │
    │      │       │                                              │
    │      │       └──────────────────────────────────► GND       │
    │      │                                                      │
    │      └──────────────────────────────────────────────► Battery+
    │                                                             │
    └─────────────────────────────────────────────────────────────┘
    
    5V Rail Distribution:
    ┌─────────────────────────────────────────────────────────────┐
    │  5V ───┬───► ESP32-S3 (5V pin)                             │
    │        ├───► ESP32-CAM (5V pin)                             │
    │        ├───► L9110S Motor Driver (VCC)                      │
    │        ├───► Ultrasonic Sensor (VCC)                        │
    │        └───► SIM800L (via dedicated regulator)              │
    │                                                             │
    │  3.3V (from ESP32-S3) ───┬───► MPU6050                      │
    │                          ├───► DS18B20                      │
    │                          └───► GPS (if 3.3V compatible)     │
    └─────────────────────────────────────────────────────────────┘
```

### Base Station Power System

```
    USB Wall Charger (5V, 2.5A)
    ┌─────────────────────────────────────────────────────────────┐
    │                                                             │
    │   ┌─────────────────────────────────────────────────────┐   │
    │   │              Raspberry Pi Zero                        │   │
    │   │                                                      │   │
    │──►│  micro-USB PWR                                       │   │
    │   │                                                      │   │
    │   │  5V Pin (Pin 2) ─────────────► ESP32-C3 (5V)         │   │
    │   │  GND Pin (Pin 6) ◄──────────── ESP32-C3 (GND)        │   │
    │   │                                                      │   │
    │   │  5V Pin (Pin 2) ─────────────► LCD VCC               │   │
    │   │  GND Pin (Pin 6) ◄──────────── LCD GND               │   │
    │   │                                                      │   │
    │   └──────────────────────────────────────────────────────┘   │
    │                                                             │
    └─────────────────────────────────────────────────────────────┘
    
    Power Distribution from RPi Zero:
    ┌─────────────────────────────────────────────────────────────┐
    │  5V (Pin 2) ───┬───► ESP32-C3 (5V pin)                     │
    │                └───► LCD Display (VCC)                      │
    │                                                             │
    │  GND (Pin 6) ──┬───► ESP32-C3 (GND)                         │
    │                └───► LCD Display (GND)                      │
    └─────────────────────────────────────────────────────────────┘
```

---

## 5. Important Safety Notes

### Voltage Level Warnings

| Component | Logic Level | Warning |
|-----------|-------------|---------|
| ESP32-S3 | 3.3V | **Do NOT connect 5V signals directly!** |
| ESP32-C3 | 3.3V | **Do NOT connect 5V signals directly!** |
| ESP32-CAM | 3.3V | **Do NOT connect 5V signals directly!** |
| Ultrasonic Echo | 5V output | **Requires voltage divider!** |
| SIM800L | 3.4V-4.4V | **Do NOT power from ESP32!** |

### Required Voltage Dividers

```
    5V Signal to 3.3V GPIO:
    
    5V Signal ───┬────────────────────► 3.3V GPIO
                 │
               ┌─┴─┐
               │   │ 1kΩ
               └─┬─┘
                 │
                 ├────────────────────► GND
               ┌─┴─┐
               │   │ 2kΩ
               └─┬─┘
                 │
                GND
    
    Output: 5V × (2kΩ / (1kΩ + 2kΩ)) = 3.33V ✓
```

### Current Requirements

| Component | Typical Current | Peak Current |
|-----------|-----------------|--------------|
| ESP32-S3 | 150mA | 500mA |
| ESP32-CAM | 80mA | 300mA (flash LED) |
| ESP32-C3 | 100mA | 400mA |
| LCD 16x2 | 20mA | 50mA (with backlight) |
| SIM800L | 10mA (idle) | 2A (transmit) |
| Motors (4x) | 500mA | 2A (stall each) |
| GPS | 25mA | 50mA |
| MPU6050 | 3mA | 10mA |
| DS18B20 | 1mA | 5mA |
| Ultrasonic | 2mA | 15mA |

**Total Rover Peak: ~6A** (ensure buck converter can handle this)
**Total Base Station Peak: ~1A** (USB charger should be 2.5A minimum)

### Ground Rules

1. **Common Ground**: ALL components must share a common ground
2. **Star Ground**: Connect all grounds to a single point when possible
3. **No Ground Loops**: Avoid creating ground loops in wiring

---

## 6. Complete Pin Reference

### Rover (ESP32-S3) Pin Summary

| GPIO | Function | Connection | Notes |
|------|----------|------------|-------|
| 1 | Serial RX | ESP32-CAM TX | UART0 |
| 2 | I2C SCL | MPU6050 | |
| 3 | Serial TX | ESP32-CAM RX | UART0 |
| 4 | PWM | Motor Left Speed | L9110S B-IA |
| 5 | Digital | Motor Left Direction | L9110S B-IB |
| 6 | PWM | Motor Right Speed | L9110S A-IA |
| 7 | Digital | Motor Right Direction | L9110S A-IB |
| 8 | Serial RX | GPS TX | |
| 9 | Digital | Ultrasonic Echo | |
| 10 | Digital | Ultrasonic Trig | |
| 12 | Digital | SIM800 RX (optional) | |
| 13 | Digital | SIM800 TX (optional) | |
| 21 | OneWire | DS18B20 | 4.7k pull-up |
| 42 | I2C SDA | MPU6050 | |
| 43 | Serial TX | ESP32-C3 RX | UART1 |
| 44 | Serial RX | ESP32-C3 TX | UART1 |
| 45 | Digital | Status LED | |
| 48 | Digital | Buzzer | |

### Camera (ESP32-CAM) Pin Summary

| Pin | Function | Connection |
|-----|----------|------------|
| U0T | Serial TX | ESP32-S3 GPIO 1 |
| U0R | Serial RX | ESP32-S3 GPIO 3 |
| 5V | Power | Buck Converter 5V |
| GND | Ground | Common GND |

### Base Station Pin Summary

| ESP32-C3 GPIO | Function | Connection |
|---------------|----------|------------|
| 8 | LED | Built-in Status |
| 20 | UART RX | RPi Zero TXD (Pin 8) |
| 21 | UART TX | RPi Zero RXD (Pin 10) |

| RPi Zero Pin | Function | Connection |
|--------------|----------|------------|
| 2 | 5V Power | ESP32-C3 5V |
| 6 | GND | ESP32-C3 GND |
| 8 | TXD (GPIO 14) | ESP32-C3 RX (GPIO 20) |
| 10 | RXD (GPIO 15) | ESP32-C3 TX (GPIO 21) |

---

### Complete Data Flow

```
┌──────────────┐     Serial      ┌──────────────┐    UART1    ┌──────────────┐
│ ESP32-CAM   │◄──────────────►│  ESP32-S3   │◄───────────►│  ESP32-C3   │
│             │   (UART0)       │   (Rover)   │  (GPIO 43/44)│  (Bridge)   │
└──────────────┘                 └──────────────┘              └──────┬───────┘
      │                                                               │
      │ HAZARD: messages                                              │ UART
      │ forwarded                                                     │ (GPIO 20/21)
      │                                                               ▼
      │                                              ┌───────────────────────┐
      │                                              │    Raspberry Pi Zero  │
      │                                              │                       │
      │                                              │  station_monitor.py  │
      │                                              │  - Parse telemetry   │
      │                                              │  - Store SQLite DB  │
      │                                              │  - Serve API         │
      │                                              │                       │
      │                                              │  /api/status         │
      │                                              │  /api/command        │
      │                                              │  /api/alerts         │
      │                                              └───────────────────────┘
      │                                                        │
      │                                                        │ HTTP
      ▼                                                        ▼
┌─────────────────────────────────────────────────────────────────────┐
│                        Web Dashboard (stasis_app/index.html)        │
│                                                                     │
│  Dashboard │ Map │ Stream │ Chat │ Fencing                         │
└─────────────────────────────────────────────────────────────────────┘
```

---

*Document Version: 3.0*  
*Last Updated: February 2026*
