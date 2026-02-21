# Complete Wiring Diagrams - Stasis Rover System

This document provides comprehensive wiring diagrams for all components of the Stasis system.

---

## Table of Contents

1. [Rover Unit (ESP32-S3)](#1-rover-unit-esp32-s3)
2. [Vision Module (ESP32-CAM)](#2-vision-module-esp32-cam)
3. [Base Station (ESP32-C3 + RPi Zero)](#3-base-station-esp32-c3--rpi-zero)
4. [Power Distribution](#4-power-distribution)
5. [Important Safety Notes](#5-important-safety-notes)

---

## 1. Rover Unit (ESP32-S3)

### Complete Pin Map

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

## 2. Vision Module (ESP32-CAM)

### ESP32-CAM to ESP32-S3 Connection

```
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

## 3. Base Station (ESP32-C3 + RPi Zero)

### Architecture Overview

The ESP32-C3 serves as a **GPIO Extender and Display Controller** for the Raspberry Pi Zero:
- **Communication Bridge**: ESP-NOW (wireless) ↔ UART (wired to RPi0W)
- **Display Controller**: LCD display shows rover status and location
- **Data Router**: Routes AprilTag data from rover → RPi0W for processing → back to rover for alignment
- **Power Management**: Powered from RPi0W 5V pins (bottom row only)

### ESP32-C3 to Raspberry Pi Zero (Bottom Row Only)

**IMPORTANT**: The C3 connects ONLY to the bottom row of the RPi0W GPIO header (the row starting with double 5V - pins 2, 4, 6, 8, 10...). This leaves the top row free for other peripherals.

```
    ESP32-C3                    Raspberry Pi Zero (Bottom Row Only)
    ┌─────────────┐             ┌─────────────────────┐
    │             │             │                     │
    │  GPIO 21    │────────────►│ Pin 10 (GPIO 15 RX) │  UART TX → RPi RX
    │  (TX)       │             │                     │
    │             │             │                     │
    │  GPIO 20    │◄────────────│ Pin 8 (GPIO 14 TX)  │  UART RX ← RPi TX
    │  (RX)       │             │                     │
    │             │             │                     │
    │  5V         │◄────────────│ Pin 2 (5V)          │  Power from RPi
    │             │             │                     │
    │  GND        │────────────►│ Pin 6 (GND)         │  Common Ground
    │             │             │                     │
    └─────────────┘             └─────────────────────┘
```

### Raspberry Pi Zero GPIO Header (Pin Usage)

```
    Raspberry Pi Zero GPIO Header (40-pin)
    ┌────────────────────────────────────────┐
    │  3.3V  [01] [02] 5V  ─────► C3 Power   │  BOTTOM ROW (Even pins)
    │  SDA   [03] [04] 5V  ─────► C3 Alt PWR │  ├── Used by ESP32-C3
    │  SCL   [05] [06] GND ─────► C3 GND     │  │   Pin 2: 5V Power
    │  GPIO4 [07] [08] TXD ─────► C3 RX      │  │   Pin 4: 5V (alternate)
    │  GND   [09] [10] RXD ◄───── C3 TX      │  │   Pin 6: GND
    │  GPIO17[11] [12] GPIO18                │  │   Pin 8: TXD → C3 RX
    │  GPIO27[13] [14] GND                   │  │   Pin 10: RXD ← C3 TX
    │  GPIO22[15] [16] GPIO23                │  │
    │  3.3V  [17] [18] GPIO24                │  TOP ROW (Odd pins)
    │  MOSI  [19] [20] GND                   │  └── Free for other uses
    │  MISO  [21] [22] GPIO25                │
    │  SCLK  [23] [24] CE0                   │
    │  GND   [25] [26] CE1                   │
    │        [27] [28]                       │
    │        [29] [30] GND                   │
    │        [31] [32]                       │
    │        [33] [34] GND                   │
    │        [35] [36]                       │
    │        [37] [38]                       │
    │  GND   [39] [40]                       │
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

## Quick Reference Card

### Rover (ESP32-S3) Pin Summary

| GPIO | Function | Connection |
|------|----------|------------|
| 1 | ADC | Battery monitor |
| 2 | I2C SCL | MPU6050 |
| 4 | PWM | Motor Left Speed |
| 5 | Digital | Motor Left Direction |
| 6 | PWM | Motor Right Speed |
| 7 | Digital | Motor Right Direction |
| 8 | Serial RX | GPS TX |
| 12 | Digital | Ultrasonic Trig |
| 13 | Digital | Ultrasonic Echo |
| 15 | Serial TX | SIM800 RX |
| 16 | Serial RX | SIM800 TX |
| 17 | Serial RX | CAM TX |
| 18 | Serial TX | CAM RX |
| 19 | Serial TX | GPS RX |
| 21 | OneWire | DS18B20 |
| 42 | I2C SDA | MPU6050 |
| 48 | Digital | Buzzer |

### Base Station Pin Summary

| ESP32-C3 GPIO | Function | Connection |
|---------------|----------|------------|
| 4 | I2C SDA | LCD Display |
| 5 | I2C SCL | LCD Display |
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

*Document Version: 2.0*  
*Last Updated: February 2026*  
*Updated for C3 as GPIO Extender with LCD Display Control*
