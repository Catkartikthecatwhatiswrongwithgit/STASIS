# Complete Wiring Diagrams - Aero Sentinel Rover System

This document provides comprehensive wiring diagrams for all components of the Aero Sentinel system.

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

### ESP32-C3 to Raspberry Pi Zero

```
    ESP32-C3                    Raspberry Pi Zero
    ┌─────────────┐             ┌─────────────────────┐
    │             │             │                     │
    │  GPIO 21    │────────────►│ Pin 10 (GPIO 15 RX) │
    │  (TX)       │             │                     │
    │             │             │                     │
    │  GPIO 20    │◄────────────│ Pin 8 (GPIO 14 TX)  │
    │  (RX)       │             │                     │
    │             │             │                     │
    │  5V/3.3V    │◄────────────│ Pin 2 or 4 (5V)     │
    │             │             │                     │
    │  GND        │────────────►│ Pin 6 (GND)         │
    │             │             │                     │
    └─────────────┘             └─────────────────────┘
```

### Raspberry Pi Zero GPIO Header (Complete)

```
    Raspberry Pi Zero GPIO Header (40-pin)
    ┌────────────────────────────────────────┐
    │  3.3V  [01] [02] 5V                    │
    │  SDA   [03] [04] 5V                    │
    │  SCL   [05] [06] GND                   │
    │  GPIO4 [07] [08] TXD  ─────► ESP32-C3 RX
    │  GND   [09] [10] RXD  ◄───── ESP32-C3 TX
    │  GPIO17[11] [12] GPIO18                │
    │  GPIO27[13] [14] GND                   │
    │  GPIO22[15] [16] GPIO23                │
    │  3.3V  [17] [18] GPIO24                │
    │  MOSI  [19] [20] GND                   │
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

### LCD Display Connection (Optional)

```
    LCD 16x2 I2C            Raspberry Pi Zero
    ┌─────────────┐         ┌─────────────────┐
    │  VCC        │────────►│ Pin 2 (5V)      │
    │  GND        │────────►│ Pin 6 (GND)     │
    │  SDA        │────────►│ Pin 3 (GPIO 2)  │
    │  SCL        │────────►│ Pin 5 (GPIO 3)  │
    └─────────────┘         └─────────────────┘
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
    │   │  5V Pin ────────────────────────► ESP32-C3 (5V)      │   │
    │   │  GND Pin ◄────────────────────── ESP32-C3 (GND)      │   │
    │   │                                                      │   │
    │   └──────────────────────────────────────────────────────┘   │
    │                                                             │
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
| SIM800L | 10mA (idle) | 2A (transmit) |
| Motors (4x) | 500mA | 2A (stall each) |
| GPS | 25mA | 50mA |
| MPU6050 | 3mA | 10mA |
| DS18B20 | 1mA | 5mA |
| Ultrasonic | 2mA | 15mA |

**Total Rover Peak: ~6A** (ensure buck converter can handle this)

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

| ESP32-C3 GPIO | Function | RPi Zero Pin |
|---------------|----------|--------------|
| 20 | UART RX | Pin 8 (TXD) |
| 21 | UART TX | Pin 10 (RXD) |
| 8 | LED | Built-in |

---

*Document Version: 1.0*  
*Last Updated: February 2026*

This document provides comprehensive wiring diagrams for all components of the Aero Sentinel system.

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

### ESP32-C3 to Raspberry Pi Zero

```
    ESP32-C3                    Raspberry Pi Zero
    ┌─────────────┐             ┌─────────────────────┐
    │             │             │                     │
    │  GPIO 21    │────────────►│ Pin 10 (GPIO 15 RX) │
    │  (TX)       │             │                     │
    │             │             │                     │
    │  GPIO 20    │◄────────────│ Pin 8 (GPIO 14 TX)  │
    │  (RX)       │             │                     │
    │             │             │                     │
    │  5V/3.3V    │◄────────────│ Pin 2 or 4 (5V)     │
    │             │             │                     │
    │  GND        │────────────►│ Pin 6 (GND)         │
    │             │             │                     │
    └─────────────┘             └─────────────────────┘
```

### Raspberry Pi Zero GPIO Header (Complete)

```
    Raspberry Pi Zero GPIO Header (40-pin)
    ┌────────────────────────────────────────┐
    │  3.3V  [01] [02] 5V                    │
    │  SDA   [03] [04] 5V                    │
    │  SCL   [05] [06] GND                   │
    │  GPIO4 [07] [08] TXD  ─────► ESP32-C3 RX
    │  GND   [09] [10] RXD  ◄───── ESP32-C3 TX
    │  GPIO17[11] [12] GPIO18                │
    │  GPIO27[13] [14] GND                   │
    │  GPIO22[15] [16] GPIO23                │
    │  3.3V  [17] [18] GPIO24                │
    │  MOSI  [19] [20] GND                   │
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

### LCD Display Connection (Optional)

```
    LCD 16x2 I2C            Raspberry Pi Zero
    ┌─────────────┐         ┌─────────────────┐
    │  VCC        │────────►│ Pin 2 (5V)      │
    │  GND        │────────►│ Pin 6 (GND)     │
    │  SDA        │────────►│ Pin 3 (GPIO 2)  │
    │  SCL        │────────►│ Pin 5 (GPIO 3)  │
    └─────────────┘         └─────────────────┘
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
    │   │  5V Pin ────────────────────────► ESP32-C3 (5V)      │   │
    │   │  GND Pin ◄────────────────────── ESP32-C3 (GND)      │   │
    │   │                                                      │   │
    │   └──────────────────────────────────────────────────────┘   │
    │                                                             │
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
| SIM800L | 10mA (idle) | 2A (transmit) |
| Motors (4x) | 500mA | 2A (stall each) |
| GPS | 25mA | 50mA |
| MPU6050 | 3mA | 10mA |
| DS18B20 | 1mA | 5mA |
| Ultrasonic | 2mA | 15mA |

**Total Rover Peak: ~6A** (ensure buck converter can handle this)

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

| ESP32-C3 GPIO | Function | RPi Zero Pin |
|---------------|----------|--------------|
| 20 | UART RX | Pin 8 (TXD) |
| 21 | UART TX | Pin 10 (RXD) |
| 8 | LED | Built-in |

---

*Document Version: 1.0*  
*Last Updated: February 2026*

