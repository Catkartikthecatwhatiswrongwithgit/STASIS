# 3D Printed Parts - Stasis Rover

This document describes the 3D printed parts used in the Stasis rover system.

---

## Overview

The rover chassis and enclosure are designed to be 3D printed. All parts are located in the `RoverProject/.stl files/` directory.

---

## Parts List

### 1. Base Chassis

| File | Description |
|------|-------------|
| `base-Body.stl` | Original base chassis design |
| `base resized-Body.stl` | Resized version of the base chassis |

**Purpose:** The main chassis that holds all electronics, motors, and sensors.

**Print Settings:**
- Material: PETG or PLA+
- Layer Height: 0.2mm
- Infill: 30-40%
- Supports: Required for overhangs
- Wall Thickness: 3 perimeters minimum

**Dimensions:**
- Original: Standard rover size
- Resized: Adjusted for specific component fit

---

### 2. Lid/Cover

| File | Description |
|------|-------------|
| `lid-Body.stl` | Original lid design |
| `lid resize-Body.stl` | Resized version of the lid |

**Purpose:** Top cover that protects internal components and provides mounting for sensors.

**Print Settings:**
- Material: PETG or PLA+
- Layer Height: 0.2mm
- Infill: 20-30%
- Supports: Required
- Wall Thickness: 3 perimeters

**Features:**
- Camera mounting hole
- Sensor access points
- Ventilation slots

---

### 3. Roof

| File | Description |
|------|-------------|
| `roof-Body.stl` | Original roof design |
| `roof resized-Body.stl` | Resized version of the roof |

**Purpose:** Weather protection layer above the lid.

**Print Settings:**
- Material: PETG (recommended for outdoor use)
- Layer Height: 0.2mm
- Infill: 20%
- Supports: Minimal
- Wall Thickness: 2-3 perimeters

---

### 4. Legs (Suspension/Wheels)

#### Long Legs

| File | Description |
|------|-------------|
| `long leg-Body.stl` | Original long leg design |
| `Long leg resized-Body.stl` | Resized long leg |

**Purpose:** Longer suspension arms for rough terrain capability.

**Print Settings:**
- Material: PETG or TPU (for flexibility)
- Layer Height: 0.15-0.2mm
- Infill: 40-50%
- Supports: Yes
- Wall Thickness: 4 perimeters

#### Short Legs

| File | Description |
|------|-------------|
| `short leg-Body.stl` | Original short leg design |
| `short leg resized-Body.stl` | Resized short leg |

**Purpose:** Shorter suspension arms for standard terrain.

**Print Settings:**
- Material: PETG or TPU
- Layer Height: 0.15-0.2mm
- Infill: 40-50%
- Supports: Yes
- Wall Thickness: 4 perimeters

#### Leg Set

| File | Description |
|------|-------------|
| `legs resized.stl` | Complete set of resized legs |

**Purpose:** All four legs in a single print layout.

---

### 5. File Enclosure

| File | Description |
|------|-------------|
| `file enclosure-Body.stl` | Electronics enclosure |

**Purpose:** Houses the ESP32-S3, motor drivers, and other electronics.

**Print Settings:**
- Material: PETG
- Layer Height: 0.2mm
- Infill: 30%
- Supports: Required for internal features
- Wall Thickness: 3 perimeters

**Features:**
- Mounting points for PCBs
- Cable routing channels
- Ventilation for heat dissipation

---

## Print Recommendations

### Material Selection

| Material | Pros | Cons | Recommended For |
|----------|------|------|-----------------|
| **PETG** | Weather resistant, strong, flexible | Stringing, harder to print | Outdoor parts, chassis |
| **PLA+** | Easy to print, strong | Not heat resistant | Indoor parts, prototypes |
| **TPU** | Flexible, impact resistant | Difficult to print | Legs, bumpers |
| **ABS** | Heat resistant, strong | Warping, fumes | High-temperature applications |

### Print Orientation

For best strength and surface finish:

1. **Base Chassis:** Print upside down (flat side on bed)
2. **Lid/Roof:** Print with top surface on bed
3. **Legs:** Print vertically for layer strength
4. **Enclosures:** Print with open side up

### Post-Processing

1. **Sanding:** Smooth layer lines for better appearance
2. **Priming:** Use filler primer for painted finishes
3. **Sealing:** Apply waterproof sealant for outdoor use
4. **Threaded Inserts:** Use heat-set inserts for screw holes

---

## Assembly Notes

### Hardware Required

| Item | Quantity | Purpose |
|------|----------|---------|
| M3 screws (10mm) | 20 | General assembly |
| M3 screws (16mm) | 10 | Motor mounting |
| M3 nuts | 30 | Screw fastening |
| M3 threaded inserts | 15 | Plastic threads |
| Heat shrink tubing | - | Wire insulation |
| Zip ties | - | Cable management |

### Assembly Order

1. Print all parts and verify fit
2. Install threaded inserts in plastic parts
3. Mount motors to base chassis
4. Install electronics in enclosure
5. Attach enclosure to base
6. Route all cables
7. Attach legs/wheels
8. Install lid and roof
9. Final cable management

---

## Design Modifications

The `.stl files/` directory contains both original and resized versions:

- **Original files:** Standard dimensions as designed
- **Resized files:** Modified for specific component fit or printer calibration

If you need to modify the designs, the original CAD files should be used (not included in this repository).

---

## Troubleshooting

### Parts Don't Fit

1. Check printer calibration (XY dimensions)
2. Verify scale is 100% in slicer
3. Try the resized versions
4. Adjust tolerance in slicer settings

### Layer Separation

1. Increase extrusion temperature
2. Reduce cooling fan speed
3. Increase infill overlap
4. Use stronger material (PETG)

### Warping

1. Use heated bed
2. Apply adhesive (glue stick, hairspray)
3. Use brim or raft
4. Enclose printer

---

## File Archive

A compressed archive is also available:

| File | Description |
|------|-------------|
| `.stl files.rar` | Archive of all STL files |

---

*Document Version: 1.0*  
*Last Updated: February 2026*
