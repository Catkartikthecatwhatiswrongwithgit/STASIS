================================================================================
STASIS ROVER - QUICK REFERENCE
================================================================================

## Commands
------------------------------------------------------------------------------
STOP              → Emergency stop
PATROL            → Start autonomous patrol
STOP_PATROL       → Stop patrol
HOME/RETURN_BASE  → Return to dock
DOCKING           → Enable AprilTag docking
SETHOME           → Set current GPS as home position
RESET             → Reset all states

## States
------------------------------------------------------------------------------
SAFE_IDLE         - Waiting, minimal power
PATROL            - Autonomous patrol
ALERT             - Hazard detected
RETURN_TO_BASE    - Returning home
DOCKING           - AprilTag approach
DOCKED_CHARGING   - Charging

## Safety Features
------------------------------------------------------------------------------
✓ Dock detection: 0.4V rise, 2s confirm, 20-frame debounce
✓ GPS geofence: 50m radius, ignores 0.0,0.0
✓ GPS staleness: SAFE_HALT after 5s no fix
✓ SIM800: Motors stop before SMS, 60s cooldown
✓ Ultrasonic: Non-blocking, 30ms timeout
✓ Motor ramping: Smooth acceleration

## Pin Map
------------------------------------------------------------------------------
Motors:    2,4,5,6,7,15,16,18
Ultrasonic: 9,10
GPS:       8,19
SIM800:    12,13
Buzzer:    14
LED:       45
Battery:   1

## Troubleshooting
------------------------------------------------------------------------------
No GPS fix:     Wait 30-60s outdoors, check antenna
Dock false positive: Increase DOCK_VOLTAGE_RISE
SMS not sending: Check SIM balance, power (2A)
Motors jerky:   Check power supply, reduce speed
Rover frozen:   Check ultrasonic timeout, reduce debug output

================================================================================
