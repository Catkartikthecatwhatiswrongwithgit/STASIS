/**
 * STASIS Rover - Multi-State Autonomous Controller
 * ====================================================
 * ESP32-S3 Main Firmware
 * 
 * Handles: Navigation, Motor Control, Sensor Fusion, State Machine,
 *          Dock Detection, GPS Geofence, Charging Safety
 * 
 * Hardware: ESP32-S3, L9110S motors, DS18B20, HC-SR04, Neo-6M GPS, SIM800L
 * 
 * Author: EdgeCase Team
 */

#include <Arduino.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// =============================================================================
// HARDWARE CONFIGURATION - Updated for new requirements
// =============================================================================

// Motor Driver Pins (L9110S - 4WD paired)
#define MOTOR_A_PWM    4
#define MOTOR_A_DIR    2
#define MOTOR_B_PWM    5
#define MOTOR_B_DIR    18
#define MOTOR_C_PWM    6
#define MOTOR_C_DIR    7
#define MOTOR_D_PWM    15
#define MOTOR_D_DIR    16

// Sensors
#define ULTRASONIC_TRIG 10
#define ULTRASONIC_ECHO 9
#define ONEWIRE_PIN     21   // DS18B20
#define BUZZER_PIN      14
#define STATUS_LED      45

// GPS (NEO-6M) - SoftwareSerial on ESP32
#define GPS_TX_PIN      8
#define GPS_RX_PIN      19

// SIM800L GSM Module
#define SIM800_TX       13
#define SIM800_RX       12

// Battery Monitor (ADC)
#define BATTERY_ADC     1

// Optional: Dedicated dock detect GPIO (if wired)
// #define DOCK_DETECT_PIN  47  // Uncomment if hardware present

// UART for Pi Zero communication
#define UART_TX_PIN    43
#define UART_RX_PIN    44
#define UART_NUM       UART_NUM_1
#define UART_BAUD      115200

// PWM Configuration
#define PWM_FREQ       1000
#define PWM_RESOLUTION 8

// =============================================================================
// CONFIGURATION
// =============================================================================

// Communication Timeouts
#define COMMAND_TIMEOUT_MS  500   // Stop if no command from Pi
#define ESPNOW_TIMEOUT_MS   1000  // Stop if no ESP-NOW from base

// =============================================================================
// COMMAND PROTOCOL STRUCTURES
// =============================================================================

/**
 * DockCmd: Compact command from Pi Zero for docking
 * Packet format (6 bytes):
 *   [0] HEADER: 0xAA
 *   [1] turn:   int8_t  -100 to +100
 *   [2] speed:  uint8_t 0-100
 *   [3] docked: uint8_t 0 or 1
 *   [4] checksum: XOR of bytes 1-3
 *   [5] TAIL: 0x55
 */
struct DockCommand {
    int8_t turn;       // -100 to +100
    uint8_t speed;     // 0 to 100
    uint8_t docked;    // 0 or 1
    uint8_t checksum;
};

// =============================================================================
// STATE MACHINE DEFINITIONS - Validated for production
// =============================================================================

enum RoverState {
    STATE_SAFE_IDLE = 0,      // Minimal power, sensors active
    STATE_PATROL,             // Autonomous patrol with geofence
    STATE_ALERT,              // Hazard detected - investigating
    STATE_RETURN_TO_BASE,     // Geofence violation or manual recall
    STATE_DOCKING,            // AprilTag precision approach
    STATE_DOCKED_CHARGING     // Passive charging, motors disabled
};

enum AlertType {
    ALERT_NONE = 0,
    ALERT_FIRE,
    ALERT_EARTHQUAKE,
    ALERT_GEOFENCE,
    ALERT_INTRUDER
};

// =============================================================================
// DOCK DETECTION MODULE - Robust battery voltage rise detection
// =============================================================================
// Approach: Compare current voltage to baseline, detect sustained rise
// Primary method: Monitor voltage increase when docked (charging raises voltage)
// Debouncing: Require sustained voltage rise for CONFIRM_TIME_MS
// Tolerates: Noisy contact, brief disconnects
// Non-blocking: Uses millis() timing

#define DOCK_VOLTAGE_RISE      0.4    // Minimum voltage rise above baseline (0.4V)
#define DOCK_CONFIRM_TIME_MS   2000   // Must sustain rise for 2 seconds
#define UNDOCK_CONFIRM_MS      1500   // Must be low for 1.5s to confirm undock
#define BASELINE_SAMPLES       10      // Samples to establish baseline

struct DockDetector {
    bool isDocked;
    bool wasDocked;
    uint32_t dockStartTime;
    uint32_t undockStartTime;
    float baselineVoltage;
    bool baselineStable;
    uint8_t sampleCount;
};

static DockDetector dockState = {
    false, false, 0, 0, 0.0, false, 0
};

/**
 * Robust dock detection using voltage rise above baseline
 * Call every battery update cycle (~100ms)
 * Returns true on rising edge (just docked)
 */
bool detectDock(float currentVoltage) {
    uint32_t now = millis();
    
    // Phase 1: Establish baseline (first 10 samples)
    if (!dockState.baselineStable) {
        if (dockState.sampleCount < BASELINE_SAMPLES) {
            dockState.baselineVoltage += currentVoltage;
            dockState.sampleCount++;
        } else if (dockState.sampleCount == BASELINE_SAMPLES) {
            dockState.baselineVoltage /= BASELINE_SAMPLES;
            dockState.baselineStable = true;
            Serial.print("DOCK: Baseline established at ");
            Serial.print(dockState.baselineVoltage);
            Serial.println("V");
            dockState.sampleCount++;
        }
        return false;
    }
    
    // Phase 2: Monitor for voltage rise (charging)
    float voltageRise = currentVoltage - dockState.baselineVoltage;
    bool voltageRising = (voltageRise >= DOCK_VOLTAGE_RISE);
    
    if (voltageRising) {
        // Start or continue dock confirmation timer
        if (dockState.undockStartTime > 0) {
            dockState.undockStartTime = 0;
        }
        
        if (dockState.dockStartTime == 0) {
            dockState.dockStartTime = now;
        }
        
        // Confirm dock after sustained voltage rise
        if ((now - dockState.dockStartTime) >= DOCK_CONFIRM_TIME_MS) {
            if (!dockState.isDocked) {
                dockState.isDocked = true;
                dockState.dockStartTime = 0;
                Serial.print("DOCK: Detected at ");
                Serial.print(currentVoltage);
                Serial.print("V (rise: ");
                Serial.print(voltageRise);
                Serial.println("V)");
            }
        }
    } else {
        // Voltage dropped - start undock timer
        if (dockState.dockStartTime > 0) {
            dockState.dockStartTime = 0;
        }
        
        if (dockState.undockStartTime == 0) {
            dockState.undockStartTime = now;
        }
        
        // Confirm undock after sustained low voltage
        if ((now - dockState.undockStartTime) >= UNDOCK_CONFIRM_MS) {
            if (dockState.isDocked) {
                dockState.isDocked = false;
                dockState.undockStartTime = 0;
                dockState.baselineStable = false;  // Reset baseline
                dockState.sampleCount = 0;
                dockState.baselineVoltage = 0;
                Serial.println("DOCK: Undocked - baseline reset");
            }
        }
    }
    
    // Return true on rising edge (just docked)
    bool justDocked = dockState.isDocked && !dockState.wasDocked;
    dockState.wasDocked = dockState.isDocked;
    
    return justDocked;
}

bool isDocked() {
    return dockState.isDocked;
}

// =============================================================================
// GPS GEOFENCE MODULE
// =============================================================================
// Uses flat-earth approximation (acceptable for small areas)
// Stores home position at startup
// Monitors distance from home
// Non-blocking, tolerant of GPS signal loss

#define GEOFENCE_RADIUS_M   50.0    // 50 meter radius
#define GPS_VALID_AGE_MS   10000   // Discard fixes older than 10s

struct GeofenceMonitor {
    float homeLat;
    float homeLon;
    bool homeSet;
    bool violated;
    uint32_t lastValidFix;
    float currentLat;
    float currentLon;
};

static GeofenceMonitor geofence = {
    0.0f, 0.0f, false, false, 0, 0.0f, 0.0f
};

// Flat-earth approximation for small distances
// Returns distance in meters
float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
    const float EARTH_RADIUS = 6371000.0;  // meters
    const float PI = 3.14159265;
    
    float dLat = (lat2 - lat1) * PI / 180.0;
    float dLon = (lon2 - lon1) * PI / 180.0;
    
    float a = sin(dLat / 2) * sin(dLat / 2) +
              cos(lat1 * PI / 180.0) * cos(lat2 * PI / 180.0) *
              sin(dLon / 2) * sin(dLon / 2);
    
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));
    
    return EARTH_RADIUS * c;
}

void setHomePosition(float lat, float lon) {
    geofence.homeLat = lat;
    geofence.homeLon = lon;
    geofence.homeSet = true;
    geofence.violated = false;
    Serial.print("GEOFENCE: Home set to ");
    Serial.print(lat, 6);
    Serial.print(", ");
    Serial.println(lon, 6);
}

void updateGPS(float lat, float lon, uint32_t fixAge) {
    if (fixAge > GPS_VALID_AGE_MS) {
        return;  // Ignore stale fixes
    }
    
    geofence.currentLat = lat;
    geofence.currentLon = lon;
    geofence.lastValidFix = millis();
    
    if (!geofence.homeSet) {
        // First valid fix - set as home
        setHomePosition(lat, lon);
        return;
    }
    
    // Check distance from home
    float distance = calculateDistance(
        geofence.homeLat, geofence.homeLon,
        lat, lon
    );
    
    if (distance > GEOFENCE_RADIUS_M && !geofence.violated) {
        geofence.violated = true;
        Serial.print("GEOFENCE: VIOLATION! Distance: ");
        Serial.println(distance);
    } else if (distance <= GEOFENCE_RADIUS_M * 0.8) {
        // Clear violation when back within 80% of fence (hysteresis)
        geofence.violated = false;
    }
}

bool isGeofenceViolated() {
    return geofence.violated;
}

bool isGPSValid() {
    return (millis() - geofence.lastValidFix) < GPS_VALID_AGE_MS;
}

// =============================================================================
// CHARGING SAFETY MODULE
// =============================================================================
// When docked:
// - Immediately stop all motors
// - Enter reduced-power sensor mode
// - Keep critical monitoring active
// - Log charge start timestamp
// - Attempt charge completion detection
// Non-blocking state machine

#define CHARGE_COMPLETE_VOLTAGE 8.4   // 4.2V per cell for 2S
#define CHARGE_COMPLETE_TIME    600000  // 10 minutes max charge

static bool chargeStarted = false;
static uint32_t chargeStartTime = 0;
static bool chargeComplete = false;

void onDockDetected() {
    chargeStarted = true;
    chargeStartTime = millis();
    chargeComplete = false;
    
    Serial.println("CHARGING: Dock detected - starting charge");
}

void onUndocked() {
    chargeStarted = false;
    chargeStartTime = 0;
    chargeComplete = false;
    
    Serial.println("CHARGING: Undocked - resetting charge state");
}

bool isChargingComplete(float voltage) {
    if (!chargeStarted || chargeComplete) {
        return chargeComplete;
    }
    
    // Check voltage threshold
    if (voltage >= CHARGE_COMPLETE_VOLTAGE) {
        chargeComplete = true;
        Serial.println("CHARGING: Complete - voltage threshold reached");
        return true;
    }
    
    // Check timeout
    if ((millis() - chargeStartTime) > CHARGE_COMPLETE_TIME) {
        chargeComplete = true;
        Serial.println("CHARGING: Complete - timeout");
        return true;
    }
    
    return false;
}

bool hasChargingStarted() {
    return chargeStarted;
}

// =============================================================================
// GSM ALERT ROUTINES - SIM800L
// =============================================================================
// Sends SMS alerts for critical events:
// - Geofence violation
// - Hazard detection
// Non-blocking with timeout

struct GSMState {
    bool initialized;
    bool ready;
    uint32_t initStartTime;
    String phoneNumber;
};

static GSMState gsm = {
    false, false, 0, "+1234567890"  // Configure your alert number
};

bool initGSM() {
    if (gsm.initialized) return gsm.ready;
    
    Serial2.println("AT");
    delay(500);
    
    String response = "";
    while (Serial2.available()) {
        response += (char)Serial2.read();
    }
    
    if (response.indexOf("OK") >= 0) {
        Serial2.println("AT+CMGF=1");  // Text mode
        delay(200);
        gsm.ready = true;
        gsm.initialized = true;
        Serial.println("GSM: Ready");
        return true;
    }
    
    gsm.initialized = true;
    Serial.println("GSM: Not ready");
    return false;
}

bool sendSMS(const char* message) {
    if (!gsm.ready) {
        Serial.println("GSM: Not ready for SMS");
        return false;
    }
    
    Serial2.print("AT+CMGS=\"");
    Serial2.print(gsm.phoneNumber.c_str());
    Serial2.println("\"");
    delay(500);
    
    Serial2.print(message);
    Serial2.write(26);  // Ctrl+Z to send
    delay(2000);
    
    String response = "";
    while (Serial2.available()) {
        response += (char)Serial2.read();
    }
    
    if (response.indexOf("OK") >= 0) {
        Serial.println("GSM: SMS sent");
        return true;
    }
    
    Serial.println("GSM: SMS failed");
    return false;
}

void sendGeofenceAlert() {
    if (alertSent) return;
    
    char msg[100];
    snprintf(msg, sizeof(msg), 
        "STASIS ALERT: Geofence violation! Lat: %.6f, Lon: %.6f",
        (double)latitude, (double)longitude);
    
    sendSMS(msg);
    alertSent = true;
}

void sendHazardAlert(AlertType type) {
    if (alertSent) return;
    
    const char* msg = "STASIS ALERT: Hazard detected! Investigating.";
    
    if (type == ALERT_FIRE) {
        msg = "STASIS ALERT: Fire detected! Immediate attention required.";
    } else if (type == ALERT_EARTHQUAKE) {
        msg = "STASIS ALERT: Seismic activity detected!";
    }
    
    sendSMS(msg);
    alertSent = true;
}

// =============================================================================
// COMMAND PARSING
// =============================================================================

String commandBuffer = "";

void processCommand(String cmd) {
    cmd.trim();
    cmd.toUpperCase();
    
    if (cmd == "STOP") {
        currentState = STATE_SAFE_IDLE;
        currentSpeed = 0;
        currentTurn = 0;
        drive(0, 0);
        Serial.println("CMD:STOP_OK");
    }
    else if (cmd == "FORWARD") {
        currentTurn = 0;
        currentSpeed = 50;
        if (currentState == STATE_SAFE_IDLE) currentState = STATE_PATROL;
    }
    else if (cmd == "BACKWARD") {
        currentTurn = 0;
        currentSpeed = 50;
        drive(0, 0);
    }
    else if (cmd == "LEFT") {
        currentTurn = -50;
        currentSpeed = 30;
        if (currentState == STATE_SAFE_IDLE) currentState = STATE_PATROL;
    }
    else if (cmd == "RIGHT") {
        currentTurn = 50;
        currentSpeed = 30;
        if (currentState == STATE_SAFE_IDLE) currentState = STATE_PATROL;
    }
    else if (cmd == "PATROL" || cmd == "START_PATROL") {
        currentState = STATE_PATROL;
        currentSpeed = 40;
        currentTurn = 0;
        alertSent = false;
    }
    else if (cmd == "STOP_PATROL") {
        currentState = STATE_SAFE_IDLE;
        currentSpeed = 0;
    }
    else if (cmd == "HOME" || cmd == "RETURN_HOME" || cmd == "RETURN_BASE") {
        currentState = STATE_RETURN_TO_BASE;
        currentSpeed = 40;
        alertSent = false;
    }
    else if (cmd == "DOCKING" || cmd == "ENABLE_DOCKING") {
        currentState = STATE_DOCKING;
        currentSpeed = 0;
    }
    else if (cmd == "DISABLE_DOCKING") {
        currentState = STATE_SAFE_IDLE;
    }
    else if (cmd == "RESET") {
        currentState = STATE_SAFE_IDLE;
        currentSpeed = 0;
        currentTurn = 0;
        currentAlert = ALERT_NONE;
        estopTriggered = false;
        alertSent = false;
        Serial.println("CMD:RESET_OK");
    }
    else if (cmd == "ALERT") {
        currentState = STATE_ALERT;
        currentSpeed = 0;
    }
    else if (cmd == "SETHOME") {
        if (latitude != 0.0 && longitude != 0.0) {
            setHomePosition(latitude, longitude);
            Serial.println("CMD:SETHOME_OK");
        } else {
            Serial.println("CMD:SETHOME_FAIL - no GPS fix");
        }
    }
    else {
        Serial.println("CMD:UNKNOWN");
    }
    
    lastCommandTime = millis();
}

// =============================================================================
// MOTOR CONTROL - L9110S DRIVER
// =============================================================================

/**
 * Set motor speed and direction for single motor
 * L9110S: IA = direction, IB = PWM speed
 */
void setMotor(uint8_t pwmPin, uint8_t dirPin, int8_t speed) {
    // Clamp speed
    speed = constrain(speed, -100, 100);
    
    if (speed >= 0) {
        digitalWrite(dirPin, HIGH);  // Forward
        ledcWrite(pwmPin, map(speed, 0, 100, 0, 255));
    } else {
        digitalWrite(dirPin, LOW);   // Reverse
        ledcWrite(pwmPin, map(-speed, 0, 100, 0, 255));
    }
}

/**
 * Differential drive control
 * turn: -100 (left) to +100 (right)
 * speed: 0-100 (forward)
 */
void drive(int8_t turn, uint8_t speed) {
    // Apply low power mode reduction
    if (lowPowerMode) {
        speed = speed * 0.6;
    }
    
    // Differential steering
    int8_t leftSpeed, rightSpeed;
    
    if (turn == 0) {
        // Straight
        leftSpeed = speed;
        rightSpeed = speed;
    } else if (turn > 0) {
        // Turn right - right motor slower
        rightSpeed = speed - (int8_t)((speed * turn) / 100);
        leftSpeed = speed;
    } else {
        // Turn left - left motor slower
        leftSpeed = speed + (int8_t)((speed * turn) / 100);  // turn is negative
        rightSpeed = speed;
    }
    
    // Clamp
    leftSpeed = constrain(leftSpeed, -100, 100);
    rightSpeed = constrain(rightSpeed, -100, 100);
    
    // Drive all 4 motors (2 left, 2 right)
    setMotor(MOTOR_A_PWM, MOTOR_A_DIR, leftSpeed);
    setMotor(MOTOR_B_PWM, MOTOR_B_DIR, leftSpeed);
    setMotor(MOTOR_C_PWM, MOTOR_C_DIR, rightSpeed);
    setMotor(MOTOR_D_PWM, MOTOR_D_DIR, rightSpeed);
}

/**
 * Emergency stop - all motors off
 */
void estop() {
    estopTriggered = true;
    digitalWrite(MOTOR_A_PWM, LOW);
    digitalWrite(MOTOR_B_PWM, LOW);
    digitalWrite(MOTOR_C_PWM, LOW);
    digitalWrite(MOTOR_D_PWM, LOW);
    digitalWrite(MOTOR_A_DIR, LOW);
    digitalWrite(MOTOR_B_DIR, LOW);
    digitalWrite(MOTOR_C_DIR, LOW);
    digitalWrite(MOTOR_D_DIR, LOW);
}

// =============================================================================
// COMMAND PARSING - FROM PI ZERO
// =============================================================================

/**
 * Parse docking command from Pi Zero
 * Non-blocking UART parsing with packet framing
 */
bool parseDockCommand(uint8_t byte, DockCommand* cmd) {
    static uint8_t buffer[6];
    static uint8_t index = 0;
    static bool waitingForHeader = true;
    
    if (waitingForHeader) {
        if (byte == 0xAA) {
            buffer[0] = byte;
            index = 1;
            waitingForHeader = false;
        }
        return false;
    }
    
    buffer[index++] = byte;
    
    if (index >= 6) {
        // Full packet received
        waitingForHeader = true;
        
        // Verify tail
        if (buffer[5] != 0x55) {
            return false;
        }
        
        // Verify checksum
        uint8_t checksum = buffer[1] ^ buffer[2] ^ buffer[3];
        if (checksum != buffer[4]) {
            return false;
        }
        
        // Parse command
        cmd->turn = (int8_t)buffer[1];
        cmd->speed = buffer[2];
        cmd->docked = buffer[3];
        
        return true;
    }
    
    return false;
}

// =============================================================================
// ULTRASONIC SENSOR - NON-BLOCKING IMPLEMENTATION
// =============================================================================

#define ULTRASONIC_MAX_DISTANCE_CM  400
#define ULTRASONIC_TIMEOUT_US        25000

static struct {
    uint8_t state;            // 0=idle, 1=triggering, 2=waiting_echo, 3=measuring
    uint32_t triggerTime;
    uint32_t echoStartTime;
    float lastDistance;
} ultrasonic = {0, 0, 0, 0, 999.0};

/**
 * Non-blocking ultrasonic read
 * Call repeatedly from sensor task
 * Returns last valid distance or 999.0 if no reading
 */
float readUltrasonic() {
    uint32_t now = micros();
    
    switch (ultrasonic.state) {
        case 0:  // Idle - trigger new reading
            digitalWrite(ULTRASONIC_TRIG, HIGH);
            ultrasonic.triggerTime = now;
            ultrasonic.state = 1;
            break;
            
        case 1:  // Wait for trigger pulse (10us)
            if (now - ultrasonic.triggerTime >= 10) {
                digitalWrite(ULTRASONIC_TRIG, LOW);
                ultrasonic.state = 2;
            }
            break;
            
        case 2:  // Wait for echo start
            if (digitalRead(ULTRASONIC_ECHO) == HIGH) {
                ultrasonic.echoStartTime = now;
                ultrasonic.state = 3;
            } else if (now - ultrasonic.triggerTime > ULTRASONIC_TIMEOUT_US) {
                // Timeout - no echo received
                ultrasonic.state = 0;
            }
            break;
            
        case 3:  // Wait for echo end
            if (digitalRead(ULTRASONIC_ECHO) == LOW) {
                uint32_t echoDuration = now - ultrasonic.echoStartTime;
                if (echoDuration < ULTRASONIC_TIMEOUT_US) {
                    ultrasonic.lastDistance = (echoDuration * 0.0343) / 2.0;
                    ultrasonic.lastDistance = constrain(ultrasonic.lastDistance, 0, ULTRASONIC_MAX_DISTANCE_CM);
                }
                ultrasonic.state = 0;
            } else if (now - ultrasonic.echoStartTime > ULTRASONIC_TIMEOUT_US) {
                // Echo timeout
                ultrasonic.state = 0;
            }
            break;
            
        default:
            ultrasonic.state = 0;
            break;
    }
    
    return ultrasonic.lastDistance;
}

// =============================================================================
// TEMPERATURE SENSOR
// =============================================================================

// Simple DS18B20 reading (would use OneWire library in production)
float readTemperature() {
    // Placeholder - actual implementation would use OneWire + DallasTemperature
    // For now, return simulated value
    return temperature;
}

// =============================================================================
// BATTERY MONITORING - Updated for 2S Li-ion (7.4V nominal)
// =============================================================================

void updateBattery() {
    // Read ADC and calculate battery percentage
    // 2S Li-ion: 6.0V (empty) to 8.4V (full)
    // Voltage divider: 30k + 10k = 40k total, ratio = 10/40 = 0.25
    int adcValue = analogRead(BATTERY_ADC);
    float adcVoltage = (adcValue / 4095.0) * 3.3;
    batteryVoltage = adcVoltage / 0.25;
    
    // Percentage: 6.0V = 0%, 8.4V = 100%
    batteryPercent = ((batteryVoltage - 6.0) / 2.4) * 100.0;
    batteryPercent = constrain(batteryPercent, 0, 100);
    
    // Dock detection via voltage rise
    bool justDocked = detectDock(batteryVoltage);
    
    if (justDocked) {
        onDockDetected();
    } else if (!isDocked() && chargeStarted) {
        onUndocked();
    }
    
    // Enable low power mode
    lowPowerMode = (batteryPercent < LOW_BATTERY_THRESHOLD);
}

// =============================================================================
// STATE MACHINE LOGIC - Updated for charging and geofence
// =============================================================================

/**
 * Check transitions between states
 * Event-driven, non-blocking, robust to sensor noise
 */
void updateStateMachine() {
    uint32_t currentTime = millis();
    previousState = currentState;
    
    switch (currentState) {
        case STATE_SAFE_IDLE:
            // Stay idle until command received or docked
            if (isDocked()) {
                currentState = STATE_DOCKED_CHARGING;
            } else if (dockingEnabled) {
                currentState = STATE_DOCKING;
            }
            break;
            
        case STATE_PATROL:
            // Transitions:
            // Geofence violation takes priority - send alert
            if (isGeofenceViolated() && !alertSent) {
                currentState = STATE_RETURN_TO_BASE;
                currentAlert = ALERT_GEOFENCE;
                alertStartTime = currentTime;
                sendGeofenceAlert();
            }
            // Hazard detection
            else if (fireDetected || humanDetected || tiltDetected) {
                currentState = STATE_ALERT;
                alertStartTime = currentTime;
                alertConfirmed = false;
                // Determine alert type
                if (fireDetected) currentAlert = ALERT_FIRE;
                else if (tiltDetected) currentAlert = ALERT_EARTHQUAKE;
                else currentAlert = ALERT_INTRUDER;
            }
            // Low battery
            else if (batteryPercent < LOW_BATTERY_THRESHOLD) {
                currentState = STATE_RETURN_TO_BASE;
            }
            // Manual docking
            else if (dockingEnabled) {
                currentState = STATE_DOCKING;
            }
            // Auto-dock detection
            else if (isDocked()) {
                currentState = STATE_DOCKED_CHARGING;
            }
            break;
            
        case STATE_ALERT:
            // Send alert on first entry to alert state
            if (currentTime - stateEntryTime < 1000) {
                sendHazardAlert(currentAlert);
            }
            
            // Sensor fusion: require BOTH temperature AND camera flag for fire
            if (currentAlert == ALERT_FIRE) {
                if (temperature > 60.0 && fireDetected) {
                    alertConfirmed = true;
                }
            }
            
            // Timeout after 30 seconds if not confirmed
            if (currentTime - alertStartTime > 30000) {
                currentState = STATE_PATROL;
                currentAlert = ALERT_NONE;
                alertSent = false;
            }
            
            // Can return to base if commanded
            if (dockingEnabled) {
                currentState = STATE_DOCKING;
            }
            break;
            
        case STATE_RETURN_TO_BASE:
            // Navigate toward base (using GPS - simplified)
            if (dockingEnabled) {
                currentState = STATE_DOCKING;
            }
            else if (isDocked()) {
                currentState = STATE_DOCKED_CHARGING;
            }
            // Return to patrol if back within geofence (if not geofence alert)
            else if (!isGeofenceViolated() && currentAlert != ALERT_GEOFENCE) {
                currentState = STATE_PATROL;
                currentAlert = ALERT_NONE;
                alertSent = false;
            }
            // Also clear alert if manually reset
            else if (currentAlert == ALERT_GEOFENCE && !isGeofenceViolated()) {
                alertSent = false;
            }
            break;
            
        case STATE_DOCKING:
            // Use commands from Pi Zero
            if (isDocked() || (currentTime - lastCommandTime > COMMAND_TIMEOUT_MS)) {
                currentState = STATE_DOCKED_CHARGING;
                estop();
            }
            // Abort if undocked mid-docking
            if (!isDocked() && !dockingEnabled) {
                currentState = STATE_SAFE_IDLE;
            }
            break;
            
        case STATE_DOCKED_CHARGING:
            // Charging state - stay here until undocked
            if (!isDocked()) {
                currentState = STATE_SAFE_IDLE;
                alertSent = false;  // Reset alert flag on undock
            }
            // Manual undock command
            if (!dockingEnabled && currentTime - stateEntryTime > 2000) {
                // Require 2 seconds docked before allowing undock
            }
            break;
    }
    
    // Update entry time on state change
    if (currentState != previousState) {
        stateEntryTime = millis();
        Serial.print("State: ");
        Serial.println(currentState);
    }
}

/**
 * Execute current state behavior
 * Includes charging safety behavior when docked
 */
void executeState() {
    uint32_t currentTime = millis();
    
    switch (currentState) {
        case STATE_SAFE_IDLE:
            // Full stop - minimal power
            currentTurn = 0;
            currentSpeed = 0;
            drive(0, 0);
            digitalWrite(STATUS_LED, (currentTime / 1000) % 2);  // Slow blink
            break;
            
        case STATE_PATROL:
            // Normal patrol behavior
            currentTurn = 0;
            currentSpeed = lowPowerMode ? 30 : 50;
            digitalWrite(STATUS_LED, HIGH);
            break;
            
        case STATE_ALERT:
            // Move toward alert source, or stop and investigate
            currentTurn = 0;
            currentSpeed = 0;
            // Alert buzzer pattern
            if ((currentTime / 500) % 2 == 0) {
                digitalWrite(BUZZER_PIN, HIGH);
            } else {
                digitalWrite(BUZZER_PIN, LOW);
            }
            digitalWrite(STATUS_LED, (currentTime / 200) % 2);  // Fast blink
            break;
            
        case STATE_RETURN_TO_BASE:
            // Navigate toward base coordinates
            // Simplified: just go straight (GPS waypoint would go here)
            currentTurn = 0;
            currentSpeed = lowPowerMode ? 20 : 40;
            digitalWrite(STATUS_LED, (currentTime / 500) % 2);  // Medium blink
            break;
            
        case STATE_DOCKING:
            // Use commands from Pi Zero via UART
            // Check command timeout
            if (currentTime - lastCommandTime > COMMAND_TIMEOUT_MS) {
                // No recent command - STOP for safety
                currentTurn = 0;
                currentSpeed = 0;
            }
            // Otherwise use currentTurn and currentSpeed from UART
            break;
            
        case STATE_DOCKED_CHARGING:
            // CHARGING SAFETY: Immediately stop all motors
            currentTurn = 0;
            currentSpeed = 0;
            drive(0, 0);
            
            // Reduced-power sensor mode - continue monitoring
            // Keep critical monitoring active (battery, temp)
            
            // Log charge status periodically (non-blocking check)
            static uint32_t lastChargeLog = 0;
            if (currentTime - lastChargeLog > 10000) {
                Serial.print("CHARGING: Voltage=");
                Serial.print(batteryVoltage);
                Serial.print("V, Started=");
                Serial.println(hasChargingStarted() ? "YES" : "NO");
                lastChargeLog = currentTime;
            }
            
            // Check for charge complete - non-blocking beep
            static bool chargeBeepDone = false;
            if (isChargingComplete(batteryVoltage) && !chargeBeepDone) {
                // Queue beep - will be done by main loop
                chargeBeepDone = true;
            }
            
            // LED indication - solid when charging
            digitalWrite(STATUS_LED, HIGH);
            break;
    }
    
    // Apply motor commands (except in DOCKING which uses UART commands directly)
    if (currentState != STATE_DOCKING && currentState != STATE_DOCKED_CHARGING) {
        drive(currentTurn, currentSpeed);
    }
}

// =============================================================================
// TASKS
// =============================================================================

/**
 * UART receive task - from Pi Zero
 * Handles both text commands and docking commands
 */
void uartTask(void* parameter) {
    uint8_t buffer[1];
    DockCommand cmd;
    static String textCmd = "";
    
    while (true) {
        int length = uart_read_bytes(UART_NUM, buffer, 1, pdMS_TO_TICKS(10));
        if (length > 0) {
            char c = (char)buffer[0];
            
            // Check for docking protocol (starts with 0xAA)
            if (c == 0xAA && currentState == STATE_DOCKING) {
                if (parseDockCommand(buffer[0], &cmd)) {
                    currentTurn = cmd.turn;
                    currentSpeed = cmd.speed;
                    lastCommandTime = millis();
                    estopTriggered = false;
                }
            }
            // Text command parsing
            else if (c == '\n' || c == '\r') {
                if (textCmd.length() > 0) {
                    processCommand(textCmd);
                    textCmd = "";
                }
            }
            else if (c >= 32) {  // Printable characters
                textCmd += c;
                if (textCmd.length() >= 16) {
                    textCmd = "";  // Reset if too long
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// =============================================================================
// GPS PARSING - NEO-6M NMEA sentences
// =============================================================================

String gpsBuffer = "";

void parseGPSLine(String& line) {
    // Look for GGA sentence (contains position data)
    if (!line.startsWith("$GPGGA") && !line.startsWith("$GNGGA")) {
        return;
    }
    
    // Parse GGA: $GPGGA,time,lat,NS,lon,EW,quality,satellites,hdop,alt,M,geoid,M,...
    int commas[15];
    int commaCount = 0;
    
    for (int i = 0; i < line.length() && commaCount < 15; i++) {
        if (line.charAt(i) == ',') {
            commas[commaCount++] = i;
        }
    }
    
    if (commaCount < 9) return;
    
    // Check GPS quality (0 = no fix, 1 = GPS, 2 = DGPS)
    int quality = line.substring(commas[5] + 1, commas[6]).toInt();
    if (quality == 0) return;  // No valid fix
    
    // Parse latitude
    String latStr = line.substring(commas[1] + 1, commas[2]);
    String latDir = line.substring(commas[2] + 1, commas[3]);
    if (latStr.length() > 0) {
        float lat = latStr.toFloat();
        // Convert DDMM.MMMM to degrees
        int latDeg = lat / 100;
        float latMin = lat - (latDeg * 100);
        latitude = latDeg + (latMin / 60.0);
        if (latDir == "S") latitude = -latitude;
    }
    
    // Parse longitude
    String lonStr = line.substring(commas[3] + 1, commas[4]);
    String lonDir = line.substring(commas[4] + 1, commas[5]);
    if (lonStr.length() > 0) {
        float lon = lonStr.toFloat();
        // Convert DDDMM.MMMM to degrees
        int lonDeg = lon / 100;
        float lonMin = lon - (lonDeg * 100);
        longitude = lonDeg + (lonMin / 60.0);
        if (lonDir == "W") longitude = -longitude;
    }
    
    // Update geofence
    gpsFixAge = 0;
    updateGPS(latitude, longitude, 0);
}

/**
 * GPS polling task - reads from Serial (GPS)
 * Non-blocking NMEA parsing
 */
void gpsTask(void* parameter) {
    uint32_t lastGPSUpdate = 0;
    
    while (true) {
        // Read available GPS characters
        while (Serial2.available()) {
            char c = Serial2.read();
            if (c == '\n' || c == '\r') {
                if (gpsBuffer.length() > 0) {
                    parseGPSLine(gpsBuffer);
                    gpsBuffer = "";
                    lastGPSUpdate = millis();
                }
            } else if (gpsBuffer.length() < 100) {
                gpsBuffer += c;
            }
        }
        
        // Age the fix based on time since last valid update
        if (lastGPSUpdate > 0) {
            gpsFixAge = millis() - lastGPSUpdate;
        } else {
            gpsFixAge = 60000;  // No fix yet
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/**
 * Camera forwarding task
 * Reads from Serial (camera) and forwards to Pi via UART1
 */
void cameraTask(void* parameter) {
    while (true) {
        if (Serial.available()) {
            String line = Serial.readStringUntil('\n');
            line.trim();
            if (line.length() > 0) {
                // Forward to Pi via UART
                line += "\n";
                uart_write_bytes(UART_NUM, line.c_str(), line.length());
                // Also parse for hazard detection
                if (line.startsWith("HAZARD:")) {
                    String hazardType = line.substring(7);
                    hazardType.trim();
                    if (hazardType == "FIRE") fireDetected = true;
                    else if (hazardType == "MOTION") humanDetected = true;
                    else if (hazardType == "HUMAN") humanDetected = true;
                    else if (hazardType == "CLEAR") {
                        fireDetected = false;
                        humanDetected = false;
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * Sensor polling task - non-critical sensors
 */
void sensorTask(void* parameter) {
    while (true) {
        // Read ultrasonic (now non-blocking)
        distance = readUltrasonic();
        
        // Obstacle avoidance override - only in patrol/return states
        if ((currentState == STATE_PATROL || currentState == STATE_RETURN_TO_BASE) 
            && distance < 20.0 && currentSpeed > 20) {
            // Slow down for obstacles - non-blocking
            currentSpeed = 15;
        }
        
        // Read temperature (placeholder - would use OneWire library)
        // temperature = readTemperature();
        
        // Update battery and dock detection
        updateBattery();
        
        // GPS is handled by gpsTask
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/**
 * State machine task
 */
void stateTask(void* parameter) {
    while (true) {
        updateStateMachine();
        executeState();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// =============================================================================
// SETUP
// =============================================================================

void setup() {
    Serial.begin(115200);
    Serial.println("STASIS Rover Initializing...");
    
    // Configure motor pins
    pinMode(MOTOR_A_PWM, OUTPUT);
    pinMode(MOTOR_A_DIR, OUTPUT);
    pinMode(MOTOR_B_PWM, OUTPUT);
    pinMode(MOTOR_B_DIR, OUTPUT);
    pinMode(MOTOR_C_PWM, OUTPUT);
    pinMode(MOTOR_C_DIR, OUTPUT);
    pinMode(MOTOR_D_PWM, OUTPUT);
    pinMode(MOTOR_D_DIR, OUTPUT);
    
    // Configure PWM for motors (ESP32 Arduino Core 3.x API)
    ledcAttach(MOTOR_A_PWM, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(MOTOR_B_PWM, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(MOTOR_C_PWM, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(MOTOR_D_PWM, PWM_FREQ, PWM_RESOLUTION);
    
    // Configure ultrasonic
    pinMode(ULTRASONIC_TRIG, OUTPUT);
    pinMode(ULTRASONIC_ECHO, INPUT);
    
    // Configure buzzer and status LED
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(STATUS_LED, LOW);
    
    // Configure UART for Pi Zero
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, 1024, 1024, 0, NULL, 0);
    
    // Configure Serial2 for GPS (NEO-6M)
    // GPS uses 9600 baud typically
    Serial2.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    
    // Initialize ADC for battery
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);  // Allow up to ~3.3V input
    
    // Initialize dock detection baseline
    dockState.baselineStable = false;
    dockState.sampleCount = 0;
    dockState.isDocked = false;
    dockState.wasDocked = false;
    dockState.dockStartTime = 0;
    dockState.undockStartTime = 0;
    dockState.baselineVoltage = 0;
    
    // Initialize geofence
    geofence.homeSet = false;
    geofence.violated = false;
    geofence.lastValidFix = 0;
    
    // Initialize GSM (non-blocking, will be ready after warmup)
    // initGSM();  // Uncomment when SIM800 wired
    
    // Create tasks
    xTaskCreatePinnedToCore(uartTask, "UART", 2048, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(cameraTask, "Camera", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(sensorTask, "Sensors", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(gpsTask, "GPS", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(stateTask, "State", 2048, NULL, 1, NULL, 1);
    
    lastCommandTime = millis();
    stateEntryTime = millis();
    
    Serial.println("STASIS Rover Ready");
    Serial.println("States: SAFE_IDLE, PATROL, ALERT, RETURN_TO_BASE, DOCKING, DOCKED_CHARGING");
}

// =============================================================================
// MAIN LOOP
// =============================================================================

void loop() {
    // Main loop - most work done in tasks
    // Periodic safety check
    if (estopTriggered) {
        estop();
    }
    
    // Watchdog reset
    // esp_task_wdt_reset();
    
    delay(10);
}
