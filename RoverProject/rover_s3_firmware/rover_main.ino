/**
 * STASIS Rover - Multi-State Autonomous Controller
 * ====================================================
 * ESP32-S3 Main Firmware
 * 
 * Handles: Navigation, Motor Control, Sensor Fusion, State Machine,
 *          Communication with Pi Zero, ESP-NOW, and SIM800
 * 
 * Hardware: ESP32-S3, L9110S motors, DS18B20, HC-SR04, MPU6050, Neo-6M GPS
 * 
 * Author: AeroSentinel
 */

#include <Arduino.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// =============================================================================
// CONFIGURATION
// =============================================================================

// Pin Definitions
#define MOTOR_A_PWM    4    // GPIO4 - Motor A speed (L9110S)
#define MOTOR_A_DIR    2    // GPIO2 - Motor A direction
#define MOTOR_B_PWM    5    // GPIO5 - Motor B speed
#define MOTOR_B_DIR    18   // GPIO18 - Motor B direction
#define MOTOR_C_PWM    6    // GPIO6 - Motor C speed
#define MOTOR_C_DIR    7    // GPIO7 - Motor C direction
#define MOTOR_D_PWM    15   // GPIO15 - Motor D speed
#define MOTOR_D_DIR    16   // GPIO16 - Motor D direction

#define ULTRASONIC_TRIG 10  // GPIO10 - HC-SR04 Trigger
#define ULTRASONIC_ECHO 9   // GPIO9 - HC-SR04 Echo

#define BUZZER_PIN     14   // GPIO14 - Piezo buzzer

#define ONEWIRE_PIN    11   // GPIO11 - DS18B20

// UART for Pi Zero communication
#define UART_TX_PIN    43
#define UART_RX_PIN    44
#define UART_NUM       UART_NUM_1
#define UART_BAUD      115200

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
// STATE MACHINE DEFINITIONS
// =============================================================================

enum RoverState {
    STATE_PATROL = 0,       // Normal patrol mode
    STATE_RESEARCH,         // Research/data collection
    STATE_ALERT,            // Hazard detected - investigating
    STATE_RETURN_TO_BASE,   // Low battery or manual recall
    STATE_DOCKING,          // AprilTag docking procedure
    STATE_DOCKED            // Successfully docked
};

enum AlertType {
    ALERT_NONE = 0,
    ALERT_FIRE,
    ALERT_EARTHQUAKE,
    ALERT_INTRUDER
};

// =============================================================================
// GLOBAL VARIABLES
// =============================================================================

// Current rover state
volatile RoverState currentState = STATE_PATROL;
volatile RoverState previousState = STATE_PATROL;

// Control inputs
volatile int8_t currentTurn = 0;
volatile uint8_t currentSpeed = 0;
volatile bool dockingEnabled = false;
volatile bool docked = false;

// Timing
volatile uint32_t lastCommandTime = 0;
volatile uint32_t lastEspNowTime = 0;
volatile uint32_t stateEntryTime = 0;

// Sensor data (updated by tasks)
volatile float temperature = 25.0;
volatile float distance = 999.0;  // cm
volatile float batteryVoltage = 12.0;
volatile float batteryPercent = 100.0;
volatile bool fireDetected = false;
volatile bool humanDetected = false;
volatile bool tiltDetected = false;
volatile float latitude = 0.0;
volatile float longitude = 0.0;

// Alert tracking
volatile AlertType currentAlert = ALERT_NONE;
volatile uint32_t alertStartTime = 0;
volatile bool alertConfirmed = false;

// Power management
volatile bool lowPowerMode = false;
const float LOW_BATTERY_THRESHOLD = 30.0;

// Safety
volatile bool estopTriggered = false;

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
        ledcWrite(0, map(speed, 0, 100, 0, 255));
    } else {
        digitalWrite(dirPin, LOW);   // Reverse
        ledcWrite(0, map(-speed, 0, 100, 0, 255));
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
// ULTRASONIC SENSOR - NON-BLOCKING
// =============================================================================

/**
 * Read ultrasonic distance (non-blocking)
 * Returns distance in cm, or 999.0 on timeout/error
 */
float readUltrasonic() {
    // Trigger pulse
    digitalWrite(ULTRASONIC_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIG, LOW);
    
    // Measure echo (with timeout)
    uint32_t timeout = 25000;  // ~4m max
    uint32_t start = micros();
    
    while (digitalRead(ULTRASONIC_ECHO) == LOW) {
        if (micros() - start > timeout) return 999.0;
    }
    
    uint32_t echoStart = micros();
    
    while (digitalRead(ULTRASONIC_ECHO) == HIGH) {
        if (micros() - echoStart > timeout) return 999.0;
    }
    
    uint32_t echoDuration = micros() - echoStart;
    
    // Convert to cm (speed of sound = 343 m/s)
    float distance = (echoDuration * 0.0343) / 2.0;
    
    return constrain(distance, 0, 400);
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
// BATTERY MONITORING
// =============================================================================

void updateBattery() {
    // Read ADC and calculate battery percentage
    // Assuming voltage divider and 12V battery
    int adcValue = analogRead(A0);
    batteryVoltage = (adcValue / 4095.0) * 3.3 * 4.2;  // Adjust divider ratio
    batteryPercent = ((batteryVoltage - 9.0) / 3.0) * 100.0;
    batteryPercent = constrain(batteryPercent, 0, 100);
    
    // Enable low power mode
    lowPowerMode = (batteryPercent < LOW_BATTERY_THRESHOLD);
}

// =============================================================================
// STATE MACHINE LOGIC
// =============================================================================

/**
 * Check transitions between states
 */
void updateStateMachine() {
    uint32_t currentTime = millis();
    previousState = currentState;
    
    switch (currentState) {
        case STATE_PATROL:
            // Transitions:
            if (fireDetected || humanDetected || tiltDetected) {
                currentState = STATE_ALERT;
                alertStartTime = currentTime;
                alertConfirmed = false;
            } else if (batteryPercent < LOW_BATTERY_THRESHOLD) {
                currentState = STATE_RETURN_TO_BASE;
            } else if (dockingEnabled) {
                currentState = STATE_DOCKING;
            }
            break;
            
        case STATE_RESEARCH:
            // Stay in research until timeout or alert
            if (fireDetected || humanDetected || tiltDetected) {
                currentState = STATE_ALERT;
                alertStartTime = currentTime;
            } else if (currentTime - stateEntryTime > 300000) {  // 5 min timeout
                currentState = STATE_PATROL;
            }
            break;
            
        case STATE_ALERT:
            // Sensor fusion: require BOTH temperature AND camera flag for fire
            if (currentAlert == ALERT_FIRE) {
                if (temperature > 60.0 && fireDetected) {
                    alertConfirmed = true;
                }
            }
            
            // Timeout after 30 seconds if not confirmed
            if (currentTime - alertStartTime > 30000) {
                if (alertConfirmed) {
                    // Send alert via SIM800
                    // Then return to patrol
                }
                currentState = STATE_PATROL;
            }
            break;
            
        case STATE_RETURN_TO_BASE:
            // Navigate toward base (using GPS)
            if (dockingEnabled) {
                currentState = STATE_DOCKING;
            }
            // Could add GPS waypoint navigation here
            break;
            
        case STATE_DOCKING:
            // Use commands from Pi Zero
            if (docked || (currentTime - lastCommandTime > COMMAND_TIMEOUT_MS)) {
                currentState = STATE_DOCKED;
                estop();
            }
            break;
            
        case STATE_DOCKED:
            // Stay docked, waiting
            if (!dockingEnabled) {
                currentState = STATE_PATROL;
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
 */
void executeState() {
    uint32_t currentTime = millis();
    
    switch (currentState) {
        case STATE_PATROL:
            // Normal patrol behavior
            // Could implement waypoint following or random walk
            currentTurn = 0;
            currentSpeed = lowPowerMode ? 30 : 50;
            break;
            
        case STATE_RESEARCH:
            // Slower, more thorough movement
            currentTurn = 0;
            currentSpeed = 20;
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
            break;
            
        case STATE_RETURN_TO_BASE:
            // Navigate toward base coordinates
            // Simplified: just go straight (would use GPS in production)
            currentTurn = 0;
            currentSpeed = lowPowerMode ? 20 : 40;
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
            
        case STATE_DOCKED:
            // Full stop
            currentTurn = 0;
            currentSpeed = 0;
            docked = true;
            break;
    }
    
    // Apply motor commands (except in DOCKING which uses UART commands directly)
    if (currentState != STATE_DOCKING) {
        drive(currentTurn, currentSpeed);
    }
}

// =============================================================================
// TASKS
// =============================================================================

/**
 * UART receive task - from Pi Zero
 */
void uartTask(void* parameter) {
    uint8_t buffer[1];
    DockCommand cmd;
    
    while (true) {
        int length = UART.read(UART_NUM, buffer, 1);
        if (length > 0) {
            if (parseDockCommand(buffer[0], &cmd)) {
                // Valid command received
                currentTurn = cmd.turn;
                currentSpeed = cmd.speed;
                docked = (cmd.docked == 1);
                lastCommandTime = millis();
                estopTriggered = false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/**
 * Sensor polling task
 */
void sensorTask(void* parameter) {
    while (true) {
        // Read ultrasonic (non-blocking handled internally)
        distance = readUltrasonic();
        
        // Obstacle avoidance override
        if (distance < 20.0 && currentSpeed > 20) {
            estop();
        }
        
        // Read temperature
        temperature = readTemperature();
        
        // Update battery
        updateBattery();
        
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
    
    // Configure PWM for motors
    ledcSetup(0, 1000, 8);  // Channel 0, 1kHz, 8-bit
    ledcAttachPin(MOTOR_A_PWM, 0);
    ledcSetup(1, 1000, 8);
    ledcAttachPin(MOTOR_B_PWM, 1);
    ledcSetup(2, 1000, 8);
    ledcAttachPin(MOTOR_C_PWM, 2);
    ledcSetup(3, 1000, 8);
    ledcAttachPin(MOTOR_D_PWM, 3);
    
    // Configure ultrasonic
    pinMode(ULTRASONIC_TRIG, OUTPUT);
    pinMode(ULTRASONIC_ECHO, INPUT);
    
    // Configure buzzer
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    
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
    
    // Initialize watchdog
    // In production: esp_task_wdt_init(5, true);
    
    // Create tasks
    xTaskCreatePinnedToCore(uartTask, "UART", 2048, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(sensorTask, "Sensors", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(stateTask, "State", 2048, NULL, 1, NULL, 1);
    
    lastCommandTime = millis();
    stateEntryTime = millis();
    
    Serial.println("STASIS Rover Ready");
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
 * STASIS Rover - Multi-State Autonomous Controller
 * ====================================================
 * ESP32-S3 Main Firmware
 * 
 * Handles: Navigation, Motor Control, Sensor Fusion, State Machine,
 *          Communication with Pi Zero, ESP-NOW, and SIM800
 * 
 * Hardware: ESP32-S3, L9110S motors, DS18B20, HC-SR04, MPU6050, Neo-6M GPS
 * 
 * Author: AeroSentinel
 */

#include <Arduino.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// =============================================================================
// CONFIGURATION
// =============================================================================

// Pin Definitions
#define MOTOR_A_PWM    4    // GPIO4 - Motor A speed (L9110S)
#define MOTOR_A_DIR    2    // GPIO2 - Motor A direction
#define MOTOR_B_PWM    5    // GPIO5 - Motor B speed
#define MOTOR_B_DIR    18   // GPIO18 - Motor B direction
#define MOTOR_C_PWM    6    // GPIO6 - Motor C speed
#define MOTOR_C_DIR    7    // GPIO7 - Motor C direction
#define MOTOR_D_PWM    15   // GPIO15 - Motor D speed
#define MOTOR_D_DIR    16   // GPIO16 - Motor D direction

#define ULTRASONIC_TRIG 10  // GPIO10 - HC-SR04 Trigger
#define ULTRASONIC_ECHO 9   // GPIO9 - HC-SR04 Echo

#define BUZZER_PIN     14   // GPIO14 - Piezo buzzer

#define ONEWIRE_PIN    11   // GPIO11 - DS18B20

// UART for Pi Zero communication
#define UART_TX_PIN    43
#define UART_RX_PIN    44
#define UART_NUM       UART_NUM_1
#define UART_BAUD      115200

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
// STATE MACHINE DEFINITIONS
// =============================================================================

enum RoverState {
    STATE_PATROL = 0,       // Normal patrol mode
    STATE_RESEARCH,         // Research/data collection
    STATE_ALERT,            // Hazard detected - investigating
    STATE_RETURN_TO_BASE,   // Low battery or manual recall
    STATE_DOCKING,          // AprilTag docking procedure
    STATE_DOCKED            // Successfully docked
};

enum AlertType {
    ALERT_NONE = 0,
    ALERT_FIRE,
    ALERT_EARTHQUAKE,
    ALERT_INTRUDER
};

// =============================================================================
// GLOBAL VARIABLES
// =============================================================================

// Current rover state
volatile RoverState currentState = STATE_PATROL;
volatile RoverState previousState = STATE_PATROL;

// Control inputs
volatile int8_t currentTurn = 0;
volatile uint8_t currentSpeed = 0;
volatile bool dockingEnabled = false;
volatile bool docked = false;

// Timing
volatile uint32_t lastCommandTime = 0;
volatile uint32_t lastEspNowTime = 0;
volatile uint32_t stateEntryTime = 0;

// Sensor data (updated by tasks)
volatile float temperature = 25.0;
volatile float distance = 999.0;  // cm
volatile float batteryVoltage = 12.0;
volatile float batteryPercent = 100.0;
volatile bool fireDetected = false;
volatile bool humanDetected = false;
volatile bool tiltDetected = false;
volatile float latitude = 0.0;
volatile float longitude = 0.0;

// Alert tracking
volatile AlertType currentAlert = ALERT_NONE;
volatile uint32_t alertStartTime = 0;
volatile bool alertConfirmed = false;

// Power management
volatile bool lowPowerMode = false;
const float LOW_BATTERY_THRESHOLD = 30.0;

// Safety
volatile bool estopTriggered = false;

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
        ledcWrite(0, map(speed, 0, 100, 0, 255));
    } else {
        digitalWrite(dirPin, LOW);   // Reverse
        ledcWrite(0, map(-speed, 0, 100, 0, 255));
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
// ULTRASONIC SENSOR - NON-BLOCKING
// =============================================================================

/**
 * Read ultrasonic distance (non-blocking)
 * Returns distance in cm, or 999.0 on timeout/error
 */
float readUltrasonic() {
    // Trigger pulse
    digitalWrite(ULTRASONIC_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIG, LOW);
    
    // Measure echo (with timeout)
    uint32_t timeout = 25000;  // ~4m max
    uint32_t start = micros();
    
    while (digitalRead(ULTRASONIC_ECHO) == LOW) {
        if (micros() - start > timeout) return 999.0;
    }
    
    uint32_t echoStart = micros();
    
    while (digitalRead(ULTRASONIC_ECHO) == HIGH) {
        if (micros() - echoStart > timeout) return 999.0;
    }
    
    uint32_t echoDuration = micros() - echoStart;
    
    // Convert to cm (speed of sound = 343 m/s)
    float distance = (echoDuration * 0.0343) / 2.0;
    
    return constrain(distance, 0, 400);
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
// BATTERY MONITORING
// =============================================================================

void updateBattery() {
    // Read ADC and calculate battery percentage
    // Assuming voltage divider and 12V battery
    int adcValue = analogRead(A0);
    batteryVoltage = (adcValue / 4095.0) * 3.3 * 4.2;  // Adjust divider ratio
    batteryPercent = ((batteryVoltage - 9.0) / 3.0) * 100.0;
    batteryPercent = constrain(batteryPercent, 0, 100);
    
    // Enable low power mode
    lowPowerMode = (batteryPercent < LOW_BATTERY_THRESHOLD);
}

// =============================================================================
// STATE MACHINE LOGIC
// =============================================================================

/**
 * Check transitions between states
 */
void updateStateMachine() {
    uint32_t currentTime = millis();
    previousState = currentState;
    
    switch (currentState) {
        case STATE_PATROL:
            // Transitions:
            if (fireDetected || humanDetected || tiltDetected) {
                currentState = STATE_ALERT;
                alertStartTime = currentTime;
                alertConfirmed = false;
            } else if (batteryPercent < LOW_BATTERY_THRESHOLD) {
                currentState = STATE_RETURN_TO_BASE;
            } else if (dockingEnabled) {
                currentState = STATE_DOCKING;
            }
            break;
            
        case STATE_RESEARCH:
            // Stay in research until timeout or alert
            if (fireDetected || humanDetected || tiltDetected) {
                currentState = STATE_ALERT;
                alertStartTime = currentTime;
            } else if (currentTime - stateEntryTime > 300000) {  // 5 min timeout
                currentState = STATE_PATROL;
            }
            break;
            
        case STATE_ALERT:
            // Sensor fusion: require BOTH temperature AND camera flag for fire
            if (currentAlert == ALERT_FIRE) {
                if (temperature > 60.0 && fireDetected) {
                    alertConfirmed = true;
                }
            }
            
            // Timeout after 30 seconds if not confirmed
            if (currentTime - alertStartTime > 30000) {
                if (alertConfirmed) {
                    // Send alert via SIM800
                    // Then return to patrol
                }
                currentState = STATE_PATROL;
            }
            break;
            
        case STATE_RETURN_TO_BASE:
            // Navigate toward base (using GPS)
            if (dockingEnabled) {
                currentState = STATE_DOCKING;
            }
            // Could add GPS waypoint navigation here
            break;
            
        case STATE_DOCKING:
            // Use commands from Pi Zero
            if (docked || (currentTime - lastCommandTime > COMMAND_TIMEOUT_MS)) {
                currentState = STATE_DOCKED;
                estop();
            }
            break;
            
        case STATE_DOCKED:
            // Stay docked, waiting
            if (!dockingEnabled) {
                currentState = STATE_PATROL;
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
 */
void executeState() {
    uint32_t currentTime = millis();
    
    switch (currentState) {
        case STATE_PATROL:
            // Normal patrol behavior
            // Could implement waypoint following or random walk
            currentTurn = 0;
            currentSpeed = lowPowerMode ? 30 : 50;
            break;
            
        case STATE_RESEARCH:
            // Slower, more thorough movement
            currentTurn = 0;
            currentSpeed = 20;
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
            break;
            
        case STATE_RETURN_TO_BASE:
            // Navigate toward base coordinates
            // Simplified: just go straight (would use GPS in production)
            currentTurn = 0;
            currentSpeed = lowPowerMode ? 20 : 40;
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
            
        case STATE_DOCKED:
            // Full stop
            currentTurn = 0;
            currentSpeed = 0;
            docked = true;
            break;
    }
    
    // Apply motor commands (except in DOCKING which uses UART commands directly)
    if (currentState != STATE_DOCKING) {
        drive(currentTurn, currentSpeed);
    }
}

// =============================================================================
// TASKS
// =============================================================================

/**
 * UART receive task - from Pi Zero
 */
void uartTask(void* parameter) {
    uint8_t buffer[1];
    DockCommand cmd;
    
    while (true) {
        int length = UART.read(UART_NUM, buffer, 1);
        if (length > 0) {
            if (parseDockCommand(buffer[0], &cmd)) {
                // Valid command received
                currentTurn = cmd.turn;
                currentSpeed = cmd.speed;
                docked = (cmd.docked == 1);
                lastCommandTime = millis();
                estopTriggered = false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/**
 * Sensor polling task
 */
void sensorTask(void* parameter) {
    while (true) {
        // Read ultrasonic (non-blocking handled internally)
        distance = readUltrasonic();
        
        // Obstacle avoidance override
        if (distance < 20.0 && currentSpeed > 20) {
            estop();
        }
        
        // Read temperature
        temperature = readTemperature();
        
        // Update battery
        updateBattery();
        
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
    
    // Configure PWM for motors
    ledcSetup(0, 1000, 8);  // Channel 0, 1kHz, 8-bit
    ledcAttachPin(MOTOR_A_PWM, 0);
    ledcSetup(1, 1000, 8);
    ledcAttachPin(MOTOR_B_PWM, 1);
    ledcSetup(2, 1000, 8);
    ledcAttachPin(MOTOR_C_PWM, 2);
    ledcSetup(3, 1000, 8);
    ledcAttachPin(MOTOR_D_PWM, 3);
    
    // Configure ultrasonic
    pinMode(ULTRASONIC_TRIG, OUTPUT);
    pinMode(ULTRASONIC_ECHO, INPUT);
    
    // Configure buzzer
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    
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
    
    // Initialize watchdog
    // In production: esp_task_wdt_init(5, true);
    
    // Create tasks
    xTaskCreatePinnedToCore(uartTask, "UART", 2048, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(sensorTask, "Sensors", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(stateTask, "State", 2048, NULL, 1, NULL, 1);
    
    lastCommandTime = millis();
    stateEntryTime = millis();
    
    Serial.println("STASIS Rover Ready");
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

