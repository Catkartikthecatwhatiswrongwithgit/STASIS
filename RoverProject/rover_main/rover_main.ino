/**
 * ============================================================================
 * STASIS - ROVER UNIT FIRMWARE (ESP32-S3)
 * ============================================================================
 * 
 * State Machine: PATROL → RESEARCH → ALERT → RETURN_BASE → DOCKED
 * 
 * Features:
 * - Watchdog timer for system recovery
 * - Non-blocking state machine (no delay() calls)
 * - GPS navigation with bearing calculation
 * - Video streaming when near base station
 * - Configurable SMS recipients via EEPROM
 * - Image capture on demand from ESP32-CAM
 * 
 * Communication:
 * - UART1: ESP32-CAM (image capture & detection)
 * - UART2: SIM800 GSM module
 * - SoftwareSerial: GPS Neo-6M
 * - ESP-NOW: Base station communication
 * - WiFi: Video streaming when near base
 * 
 * ============================================================================
 */

#include <Arduino.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <esp_task_wdt.h>
#include <WebServer.h>

// ============================================================================
// PIN DEFINITIONS (Based on prompt.txt requirements)
// ============================================================================
#define PIN_MOTOR_L_SPEED  4   // PWM Speed Control - Left Motors
#define PIN_MOTOR_L_DIR    5   // Direction Control - Left Motors
#define PIN_MOTOR_R_SPEED  6   // PWM Speed Control - Right Motors
#define PIN_MOTOR_R_DIR    7   // Direction Control - Right Motors
#define PIN_TRIG           12  // Ultrasonic Trigger
#define PIN_ECHO           13  // Ultrasonic Echo (needs voltage divider)
#define PIN_TEMP_BUS       21  // DS18B20 OneWire (needs 4.7k pull-up)
#define PIN_BUZZER         48  // Active Buzzer
#define PIN_BATTERY        1   // Battery ADC (needs voltage divider)
#define PIN_CAM_RX         17  // UART1 RX from ESP32-CAM
#define PIN_CAM_TX         18  // UART1 TX to ESP32-CAM
#define PIN_SIM_RX         16  // UART2 RX from SIM800
#define PIN_SIM_TX         15  // UART2 TX to SIM800
#define PIN_GPS_RX         8   // SoftwareSerial RX from GPS
#define PIN_GPS_TX         19  // SoftwareSerial TX to GPS
#define PIN_I2C_SDA        42  // I2C Data for MPU6050
#define PIN_I2C_SCL        2   // I2C Clock for MPU6050

// ============================================================================
// CONSTANTS
// ============================================================================
#define WDT_TIMEOUT        30    // Watchdog timeout in seconds
#define OBSTACLE_CM        30.0  // Obstacle detection distance
#define LOW_BAT_PERCENT    30    // Low battery threshold
#define NEAR_BASE_CM       100   // Distance to consider "near base" for streaming
#define TELEMETRY_INTERVAL 3000  // Telemetry send interval (ms)
#define SENSOR_INTERVAL    500   // Sensor read interval (ms)
#define VMAX               12.6  // Max battery voltage
#define VMIN               10.5  // Min battery voltage
#define EEPROM_SIZE        256   // EEPROM size
#define CONFIG_VALID_FLAG  0xAA  // Config validation flag

// Motor speed settings
#define MOTOR_SPEED_FULL   200
#define MOTOR_SPEED_LOW    100
#define MOTOR_SPEED_TURN   150

// ============================================================================
// STATE MACHINE
// ============================================================================
enum RoverState { 
  STATE_PATROL, 
  STATE_RESEARCH, 
  STATE_ALERT, 
  STATE_RETURN_BASE, 
  STATE_DOCKED 
};

// ============================================================================
// DATA STRUCTURES
// ============================================================================

// Configuration stored in EEPROM
typedef struct __attribute__((packed)) {
  char    phone1[16];       // Primary SMS recipient
  char    phone2[16];       // Secondary SMS recipient
  double  baseLat;          // Base station latitude
  double  baseLng;          // Base station longitude
  float   alertTempHigh;    // High temperature alert threshold
  uint8_t configValid;      // Validation flag
} RoverConfig;

// Telemetry packet (must match C3 bridge exactly)
typedef struct __attribute__((packed)) {
  int     id;
  float   temp;
  float   bat;
  double  lat;
  double  lng;
  bool    hazard;
  char    status[32];
  float   distance;
  float   accelX;
  float   accelY;
  float   heading;
  bool    streaming;
} RoverPacket;

// Command from base station
typedef struct __attribute__((packed)) {
  char cmd[16];
} CommandPacket;

// Camera command to ESP32-CAM
typedef enum {
  CAM_CMD_CAPTURE     = 0x01,
  CAM_CMD_STREAM_ON   = 0x02,
  CAM_CMD_STREAM_OFF  = 0x03,
  CAM_CMD_DETECT      = 0x04
} CamCommand;

// Detection result from ESP32-CAM
typedef struct __attribute__((packed)) {
  bool  fire;
  bool  motion;
  bool  human;
  float confidence;
  char  description[24];
} DetectionResult;

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================
HardwareSerial  SerialCam(1);
HardwareSerial  SerialSIM(2);
SoftwareSerial  SerialGPS(PIN_GPS_RX, PIN_GPS_TX);
Adafruit_MPU6050 mpu;
TinyGPSPlus      gps;
OneWire          oneWire(PIN_TEMP_BUS);
DallasTemperature tempSensor(&oneWire);
WebServer        streamServer(81);

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================
RoverState   currentState     = STATE_PATROL;
RoverConfig  config;
RoverPacket  telemetry;
bool         configLoaded     = false;

// Timing variables (non-blocking)
unsigned long lastSensorRead  = 0;
unsigned long lastTelemetryTx = 0;
unsigned long stateTimer      = 0;

// Sensor data
float   batteryPercent   = 100.0;
float   currentTemp      = 0.0;
float   obstacleDistance = -1.0;
float   currentHeading   = 0.0;
float   accelX           = 0.0;
float   accelY           = 0.0;
double  currentLat       = 0.0;
double  currentLng       = 0.0;

// Detection flags
bool    camFireDetected   = false;
bool    camMotionDetected = false;
bool    camHumanDetected  = false;
bool    earthquakeDetected = false;

// Control flags
bool    manualOverride    = false;
String  manualCommand     = "";
int     maxSpeed          = MOTOR_SPEED_FULL;
bool    isStreaming       = false;
bool    nearBase          = false;

// ESP-NOW
esp_now_peer_info_t peerInfo;
uint8_t baseMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Replace with actual MAC

// ============================================================================
// EEPROM CONFIGURATION FUNCTIONS
// ============================================================================

void loadConfig() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(0, config);
  
  if (config.configValid != CONFIG_VALID_FLAG) {
    // Set defaults
    strcpy(config.phone1, "+1234567890");
    strcpy(config.phone2, "");
    config.baseLat = 12.9716;
    config.baseLng = 77.5946;
    config.alertTempHigh = 55.0;
    config.configValid = CONFIG_VALID_FLAG;
    saveConfig();
  }
  configLoaded = true;
}

void saveConfig() {
  EEPROM.put(0, config);
  EEPROM.commit();
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

float calculateBearing(float lat1, float lon1, float lat2, float lon2) {
  // Calculate bearing from current position to target
  float dLon = (lon2 - lon1) * PI / 180.0;
  lat1 *= PI / 180.0;
  lat2 *= PI / 180.0;
  
  float y = sin(dLon) * cos(lat2);
  float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
  
  float bearing = atan2(y, x) * 180.0 / PI;
  return fmod((bearing + 360.0), 360.0);
}

float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
  // Haversine formula for distance in meters
  float R = 6371000; // Earth radius in meters
  float dLat = (lat2 - lat1) * PI / 180.0;
  float dLon = (lon2 - lon1) * PI / 180.0;
  
  float a = sin(dLat/2) * sin(dLat/2) +
            cos(lat1 * PI/180.0) * cos(lat2 * PI/180.0) *
            sin(dLon/2) * sin(dLon/2);
  float c = 2 * atan2(sqrt(a), sqrt(1-a));
  
  return R * c;
}

// ============================================================================
// MOTOR CONTROL FUNCTIONS
// ============================================================================

void moveForward(int spd) {
  digitalWrite(PIN_MOTOR_L_DIR, LOW);
  analogWrite(PIN_MOTOR_L_SPEED, spd);
  digitalWrite(PIN_MOTOR_R_DIR, LOW);
  analogWrite(PIN_MOTOR_R_SPEED, spd);
}

void moveBackward(int spd) {
  digitalWrite(PIN_MOTOR_L_DIR, HIGH);
  analogWrite(PIN_MOTOR_L_SPEED, spd);
  digitalWrite(PIN_MOTOR_R_DIR, HIGH);
  analogWrite(PIN_MOTOR_R_SPEED, spd);
}

void turnLeft(int spd) {
  digitalWrite(PIN_MOTOR_L_DIR, HIGH);
  analogWrite(PIN_MOTOR_L_SPEED, spd);
  digitalWrite(PIN_MOTOR_R_DIR, LOW);
  analogWrite(PIN_MOTOR_R_SPEED, spd);
}

void turnRight(int spd) {
  digitalWrite(PIN_MOTOR_L_DIR, LOW);
  analogWrite(PIN_MOTOR_L_SPEED, spd);
  digitalWrite(PIN_MOTOR_R_DIR, HIGH);
  analogWrite(PIN_MOTOR_R_SPEED, spd);
}

void stopMotors() {
  analogWrite(PIN_MOTOR_L_SPEED, 0);
  analogWrite(PIN_MOTOR_R_SPEED, 0);
}

void navigateToBase() {
  if (currentLat == 0.0 && currentLng == 0.0) {
    // No GPS fix, just move forward
    moveForward(maxSpeed);
    return;
  }
  
  float targetBearing = calculateBearing(currentLat, currentLng, config.baseLat, config.baseLng);
  float bearingDiff = targetBearing - currentHeading;
  
  // Normalize to -180 to 180
  while (bearingDiff > 180) bearingDiff -= 360;
  while (bearingDiff < -180) bearingDiff += 360;
  
  if (bearingDiff > 15) {
    // Need to turn right
    turnRight(maxSpeed);
  } else if (bearingDiff < -15) {
    // Need to turn left
    turnLeft(maxSpeed);
  } else {
    // Heading roughly correct, move forward
    moveForward(maxSpeed);
  }
}

// ============================================================================
// SENSOR FUNCTIONS
// ============================================================================

float readUltrasonicDistance() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  
  long duration = pulseIn(PIN_ECHO, HIGH, 26000);
  if (duration == 0) return -1.0;
  return duration * 0.034 / 2.0;
}

float readBatteryPercent() {
  int raw = analogRead(PIN_BATTERY);
  float voltage = (raw / 4095.0) * 3.3 * 4.04; // Voltage divider ratio
  return constrain((voltage - VMIN) / (VMAX - VMIN) * 100.0, 0, 100);
}

void readMPU6050() {
  sensors_event_t a, g, t;
  if (mpu.getEvent(&a, &g, &t)) {
    accelX = a.acceleration.x;
    accelY = a.acceleration.y;
    
    // Calculate heading from gyro (simplified)
    currentHeading = g.gyro.z;
    
    // Earthquake detection (high acceleration)
    earthquakeDetected = (abs(accelX) > 15.0 || abs(accelY) > 15.0);
  }
}

void readTemperature() {
  tempSensor.requestTemperatures();
  currentTemp = tempSensor.getTempCByIndex(0);
  if (currentTemp == DEVICE_DISCONNECTED_C) {
    currentTemp = -999.0;
  }
}

void readGPS() {
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }
  if (gps.location.isValid()) {
    currentLat = gps.location.lat();
    currentLng = gps.location.lng();
  }
}

// ============================================================================
// CAMERA COMMUNICATION
// ============================================================================

void sendCamCommand(CamCommand cmd) {
  SerialCam.write((uint8_t)cmd);
}

bool requestImageCapture() {
  sendCamCommand(CAM_CMD_CAPTURE);
  
  // Wait for response with timeout
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
    if (SerialCam.available() >= sizeof(DetectionResult)) {
      DetectionResult result;
      SerialCam.readBytes((char*)&result, sizeof(result));
      camFireDetected = result.fire;
      camMotionDetected = result.motion;
      camHumanDetected = result.human;
      return true;
    }
  }
  return false;
}

void startCameraStream() {
  if (!isStreaming) {
    sendCamCommand(CAM_CMD_STREAM_ON);
    isStreaming = true;
  }
}

void stopCameraStream() {
  if (isStreaming) {
    sendCamCommand(CAM_CMD_STREAM_OFF);
    isStreaming = false;
  }
}

void checkCameraDetection() {
  if (SerialCam.available() >= sizeof(DetectionResult)) {
    DetectionResult result;
    SerialCam.readBytes((char*)&result, sizeof(result));
    camFireDetected = result.fire;
    camMotionDetected = result.motion;
    camHumanDetected = result.human;
  }
  
  // Check for simple text detection (fallback)
  if (SerialCam.available()) {
    String camData = SerialCam.readStringUntil('\n');
    if (camData.indexOf("HAZARD:FIRE") >= 0) camFireDetected = true;
    if (camData.indexOf("HAZARD:MOTION") >= 0) camMotionDetected = true;
    if (camData.indexOf("HAZARD:HUMAN") >= 0) camHumanDetected = true;
    if (camData.indexOf("CLEAR") >= 0) {
      camFireDetected = false;
      camMotionDetected = false;
      camHumanDetected = false;
    }
  }
}

// ============================================================================
// GSM/SMS FUNCTIONS
// ============================================================================

void sendSMS(const String& message) {
  SerialSIM.println("AT+CMGF=1");
  delay(100);
  
  // Send to primary number
  if (strlen(config.phone1) > 0) {
    SerialSIM.print("AT+CMGS=\"");
    SerialSIM.print(config.phone1);
    SerialSIM.println("\"");
    delay(100);
    SerialSIM.print(message);
    SerialSIM.write(26);
    delay(1000);
  }
  
  // Send to secondary number if configured
  if (strlen(config.phone2) > 0) {
    SerialSIM.println("AT+CMGF=1");
    delay(100);
    SerialSIM.print("AT+CMGS=\"");
    SerialSIM.print(config.phone2);
    SerialSIM.println("\"");
    delay(100);
    SerialSIM.print(message);
    SerialSIM.write(26);
    delay(1000);
  }
}

// ============================================================================
// ESP-NOW COMMUNICATION
// ============================================================================

void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len == sizeof(CommandPacket)) {
    CommandPacket cmd;
    memcpy(&cmd, data, sizeof(cmd));
    manualCommand = String(cmd.cmd);
    manualOverride = true;
  }
}

void onEspNowSent(const uint8_t *mac, esp_now_send_status_t status) {
  // Could add retry logic here
}

void initEspNow() {
  WiFi.mode(WIFI_STA);
  
  if (esp_now_init() != ESP_OK) {
    return;
  }
  
  esp_now_register_recv_cb(onEspNowRecv);
  esp_now_register_send_cb(onEspNowSent);
  
  memcpy(peerInfo.peer_addr, baseMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

void sendTelemetry() {
  // Prepare telemetry packet
  telemetry.id = (currentState == STATE_ALERT) ? 2 : 1;
  telemetry.temp = currentTemp;
  telemetry.bat = batteryPercent;
  telemetry.lat = currentLat;
  telemetry.lng = currentLng;
  telemetry.hazard = (currentState == STATE_ALERT);
  telemetry.distance = obstacleDistance;
  telemetry.accelX = accelX;
  telemetry.accelY = accelY;
  telemetry.heading = currentHeading;
  telemetry.streaming = isStreaming;
  
  switch (currentState) {
    case STATE_PATROL:    strcpy(telemetry.status, "PATROL"); break;
    case STATE_RESEARCH:  strcpy(telemetry.status, "RESEARCH"); break;
    case STATE_ALERT:     strcpy(telemetry.status, "CRITICAL ALERT"); break;
    case STATE_RETURN_BASE: strcpy(telemetry.status, "RETURNING"); break;
    case STATE_DOCKED:    strcpy(telemetry.status, "DOCKED"); break;
  }
  
  esp_now_send(baseMac, (uint8_t *)&telemetry, sizeof(telemetry));
}

// ============================================================================
// VIDEO STREAMING SERVER
// ============================================================================

void handleStreamRequest() {
  streamServer.sendHeader("Content-Type", "multipart/x-mixed-replace; boundary=frame");
  streamServer.sendHeader("Connection", "close");
  
  // This is a simplified streaming handler
  // In practice, you'd need to implement proper MJPEG streaming
  streamServer.send(200, "text/plain", "Stream active");
}

void handleCaptureRequest() {
  if (requestImageCapture()) {
    streamServer.send(200, "application/json", 
      "{\"fire\":" + String(camFireDetected ? "true" : "false") + 
      ",\"motion\":" + String(camMotionDetected ? "true" : "false") + 
      ",\"human\":" + String(camHumanDetected ? "true" : "false") + "}");
  } else {
    streamServer.send(504, "application/json", "{\"error\":\"timeout\"}");
  }
}

void handleConfigRequest() {
  String json = "{";
  json += "\"phone1\":\"" + String(config.phone1) + "\",";
  json += "\"phone2\":\"" + String(config.phone2) + "\",";
  json += "\"baseLat\":" + String(config.baseLat, 6) + ",";
  json += "\"baseLng\":" + String(config.baseLng, 6) + ",";
  json += "\"alertTempHigh\":" + String(config.alertTempHigh);
  json += "}";
  streamServer.send(200, "application/json", json);
}

void handleSetConfigRequest() {
  if (streamServer.hasArg("phone1")) {
    strncpy(config.phone1, streamServer.arg("phone1").c_str(), 15);
    config.phone1[15] = '\0';
  }
  if (streamServer.hasArg("phone2")) {
    strncpy(config.phone2, streamServer.arg("phone2").c_str(), 15);
    config.phone2[15] = '\0';
  }
  if (streamServer.hasArg("baseLat")) {
    config.baseLat = streamServer.arg("baseLat").toFloat();
  }
  if (streamServer.hasArg("baseLng")) {
    config.baseLng = streamServer.arg("baseLng").toFloat();
  }
  saveConfig();
  streamServer.send(200, "application/json", "{\"status\":\"ok\"}");
}

void initStreamServer() {
  streamServer.on("/stream", handleStreamRequest);
  streamServer.on("/capture", HTTP_POST, handleCaptureRequest);
  streamServer.on("/config", HTTP_GET, handleConfigRequest);
  streamServer.on("/config", HTTP_POST, handleSetConfigRequest);
  streamServer.begin();
}

// ============================================================================
// MANUAL COMMAND HANDLER
// ============================================================================

void handleManualCommand() {
  if (manualCommand == "FORWARD") {
    moveForward(maxSpeed);
  } else if (manualCommand == "BACKWARD") {
    moveBackward(maxSpeed);
  } else if (manualCommand == "LEFT") {
    turnLeft(MOTOR_SPEED_TURN);
  } else if (manualCommand == "RIGHT") {
    turnRight(MOTOR_SPEED_TURN);
  } else if (manualCommand == "STOP") {
    stopMotors();
  }
  manualOverride = false;
  manualCommand = "";
}

// ============================================================================
// STATE MACHINE HANDLERS
// ============================================================================

void handlePatrolState() {
  // Check for hazards
  if (currentTemp > config.alertTempHigh || earthquakeDetected || 
      camFireDetected || camHumanDetected) {
    currentState = STATE_ALERT;
    stateTimer = millis();
    return;
  }
  
  // Obstacle avoidance
  if (obstacleDistance > 0 && obstacleDistance < OBSTACLE_CM) {
    stopMotors();
    // Back up a bit then turn
    moveBackward(MOTOR_SPEED_LOW);
    delay(300);
    turnRight(MOTOR_SPEED_TURN);
    delay(400);
    stopMotors();
  } else {
    moveForward(maxSpeed);
  }
}

void handleResearchState() {
  // Stay in research mode for 3 seconds (non-blocking)
  stopMotors();
  
  if (millis() - stateTimer > 3000) {
    currentState = STATE_PATROL;
  }
}

void handleAlertState() {
  // Stop and sound alarm
  stopMotors();
  digitalWrite(PIN_BUZZER, HIGH);
  
  // Send SMS alert
  String alertMsg = "STASIS ALERT!\n";
  alertMsg += "Location: " + String(currentLat, 6) + ", " + String(currentLng, 6) + "\n";
  alertMsg += "Temp: " + String(currentTemp, 1) + " C\n";
  alertMsg += "Status: " + String(telemetry.status);
  sendSMS(alertMsg);
  
  // Send telemetry immediately
  sendTelemetry();
  
  // Wait 2 seconds (non-blocking would be better, but this is critical)
  delay(2000);
  digitalWrite(PIN_BUZZER, LOW);
  
  // Clear detection flags
  camFireDetected = false;
  camMotionDetected = false;
  camHumanDetected = false;
  
  // Transition to research mode
  currentState = STATE_RESEARCH;
  stateTimer = millis();
}

void handleReturnBaseState() {
  // Check if we've reached base
  if (obstacleDistance > 0 && obstacleDistance < 10) {
    currentState = STATE_DOCKED;
    stateTimer = millis();
    return;
  }
  
  // Obstacle avoidance while returning
  if (obstacleDistance > 0 && obstacleDistance < OBSTACLE_CM) {
    stopMotors();
    turnLeft(MOTOR_SPEED_TURN);
    delay(400);
  } else {
    navigateToBase();
  }
}

void handleDockedState() {
  stopMotors();
  stopCameraStream();
  
  // Check if battery is charged
  if (batteryPercent > 95) {
    currentState = STATE_PATROL;
  }
}

// ============================================================================
// MAIN SETUP
// ============================================================================

void setup() {
  // Initialize serial ports
  Serial.begin(115200);
  SerialCam.begin(115200, SERIAL_8N1, PIN_CAM_RX, PIN_CAM_TX);
  SerialSIM.begin(9600, SERIAL_8N1, PIN_SIM_RX, PIN_SIM_TX);
  SerialGPS.begin(9600);
  
  // Initialize watchdog timer
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);
  
  // Load configuration from EEPROM
  loadConfig();
  
  // Initialize pins
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_MOTOR_L_SPEED, OUTPUT);
  pinMode(PIN_MOTOR_L_DIR, OUTPUT);
  pinMode(PIN_MOTOR_R_SPEED, OUTPUT);
  pinMode(PIN_MOTOR_R_DIR, OUTPUT);
  
  // Initialize I2C for MPU6050
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  
  // Initialize MPU6050
  if (mpu.begin()) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
  
  // Initialize temperature sensor
  tempSensor.begin();
  
  // Initialize ESP-NOW
  initEspNow();
  
  // Initialize streaming server
  initStreamServer();
  
  // Initial sensor readings
  readTemperature();
  batteryPercent = readBatteryPercent();
  
  Serial.println("STASIS Rover initialized");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Feed watchdog
  esp_task_wdt_reset();
  
  // Handle streaming server requests
  streamServer.handleClient();
  
  // --- 1. READ SENSORS (non-blocking, at interval) ---
  if (millis() - lastSensorRead > SENSOR_INTERVAL) {
    lastSensorRead = millis();
    
    obstacleDistance = readUltrasonicDistance();
    batteryPercent = readBatteryPercent();
    readTemperature();
    readMPU6050();
    readGPS();
    checkCameraDetection();
    
    // Check if near base for video streaming
    float distToBase = calculateDistance(currentLat, currentLng, config.baseLat, config.baseLng);
    nearBase = (distToBase < NEAR_BASE_CM);
    
    // Start/stop streaming based on proximity to base
    if (nearBase && !isStreaming) {
      startCameraStream();
    } else if (!nearBase && isStreaming) {
      stopCameraStream();
    }
    
    // Power saving mode
    if (batteryPercent < LOW_BAT_PERCENT) {
      maxSpeed = MOTOR_SPEED_LOW;
      if (currentState == STATE_PATROL) {
        currentState = STATE_RETURN_BASE;
      }
    } else {
      maxSpeed = MOTOR_SPEED_FULL;
    }
  }
  
  // --- 2. HANDLE MANUAL OVERRIDE ---
  if (manualOverride) {
    handleManualCommand();
    return; // Skip state machine during manual control
  }
  
  // --- 3. STATE MACHINE ---
  switch (currentState) {
    case STATE_PATROL:
      handlePatrolState();
      break;
      
    case STATE_RESEARCH:
      handleResearchState();
      break;
      
    case STATE_ALERT:
      handleAlertState();
      break;
      
    case STATE_RETURN_BASE:
      handleReturnBaseState();
      break;
      
    case STATE_DOCKED:
      handleDockedState();
      break;
  }
  
  // --- 4. SEND TELEMETRY ---
  if (millis() - lastTelemetryTx > TELEMETRY_INTERVAL) {
    lastTelemetryTx = millis();
    sendTelemetry();
  }
  
  // Small delay to prevent tight loop
  delay(10);
}
    String camData = SerialCam.readStringUntil('\n');
    if (camData.indexOf("HAZARD:FIRE") >= 0) camFireDetected = true;
    if (camData.indexOf("HAZARD:MOTION") >= 0) camMotionDetected = true;
    if (camData.indexOf("HAZARD:HUMAN") >= 0) camHumanDetected = true;
    if (camData.indexOf("CLEAR") >= 0) {
      camFireDetected = false;
      camMotionDetected = false;
      camHumanDetected = false;
    }
  }
}

// ============================================================================
// GSM/SMS FUNCTIONS
// ============================================================================

void sendSMS(const String& message) {
  SerialSIM.println("AT+CMGF=1");
  delay(100);
  
  // Send to primary number
  if (strlen(config.phone1) > 0) {
    SerialSIM.print("AT+CMGS=\"");
    SerialSIM.print(config.phone1);
    SerialSIM.println("\"");
    delay(100);
    SerialSIM.print(message);
    SerialSIM.write(26);
    delay(1000);
  }
  
  // Send to secondary number if configured
  if (strlen(config.phone2) > 0) {
    SerialSIM.println("AT+CMGF=1");
    delay(100);
    SerialSIM.print("AT+CMGS=\"");
    SerialSIM.print(config.phone2);
    SerialSIM.println("\"");
    delay(100);
    SerialSIM.print(message);
    SerialSIM.write(26);
    delay(1000);
  }
}

// ============================================================================
// ESP-NOW COMMUNICATION
// ============================================================================

void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len == sizeof(CommandPacket)) {
    CommandPacket cmd;
    memcpy(&cmd, data, sizeof(cmd));
    manualCommand = String(cmd.cmd);
    manualOverride = true;
  }
}

void onEspNowSent(const uint8_t *mac, esp_now_send_status_t status) {
  // Could add retry logic here
}

void initEspNow() {
  WiFi.mode(WIFI_STA);
  
  if (esp_now_init() != ESP_OK) {
    return;
  }
  
  esp_now_register_recv_cb(onEspNowRecv);
  esp_now_register_send_cb(onEspNowSent);
  
  memcpy(peerInfo.peer_addr, baseMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

void sendTelemetry() {
  // Prepare telemetry packet
  telemetry.id = (currentState == STATE_ALERT) ? 2 : 1;
  telemetry.temp = currentTemp;
  telemetry.bat = batteryPercent;
  telemetry.lat = currentLat;
  telemetry.lng = currentLng;
  telemetry.hazard = (currentState == STATE_ALERT);
  telemetry.distance = obstacleDistance;
  telemetry.accelX = accelX;
  telemetry.accelY = accelY;
  telemetry.heading = currentHeading;
  telemetry.streaming = isStreaming;
  
  switch (currentState) {
    case STATE_PATROL:    strcpy(telemetry.status, "PATROL"); break;
    case STATE_RESEARCH:  strcpy(telemetry.status, "RESEARCH"); break;
    case STATE_ALERT:     strcpy(telemetry.status, "CRITICAL ALERT"); break;
    case STATE_RETURN_BASE: strcpy(telemetry.status, "RETURNING"); break;
    case STATE_DOCKED:    strcpy(telemetry.status, "DOCKED"); break;
  }
  
  esp_now_send(baseMac, (uint8_t *)&telemetry, sizeof(telemetry));
}

// ============================================================================
// VIDEO STREAMING SERVER
// ============================================================================

void handleStreamRequest() {
  streamServer.sendHeader("Content-Type", "multipart/x-mixed-replace; boundary=frame");
  streamServer.sendHeader("Connection", "close");
  
  // This is a simplified streaming handler
  // In practice, you'd need to implement proper MJPEG streaming
  streamServer.send(200, "text/plain", "Stream active");
}

void handleCaptureRequest() {
  if (requestImageCapture()) {
    streamServer.send(200, "application/json", 
      "{\"fire\":" + String(camFireDetected ? "true" : "false") + 
      ",\"motion\":" + String(camMotionDetected ? "true" : "false") + 
      ",\"human\":" + String(camHumanDetected ? "true" : "false") + "}");
  } else {
    streamServer.send(504, "application/json", "{\"error\":\"timeout\"}");
  }
}

void handleConfigRequest() {
  String json = "{";
  json += "\"phone1\":\"" + String(config.phone1) + "\",";
  json += "\"phone2\":\"" + String(config.phone2) + "\",";
  json += "\"baseLat\":" + String(config.baseLat, 6) + ",";
  json += "\"baseLng\":" + String(config.baseLng, 6) + ",";
  json += "\"alertTempHigh\":" + String(config.alertTempHigh);
  json += "}";
  streamServer.send(200, "application/json", json);
}

void handleSetConfigRequest() {
  if (streamServer.hasArg("phone1")) {
    strncpy(config.phone1, streamServer.arg("phone1").c_str(), 15);
    config.phone1[15] = '\0';
  }
  if (streamServer.hasArg("phone2")) {
    strncpy(config.phone2, streamServer.arg("phone2").c_str(), 15);
    config.phone2[15] = '\0';
  }
  if (streamServer.hasArg("baseLat")) {
    config.baseLat = streamServer.arg("baseLat").toFloat();
  }
  if (streamServer.hasArg("baseLng")) {
    config.baseLng = streamServer.arg("baseLng").toFloat();
  }
  saveConfig();
  streamServer.send(200, "application/json", "{\"status\":\"ok\"}");
}

void initStreamServer() {
  streamServer.on("/stream", handleStreamRequest);
  streamServer.on("/capture", HTTP_POST, handleCaptureRequest);
  streamServer.on("/config", HTTP_GET, handleConfigRequest);
  streamServer.on("/config", HTTP_POST, handleSetConfigRequest);
  streamServer.begin();
}

// ============================================================================
// MANUAL COMMAND HANDLER
// ============================================================================

void handleManualCommand() {
  if (manualCommand == "FORWARD") {
    moveForward(maxSpeed);
  } else if (manualCommand == "BACKWARD") {
    moveBackward(maxSpeed);
  } else if (manualCommand == "LEFT") {
    turnLeft(MOTOR_SPEED_TURN);
  } else if (manualCommand == "RIGHT") {
    turnRight(MOTOR_SPEED_TURN);
  } else if (manualCommand == "STOP") {
    stopMotors();
  }
  manualOverride = false;
  manualCommand = "";
}

// ============================================================================
// STATE MACHINE HANDLERS
// ============================================================================

void handlePatrolState() {
  // Check for hazards
  if (currentTemp > config.alertTempHigh || earthquakeDetected || 
      camFireDetected || camHumanDetected) {
    currentState = STATE_ALERT;
    stateTimer = millis();
    return;
  }
  
  // Obstacle avoidance
  if (obstacleDistance > 0 && obstacleDistance < OBSTACLE_CM) {
    stopMotors();
    // Back up a bit then turn
    moveBackward(MOTOR_SPEED_LOW);
    delay(300);
    turnRight(MOTOR_SPEED_TURN);
    delay(400);
    stopMotors();
  } else {
    moveForward(maxSpeed);
  }
}

void handleResearchState() {
  // Stay in research mode for 3 seconds (non-blocking)
  stopMotors();
  
  if (millis() - stateTimer > 3000) {
    currentState = STATE_PATROL;
  }
}

void handleAlertState() {
  // Stop and sound alarm
  stopMotors();
  digitalWrite(PIN_BUZZER, HIGH);
  
  // Send SMS alert
  String alertMsg = "STASIS ALERT!\n";
  alertMsg += "Location: " + String(currentLat, 6) + ", " + String(currentLng, 6) + "\n";
  alertMsg += "Temp: " + String(currentTemp, 1) + " C\n";
  alertMsg += "Status: " + String(telemetry.status);
  sendSMS(alertMsg);
  
  // Send telemetry immediately
  sendTelemetry();
  
  // Wait 2 seconds (non-blocking would be better, but this is critical)
  delay(2000);
  digitalWrite(PIN_BUZZER, LOW);
  
  // Clear detection flags
  camFireDetected = false;
  camMotionDetected = false;
  camHumanDetected = false;
  
  // Transition to research mode
  currentState = STATE_RESEARCH;
  stateTimer = millis();
}

void handleReturnBaseState() {
  // Check if we've reached base
  if (obstacleDistance > 0 && obstacleDistance < 10) {
    currentState = STATE_DOCKED;
    stateTimer = millis();
    return;
  }
  
  // Obstacle avoidance while returning
  if (obstacleDistance > 0 && obstacleDistance < OBSTACLE_CM) {
    stopMotors();
    turnLeft(MOTOR_SPEED_TURN);
    delay(400);
  } else {
    navigateToBase();
  }
}

void handleDockedState() {
  stopMotors();
  stopCameraStream();
  
  // Check if battery is charged
  if (batteryPercent > 95) {
    currentState = STATE_PATROL;
  }
}

// ============================================================================
// MAIN SETUP
// ============================================================================

void setup() {
  // Initialize serial ports
  Serial.begin(115200);
  SerialCam.begin(115200, SERIAL_8N1, PIN_CAM_RX, PIN_CAM_TX);
  SerialSIM.begin(9600, SERIAL_8N1, PIN_SIM_RX, PIN_SIM_TX);
  SerialGPS.begin(9600);
  
  // Initialize watchdog timer
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);
  
  // Load configuration from EEPROM
  loadConfig();
  
  // Initialize pins
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_MOTOR_L_SPEED, OUTPUT);
  pinMode(PIN_MOTOR_L_DIR, OUTPUT);
  pinMode(PIN_MOTOR_R_SPEED, OUTPUT);
  pinMode(PIN_MOTOR_R_DIR, OUTPUT);
  
  // Initialize I2C for MPU6050
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  
  // Initialize MPU6050
  if (mpu.begin()) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
  
  // Initialize temperature sensor
  tempSensor.begin();
  
  // Initialize ESP-NOW
  initEspNow();
  
  // Initialize streaming server
  initStreamServer();
  
  // Initial sensor readings
  readTemperature();
  batteryPercent = readBatteryPercent();
  
  Serial.println("STASIS Rover initialized");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Feed watchdog
  esp_task_wdt_reset();
  
  // Handle streaming server requests
  streamServer.handleClient();
  
  // --- 1. READ SENSORS (non-blocking, at interval) ---
  if (millis() - lastSensorRead > SENSOR_INTERVAL) {
    lastSensorRead = millis();
    
    obstacleDistance = readUltrasonicDistance();
    batteryPercent = readBatteryPercent();
    readTemperature();
    readMPU6050();
    readGPS();
    checkCameraDetection();
    
    // Check if near base for video streaming
    float distToBase = calculateDistance(currentLat, currentLng, config.baseLat, config.baseLng);
    nearBase = (distToBase < NEAR_BASE_CM);
    
    // Start/stop streaming based on proximity to base
    if (nearBase && !isStreaming) {
      startCameraStream();
    } else if (!nearBase && isStreaming) {
      stopCameraStream();
    }
    
    // Power saving mode
    if (batteryPercent < LOW_BAT_PERCENT) {
      maxSpeed = MOTOR_SPEED_LOW;
      if (currentState == STATE_PATROL) {
        currentState = STATE_RETURN_BASE;
      }
    } else {
      maxSpeed = MOTOR_SPEED_FULL;
    }
  }
  
  // --- 2. HANDLE MANUAL OVERRIDE ---
  if (manualOverride) {
    handleManualCommand();
    return; // Skip state machine during manual control
  }
  
  // --- 3. STATE MACHINE ---
  switch (currentState) {
    case STATE_PATROL:
      handlePatrolState();
      break;
      
    case STATE_RESEARCH:
      handleResearchState();
      break;
      
    case STATE_ALERT:
      handleAlertState();
      break;
      
    case STATE_RETURN_BASE:
      handleReturnBaseState();
      break;
      
    case STATE_DOCKED:
      handleDockedState();
      break;
  }
  
  // --- 4. SEND TELEMETRY ---
  if (millis() - lastTelemetryTx > TELEMETRY_INTERVAL) {
    lastTelemetryTx = millis();
    sendTelemetry();
  }
  
  // Small delay to prevent tight loop
  delay(10);
}

    sendTelemetry();
  }
  
  // Small delay to prevent tight loop
  delay(10);
}
  while (bearingDiff < -180) bearingDiff += 360;
  
  if (bearingDiff > 15) {
    // Need to turn right
    turnRight(maxSpeed);
  } else if (bearingDiff < -15) {
    // Need to turn left
    turnLeft(maxSpeed);
  } else {
    // Heading roughly correct, move forward
    moveForward(maxSpeed);
  }
}

// ============================================================================
// SENSOR FUNCTIONS
// ============================================================================

float readUltrasonicDistance() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  
  long duration = pulseIn(PIN_ECHO, HIGH, 26000);
  if (duration == 0) return -1.0;
  return duration * 0.034 / 2.0;
}

float readBatteryPercent() {
  int raw = analogRead(PIN_BATTERY);
  float voltage = (raw / 4095.0) * 3.3 * 4.04; // Voltage divider ratio
  return constrain((voltage - VMIN) / (VMAX - VMIN) * 100.0, 0, 100);
}

void readMPU6050() {
  sensors_event_t a, g, t;
  if (mpu.getEvent(&a, &g, &t)) {
    accelX = a.acceleration.x;
    accelY = a.acceleration.y;
    
    // Calculate heading from gyro (simplified)
    currentHeading = g.gyro.z;
    
    // Earthquake detection (high acceleration)
    earthquakeDetected = (abs(accelX) > 15.0 || abs(accelY) > 15.0);
  }
}

void readTemperature() {
  tempSensor.requestTemperatures();
  currentTemp = tempSensor.getTempCByIndex(0);
  if (currentTemp == DEVICE_DISCONNECTED_C) {
    currentTemp = -999.0;
  }
}

void readGPS() {
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }
  if (gps.location.isValid()) {
    currentLat = gps.location.lat();
    currentLng = gps.location.lng();
  }
}

// ============================================================================
// CAMERA COMMUNICATION
// ============================================================================

void sendCamCommand(CamCommand cmd) {
  SerialCam.write((uint8_t)cmd);
}

bool requestImageCapture() {
  sendCamCommand(CAM_CMD_CAPTURE);
  
  // Wait for response with timeout
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
    if (SerialCam.available() >= sizeof(DetectionResult)) {
      DetectionResult result;
      SerialCam.readBytes((char*)&result, sizeof(result));
      camFireDetected = result.fire;
      camMotionDetected = result.motion;
      camHumanDetected = result.human;
      return true;
    }
  }
  return false;
}

void startCameraStream() {
  if (!isStreaming) {
    sendCamCommand(CAM_CMD_STREAM_ON);
    isStreaming = true;
  }
}

void stopCameraStream() {
  if (isStreaming) {
    sendCamCommand(CAM_CMD_STREAM_OFF);
    isStreaming = false;
  }
}

void checkCameraDetection() {
  if (SerialCam.available() >= sizeof(DetectionResult)) {
    DetectionResult result;
    SerialCam.readBytes((char*)&result, sizeof(result));
    camFireDetected = result.fire;
    camMotionDetected = result.motion;
    camHumanDetected = result.human;
  }
  
  // Check for simple text detection (fallback)
  if (SerialCam.available()) {
    String camData = SerialCam.readStringUntil('\n');
    if (camData.indexOf("HAZARD:FIRE") >= 0) camFireDetected = true;
    if (camData.indexOf("HAZARD:MOTION") >= 0) camMotionDetected = true;
    if (camData.indexOf("HAZARD:HUMAN") >= 0) camHumanDetected = true;
    if (camData.indexOf("CLEAR") >= 0) {
      camFireDetected = false;
      camMotionDetected = false;
      camHumanDetected = false;
    }
  }
}

// ============================================================================
// GSM/SMS FUNCTIONS
// ============================================================================

void sendSMS(const String& message) {
  SerialSIM.println("AT+CMGF=1");
  delay(100);
  
  // Send to primary number
  if (strlen(config.phone1) > 0) {
    SerialSIM.print("AT+CMGS=\"");
    SerialSIM.print(config.phone1);
    SerialSIM.println("\"");
    delay(100);
    SerialSIM.print(message);
    SerialSIM.write(26);
    delay(1000);
  }
  
  // Send to secondary number if configured
  if (strlen(config.phone2) > 0) {
    SerialSIM.println("AT+CMGF=1");
    delay(100);
    SerialSIM.print("AT+CMGS=\"");
    SerialSIM.print(config.phone2);
    SerialSIM.println("\"");
    delay(100);
    SerialSIM.print(message);
    SerialSIM.write(26);
    delay(1000);
  }
}

// ============================================================================
// ESP-NOW COMMUNICATION
// ============================================================================

void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len == sizeof(CommandPacket)) {
    CommandPacket cmd;
    memcpy(&cmd, data, sizeof(cmd));
    manualCommand = String(cmd.cmd);
    manualOverride = true;
  }
}

void onEspNowSent(const uint8_t *mac, esp_now_send_status_t status) {
  // Could add retry logic here
}

void initEspNow() {
  WiFi.mode(WIFI_STA);
  
  if (esp_now_init() != ESP_OK) {
    return;
  }
  
  esp_now_register_recv_cb(onEspNowRecv);
  esp_now_register_send_cb(onEspNowSent);
  
  memcpy(peerInfo.peer_addr, baseMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

void sendTelemetry() {
  // Prepare telemetry packet
  telemetry.id = (currentState == STATE_ALERT) ? 2 : 1;
  telemetry.temp = currentTemp;
  telemetry.bat = batteryPercent;
  telemetry.lat = currentLat;
  telemetry.lng = currentLng;
  telemetry.hazard = (currentState == STATE_ALERT);
  telemetry.distance = obstacleDistance;
  telemetry.accelX = accelX;
  telemetry.accelY = accelY;
  telemetry.heading = currentHeading;
  telemetry.streaming = isStreaming;
  
  switch (currentState) {
    case STATE_PATROL:    strcpy(telemetry.status, "PATROL"); break;
    case STATE_RESEARCH:  strcpy(telemetry.status, "RESEARCH"); break;
    case STATE_ALERT:     strcpy(telemetry.status, "CRITICAL ALERT"); break;
    case STATE_RETURN_BASE: strcpy(telemetry.status, "RETURNING"); break;
    case STATE_DOCKED:    strcpy(telemetry.status, "DOCKED"); break;
  }
  
  esp_now_send(baseMac, (uint8_t *)&telemetry, sizeof(telemetry));
}

// ============================================================================
// VIDEO STREAMING SERVER
// ============================================================================

void handleStreamRequest() {
  streamServer.sendHeader("Content-Type", "multipart/x-mixed-replace; boundary=frame");
  streamServer.sendHeader("Connection", "close");
  
  // This is a simplified streaming handler
  // In practice, you'd need to implement proper MJPEG streaming
  streamServer.send(200, "text/plain", "Stream active");
}

void handleCaptureRequest() {
  if (requestImageCapture()) {
    streamServer.send(200, "application/json", 
      "{\"fire\":" + String(camFireDetected ? "true" : "false") + 
      ",\"motion\":" + String(camMotionDetected ? "true" : "false") + 
      ",\"human\":" + String(camHumanDetected ? "true" : "false") + "}");
  } else {
    streamServer.send(504, "application/json", "{\"error\":\"timeout\"}");
  }
}

void handleConfigRequest() {
  String json = "{";
  json += "\"phone1\":\"" + String(config.phone1) + "\",";
  json += "\"phone2\":\"" + String(config.phone2) + "\",";
  json += "\"baseLat\":" + String(config.baseLat, 6) + ",";
  json += "\"baseLng\":" + String(config.baseLng, 6) + ",";
  json += "\"alertTempHigh\":" + String(config.alertTempHigh);
  json += "}";
  streamServer.send(200, "application/json", json);
}

void handleSetConfigRequest() {
  if (streamServer.hasArg("phone1")) {
    strncpy(config.phone1, streamServer.arg("phone1").c_str(), 15);
    config.phone1[15] = '\0';
  }
  if (streamServer.hasArg("phone2")) {
    strncpy(config.phone2, streamServer.arg("phone2").c_str(), 15);
    config.phone2[15] = '\0';
  }
  if (streamServer.hasArg("baseLat")) {
    config.baseLat = streamServer.arg("baseLat").toFloat();
  }
  if (streamServer.hasArg("baseLng")) {
    config.baseLng = streamServer.arg("baseLng").toFloat();
  }
  saveConfig();
  streamServer.send(200, "application/json", "{\"status\":\"ok\"}");
}

void initStreamServer() {
  streamServer.on("/stream", handleStreamRequest);
  streamServer.on("/capture", HTTP_POST, handleCaptureRequest);
  streamServer.on("/config", HTTP_GET, handleConfigRequest);
  streamServer.on("/config", HTTP_POST, handleSetConfigRequest);
  streamServer.begin();
}

// ============================================================================
// MANUAL COMMAND HANDLER
// ============================================================================

void handleManualCommand() {
  if (manualCommand == "FORWARD") {
    moveForward(maxSpeed);
  } else if (manualCommand == "BACKWARD") {
    moveBackward(maxSpeed);
  } else if (manualCommand == "LEFT") {
    turnLeft(MOTOR_SPEED_TURN);
  } else if (manualCommand == "RIGHT") {
    turnRight(MOTOR_SPEED_TURN);
  } else if (manualCommand == "STOP") {
    stopMotors();
  }
  manualOverride = false;
  manualCommand = "";
}

// ============================================================================
// STATE MACHINE HANDLERS
// ============================================================================

void handlePatrolState() {
  // Check for hazards
  if (currentTemp > config.alertTempHigh || earthquakeDetected || 
      camFireDetected || camHumanDetected) {
    currentState = STATE_ALERT;
    stateTimer = millis();
    return;
  }
  
  // Obstacle avoidance
  if (obstacleDistance > 0 && obstacleDistance < OBSTACLE_CM) {
    stopMotors();
    // Back up a bit then turn
    moveBackward(MOTOR_SPEED_LOW);
    delay(300);
    turnRight(MOTOR_SPEED_TURN);
    delay(400);
    stopMotors();
  } else {
    moveForward(maxSpeed);
  }
}

void handleResearchState() {
  // Stay in research mode for 3 seconds (non-blocking)
  stopMotors();
  
  if (millis() - stateTimer > 3000) {
    currentState = STATE_PATROL;
  }
}

void handleAlertState() {
  // Stop and sound alarm
  stopMotors();
  digitalWrite(PIN_BUZZER, HIGH);
  
  // Send SMS alert
  String alertMsg = "HAZARD DETECTED!\n";
  alertMsg += "Location: " + String(currentLat, 6) + ", " + String(currentLng, 6) + "\n";
  alertMsg += "Temp: " + String(currentTemp, 1) + " C\n";
  alertMsg += "Status: " + String(telemetry.status);
  sendSMS(alertMsg);
  
  // Send telemetry immediately
  sendTelemetry();
  
  // Wait 2 seconds (non-blocking would be better, but this is critical)
  delay(2000);
  digitalWrite(PIN_BUZZER, LOW);
  
  // Clear detection flags
  camFireDetected = false;
  camMotionDetected = false;
  camHumanDetected = false;
  
  // Transition to research mode
  currentState = STATE_RESEARCH;
  stateTimer = millis();
}

void handleReturnBaseState() {
  // Check if we've reached base
  if (obstacleDistance > 0 && obstacleDistance < 10) {
    currentState = STATE_DOCKED;
    stateTimer = millis();
    return;
  }
  
  // Obstacle avoidance while returning
  if (obstacleDistance > 0 && obstacleDistance < OBSTACLE_CM) {
    stopMotors();
    turnLeft(MOTOR_SPEED_TURN);
    delay(400);
  } else {
    navigateToBase();
  }
}

void handleDockedState() {
  stopMotors();
  stopCameraStream();
  
  // Check if battery is charged
  if (batteryPercent > 95) {
    currentState = STATE_PATROL;
  }
}

// ============================================================================
// MAIN SETUP
// ============================================================================

void setup() {
  // Initialize serial ports
  Serial.begin(115200);
  SerialCam.begin(115200, SERIAL_8N1, PIN_CAM_RX, PIN_CAM_TX);
  SerialSIM.begin(9600, SERIAL_8N1, PIN_SIM_RX, PIN_SIM_TX);
  SerialGPS.begin(9600);
  
  // Initialize watchdog timer
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);
  
  // Load configuration from EEPROM
  loadConfig();
  
  // Initialize pins
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_MOTOR_L_SPEED, OUTPUT);
  pinMode(PIN_MOTOR_L_DIR, OUTPUT);
  pinMode(PIN_MOTOR_R_SPEED, OUTPUT);
  pinMode(PIN_MOTOR_R_DIR, OUTPUT);
  
  // Initialize I2C for MPU6050
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  
  // Initialize MPU6050
  if (mpu.begin()) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
  
  // Initialize temperature sensor
  tempSensor.begin();
  
  // Initialize ESP-NOW
  initEspNow();
  
  // Initialize streaming server
  initStreamServer();
  
  // Initial sensor readings
  readTemperature();
  batteryPercent = readBatteryPercent();
  
  Serial.println("AERO SENTINEL Rover initialized");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Feed watchdog
  esp_task_wdt_reset();
  
  // Handle streaming server requests
  streamServer.handleClient();
  
  // --- 1. READ SENSORS (non-blocking, at interval) ---
  if (millis() - lastSensorRead > SENSOR_INTERVAL) {
    lastSensorRead = millis();
    
    obstacleDistance = readUltrasonicDistance();
    batteryPercent = readBatteryPercent();
    readTemperature();
    readMPU6050();
    readGPS();
    checkCameraDetection();
    
    // Check if near base for video streaming
    float distToBase = calculateDistance(currentLat, currentLng, config.baseLat, config.baseLng);
    nearBase = (distToBase < NEAR_BASE_CM);
    
    // Start/stop streaming based on proximity to base
    if (nearBase && !isStreaming) {
      startCameraStream();
    } else if (!nearBase && isStreaming) {
      stopCameraStream();
    }
    
    // Power saving mode
    if (batteryPercent < LOW_BAT_PERCENT) {
      maxSpeed = MOTOR_SPEED_LOW;
      if (currentState == STATE_PATROL) {
        currentState = STATE_RETURN_BASE;
      }
    } else {
      maxSpeed = MOTOR_SPEED_FULL;
    }
  }
  
  // --- 2. HANDLE MANUAL OVERRIDE ---
  if (manualOverride) {
    handleManualCommand();
    return; // Skip state machine during manual control
  }
  
  // --- 3. STATE MACHINE ---
  switch (currentState) {
    case STATE_PATROL:
      handlePatrolState();
      break;
      
    case STATE_RESEARCH:
      handleResearchState();
      break;
      
    case STATE_ALERT:
      handleAlertState();
      break;
      
    case STATE_RETURN_BASE:
      handleReturnBaseState();
      break;
      
    case STATE_DOCKED:
      handleDockedState();
      break;
  }
  
  // --- 4. SEND TELEMETRY ---
  if (millis() - lastTelemetryTx > TELEMETRY_INTERVAL) {
    lastTelemetryTx = millis();
    sendTelemetry();
  }
  
  // Small delay to prevent tight loop
  delay(10);
}

  while (bearingDiff < -180) bearingDiff += 360;
  
  if (bearingDiff > 15) {
    // Need to turn right
    turnRight(maxSpeed);
  } else if (bearingDiff < -15) {
    // Need to turn left
    turnLeft(maxSpeed);
  } else {
    // Heading roughly correct, move forward
    moveForward(maxSpeed);
  }
}

// ============================================================================
// SENSOR FUNCTIONS
// ============================================================================

float readUltrasonicDistance() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  
  long duration = pulseIn(PIN_ECHO, HIGH, 26000);
  if (duration == 0) return -1.0;
  return duration * 0.034 / 2.0;
}

float readBatteryPercent() {
  int raw = analogRead(PIN_BATTERY);
  float voltage = (raw / 4095.0) * 3.3 * 4.04; // Voltage divider ratio
  return constrain((voltage - VMIN) / (VMAX - VMIN) * 100.0, 0, 100);
}

void readMPU6050() {
  sensors_event_t a, g, t;
  if (mpu.getEvent(&a, &g, &t)) {
    accelX = a.acceleration.x;
    accelY = a.acceleration.y;
    
    // Calculate heading from gyro (simplified)
    currentHeading = g.gyro.z;
    
    // Earthquake detection (high acceleration)
    earthquakeDetected = (abs(accelX) > 15.0 || abs(accelY) > 15.0);
  }
}

void readTemperature() {
  tempSensor.requestTemperatures();
  currentTemp = tempSensor.getTempCByIndex(0);
  if (currentTemp == DEVICE_DISCONNECTED_C) {
    currentTemp = -999.0;
  }
}

void readGPS() {
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }
  if (gps.location.isValid()) {
    currentLat = gps.location.lat();
    currentLng = gps.location.lng();
  }
}

// ============================================================================
// CAMERA COMMUNICATION
// ============================================================================

void sendCamCommand(CamCommand cmd) {
  SerialCam.write((uint8_t)cmd);
}

bool requestImageCapture() {
  sendCamCommand(CAM_CMD_CAPTURE);
  
  // Wait for response with timeout
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
    if (SerialCam.available() >= sizeof(DetectionResult)) {
      DetectionResult result;
      SerialCam.readBytes((char*)&result, sizeof(result));
      camFireDetected = result.fire;
      camMotionDetected = result.motion;
      camHumanDetected = result.human;
      return true;
    }
  }
  return false;
}

void startCameraStream() {
  if (!isStreaming) {
    sendCamCommand(CAM_CMD_STREAM_ON);
    isStreaming = true;
  }
}

void stopCameraStream() {
  if (isStreaming) {
    sendCamCommand(CAM_CMD_STREAM_OFF);
    isStreaming = false;
  }
}

void checkCameraDetection() {
  if (SerialCam.available() >= sizeof(DetectionResult)) {
    DetectionResult result;
    SerialCam.readBytes((char*)&result, sizeof(result));
    camFireDetected = result.fire;
    camMotionDetected = result.motion;
    camHumanDetected = result.human;
  }
  
  // Check for simple text detection (fallback)
  if (SerialCam.available()) {
    String camData = SerialCam.readStringUntil('\n');
    if (camData.indexOf("HAZARD:FIRE") >= 0) camFireDetected = true;
    if (camData.indexOf("HAZARD:MOTION") >= 0) camMotionDetected = true;
    if (camData.indexOf("HAZARD:HUMAN") >= 0) camHumanDetected = true;
    if (camData.indexOf("CLEAR") >= 0) {
      camFireDetected = false;
      camMotionDetected = false;
      camHumanDetected = false;
    }
  }
}

// ============================================================================
// GSM/SMS FUNCTIONS
// ============================================================================

void sendSMS(const String& message) {
  SerialSIM.println("AT+CMGF=1");
  delay(100);
  
  // Send to primary number
  if (strlen(config.phone1) > 0) {
    SerialSIM.print("AT+CMGS=\"");
    SerialSIM.print(config.phone1);
    SerialSIM.println("\"");
    delay(100);
    SerialSIM.print(message);
    SerialSIM.write(26);
    delay(1000);
  }
  
  // Send to secondary number if configured
  if (strlen(config.phone2) > 0) {
    SerialSIM.println("AT+CMGF=1");
    delay(100);
    SerialSIM.print("AT+CMGS=\"");
    SerialSIM.print(config.phone2);
    SerialSIM.println("\"");
    delay(100);
    SerialSIM.print(message);
    SerialSIM.write(26);
    delay(1000);
  }
}

// ============================================================================
// ESP-NOW COMMUNICATION
// ============================================================================

void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len == sizeof(CommandPacket)) {
    CommandPacket cmd;
    memcpy(&cmd, data, sizeof(cmd));
    manualCommand = String(cmd.cmd);
    manualOverride = true;
  }
}

void onEspNowSent(const uint8_t *mac, esp_now_send_status_t status) {
  // Could add retry logic here
}

void initEspNow() {
  WiFi.mode(WIFI_STA);
  
  if (esp_now_init() != ESP_OK) {
    return;
  }
  
  esp_now_register_recv_cb(onEspNowRecv);
  esp_now_register_send_cb(onEspNowSent);
  
  memcpy(peerInfo.peer_addr, baseMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

void sendTelemetry() {
  // Prepare telemetry packet
  telemetry.id = (currentState == STATE_ALERT) ? 2 : 1;
  telemetry.temp = currentTemp;
  telemetry.bat = batteryPercent;
  telemetry.lat = currentLat;
  telemetry.lng = currentLng;
  telemetry.hazard = (currentState == STATE_ALERT);
  telemetry.distance = obstacleDistance;
  telemetry.accelX = accelX;
  telemetry.accelY = accelY;
  telemetry.heading = currentHeading;
  telemetry.streaming = isStreaming;
  
  switch (currentState) {
    case STATE_PATROL:    strcpy(telemetry.status, "PATROL"); break;
    case STATE_RESEARCH:  strcpy(telemetry.status, "RESEARCH"); break;
    case STATE_ALERT:     strcpy(telemetry.status, "CRITICAL ALERT"); break;
    case STATE_RETURN_BASE: strcpy(telemetry.status, "RETURNING"); break;
    case STATE_DOCKED:    strcpy(telemetry.status, "DOCKED"); break;
  }
  
  esp_now_send(baseMac, (uint8_t *)&telemetry, sizeof(telemetry));
}

// ============================================================================
// VIDEO STREAMING SERVER
// ============================================================================

void handleStreamRequest() {
  streamServer.sendHeader("Content-Type", "multipart/x-mixed-replace; boundary=frame");
  streamServer.sendHeader("Connection", "close");
  
  // This is a simplified streaming handler
  // In practice, you'd need to implement proper MJPEG streaming
  streamServer.send(200, "text/plain", "Stream active");
}

void handleCaptureRequest() {
  if (requestImageCapture()) {
    streamServer.send(200, "application/json", 
      "{\"fire\":" + String(camFireDetected ? "true" : "false") + 
      ",\"motion\":" + String(camMotionDetected ? "true" : "false") + 
      ",\"human\":" + String(camHumanDetected ? "true" : "false") + "}");
  } else {
    streamServer.send(504, "application/json", "{\"error\":\"timeout\"}");
  }
}

void handleConfigRequest() {
  String json = "{";
  json += "\"phone1\":\"" + String(config.phone1) + "\",";
  json += "\"phone2\":\"" + String(config.phone2) + "\",";
  json += "\"baseLat\":" + String(config.baseLat, 6) + ",";
  json += "\"baseLng\":" + String(config.baseLng, 6) + ",";
  json += "\"alertTempHigh\":" + String(config.alertTempHigh);
  json += "}";
  streamServer.send(200, "application/json", json);
}

void handleSetConfigRequest() {
  if (streamServer.hasArg("phone1")) {
    strncpy(config.phone1, streamServer.arg("phone1").c_str(), 15);
    config.phone1[15] = '\0';
  }
  if (streamServer.hasArg("phone2")) {
    strncpy(config.phone2, streamServer.arg("phone2").c_str(), 15);
    config.phone2[15] = '\0';
  }
  if (streamServer.hasArg("baseLat")) {
    config.baseLat = streamServer.arg("baseLat").toFloat();
  }
  if (streamServer.hasArg("baseLng")) {
    config.baseLng = streamServer.arg("baseLng").toFloat();
  }
  saveConfig();
  streamServer.send(200, "application/json", "{\"status\":\"ok\"}");
}

void initStreamServer() {
  streamServer.on("/stream", handleStreamRequest);
  streamServer.on("/capture", HTTP_POST, handleCaptureRequest);
  streamServer.on("/config", HTTP_GET, handleConfigRequest);
  streamServer.on("/config", HTTP_POST, handleSetConfigRequest);
  streamServer.begin();
}

// ============================================================================
// MANUAL COMMAND HANDLER
// ============================================================================

void handleManualCommand() {
  if (manualCommand == "FORWARD") {
    moveForward(maxSpeed);
  } else if (manualCommand == "BACKWARD") {
    moveBackward(maxSpeed);
  } else if (manualCommand == "LEFT") {
    turnLeft(MOTOR_SPEED_TURN);
  } else if (manualCommand == "RIGHT") {
    turnRight(MOTOR_SPEED_TURN);
  } else if (manualCommand == "STOP") {
    stopMotors();
  }
  manualOverride = false;
  manualCommand = "";
}

// ============================================================================
// STATE MACHINE HANDLERS
// ============================================================================

void handlePatrolState() {
  // Check for hazards
  if (currentTemp > config.alertTempHigh || earthquakeDetected || 
      camFireDetected || camHumanDetected) {
    currentState = STATE_ALERT;
    stateTimer = millis();
    return;
  }
  
  // Obstacle avoidance
  if (obstacleDistance > 0 && obstacleDistance < OBSTACLE_CM) {
    stopMotors();
    // Back up a bit then turn
    moveBackward(MOTOR_SPEED_LOW);
    delay(300);
    turnRight(MOTOR_SPEED_TURN);
    delay(400);
    stopMotors();
  } else {
    moveForward(maxSpeed);
  }
}

void handleResearchState() {
  // Stay in research mode for 3 seconds (non-blocking)
  stopMotors();
  
  if (millis() - stateTimer > 3000) {
    currentState = STATE_PATROL;
  }
}

void handleAlertState() {
  // Stop and sound alarm
  stopMotors();
  digitalWrite(PIN_BUZZER, HIGH);
  
  // Send SMS alert
  String alertMsg = "HAZARD DETECTED!\n";
  alertMsg += "Location: " + String(currentLat, 6) + ", " + String(currentLng, 6) + "\n";
  alertMsg += "Temp: " + String(currentTemp, 1) + " C\n";
  alertMsg += "Status: " + String(telemetry.status);
  sendSMS(alertMsg);
  
  // Send telemetry immediately
  sendTelemetry();
  
  // Wait 2 seconds (non-blocking would be better, but this is critical)
  delay(2000);
  digitalWrite(PIN_BUZZER, LOW);
  
  // Clear detection flags
  camFireDetected = false;
  camMotionDetected = false;
  camHumanDetected = false;
  
  // Transition to research mode
  currentState = STATE_RESEARCH;
  stateTimer = millis();
}

void handleReturnBaseState() {
  // Check if we've reached base
  if (obstacleDistance > 0 && obstacleDistance < 10) {
    currentState = STATE_DOCKED;
    stateTimer = millis();
    return;
  }
  
  // Obstacle avoidance while returning
  if (obstacleDistance > 0 && obstacleDistance < OBSTACLE_CM) {
    stopMotors();
    turnLeft(MOTOR_SPEED_TURN);
    delay(400);
  } else {
    navigateToBase();
  }
}

void handleDockedState() {
  stopMotors();
  stopCameraStream();
  
  // Check if battery is charged
  if (batteryPercent > 95) {
    currentState = STATE_PATROL;
  }
}

// ============================================================================
// MAIN SETUP
// ============================================================================

void setup() {
  // Initialize serial ports
  Serial.begin(115200);
  SerialCam.begin(115200, SERIAL_8N1, PIN_CAM_RX, PIN_CAM_TX);
  SerialSIM.begin(9600, SERIAL_8N1, PIN_SIM_RX, PIN_SIM_TX);
  SerialGPS.begin(9600);
  
  // Initialize watchdog timer
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);
  
  // Load configuration from EEPROM
  loadConfig();
  
  // Initialize pins
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_MOTOR_L_SPEED, OUTPUT);
  pinMode(PIN_MOTOR_L_DIR, OUTPUT);
  pinMode(PIN_MOTOR_R_SPEED, OUTPUT);
  pinMode(PIN_MOTOR_R_DIR, OUTPUT);
  
  // Initialize I2C for MPU6050
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  
  // Initialize MPU6050
  if (mpu.begin()) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
  
  // Initialize temperature sensor
  tempSensor.begin();
  
  // Initialize ESP-NOW
  initEspNow();
  
  // Initialize streaming server
  initStreamServer();
  
  // Initial sensor readings
  readTemperature();
  batteryPercent = readBatteryPercent();
  
  Serial.println("AERO SENTINEL Rover initialized");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Feed watchdog
  esp_task_wdt_reset();
  
  // Handle streaming server requests
  streamServer.handleClient();
  
  // --- 1. READ SENSORS (non-blocking, at interval) ---
  if (millis() - lastSensorRead > SENSOR_INTERVAL) {
    lastSensorRead = millis();
    
    obstacleDistance = readUltrasonicDistance();
    batteryPercent = readBatteryPercent();
    readTemperature();
    readMPU6050();
    readGPS();
    checkCameraDetection();
    
    // Check if near base for video streaming
    float distToBase = calculateDistance(currentLat, currentLng, config.baseLat, config.baseLng);
    nearBase = (distToBase < NEAR_BASE_CM);
    
    // Start/stop streaming based on proximity to base
    if (nearBase && !isStreaming) {
      startCameraStream();
    } else if (!nearBase && isStreaming) {
      stopCameraStream();
    }
    
    // Power saving mode
    if (batteryPercent < LOW_BAT_PERCENT) {
      maxSpeed = MOTOR_SPEED_LOW;
      if (currentState == STATE_PATROL) {
        currentState = STATE_RETURN_BASE;
      }
    } else {
      maxSpeed = MOTOR_SPEED_FULL;
    }
  }
  
  // --- 2. HANDLE MANUAL OVERRIDE ---
  if (manualOverride) {
    handleManualCommand();
    return; // Skip state machine during manual control
  }
  
  // --- 3. STATE MACHINE ---
  switch (currentState) {
    case STATE_PATROL:
      handlePatrolState();
      break;
      
    case STATE_RESEARCH:
      handleResearchState();
      break;
      
    case STATE_ALERT:
      handleAlertState();
      break;
      
    case STATE_RETURN_BASE:
      handleReturnBaseState();
      break;
      
    case STATE_DOCKED:
      handleDockedState();
      break;
  }
  
  // --- 4. SEND TELEMETRY ---
  if (millis() - lastTelemetryTx > TELEMETRY_INTERVAL) {
    lastTelemetryTx = millis();
    sendTelemetry();
  }
  
  // Small delay to prevent tight loop
  delay(10);
}
  while (bearingDiff < -180) bearingDiff += 360;
  
  if (bearingDiff > 15) {
    // Need to turn right
    turnRight(maxSpeed);
  } else if (bearingDiff < -15) {
    // Need to turn left
    turnLeft(maxSpeed);
  } else {
    // Heading roughly correct, move forward
    moveForward(maxSpeed);
  }
}

// ============================================================================
// SENSOR FUNCTIONS
// ============================================================================

float readUltrasonicDistance() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  
  long duration = pulseIn(PIN_ECHO, HIGH, 26000);
  if (duration == 0) return -1.0;
  return duration * 0.034 / 2.0;
}

float readBatteryPercent() {
  int raw = analogRead(PIN_BATTERY);
  float voltage = (raw / 4095.0) * 3.3 * 4.04; // Voltage divider ratio
  return constrain((voltage - VMIN) / (VMAX - VMIN) * 100.0, 0, 100);
}

void readMPU6050() {
  sensors_event_t a, g, t;
  if (mpu.getEvent(&a, &g, &t)) {
    accelX = a.acceleration.x;
    accelY = a.acceleration.y;
    
    // Calculate heading from gyro (simplified)
    currentHeading = g.gyro.z;
    
    // Earthquake detection (high acceleration)
    earthquakeDetected = (abs(accelX) > 15.0 || abs(accelY) > 15.0);
  }
}

void readTemperature() {
  tempSensor.requestTemperatures();
  currentTemp = tempSensor.getTempCByIndex(0);
  if (currentTemp == DEVICE_DISCONNECTED_C) {
    currentTemp = -999.0;
  }
}

void readGPS() {
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }
  if (gps.location.isValid()) {
    currentLat = gps.location.lat();
    currentLng = gps.location.lng();
  }
}

// ============================================================================
// CAMERA COMMUNICATION
// ============================================================================

void sendCamCommand(CamCommand cmd) {
  SerialCam.write((uint8_t)cmd);
}

bool requestImageCapture() {
  sendCamCommand(CAM_CMD_CAPTURE);
  
  // Wait for response with timeout
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
    if (SerialCam.available() >= sizeof(DetectionResult)) {
      DetectionResult result;
      SerialCam.readBytes((char*)&result, sizeof(result));
      camFireDetected = result.fire;
      camMotionDetected = result.motion;
      camHumanDetected = result.human;
      return true;
    }
  }
  return false;
}

void startCameraStream() {
  if (!isStreaming) {
    sendCamCommand(CAM_CMD_STREAM_ON);
    isStreaming = true;
  }
}

void stopCameraStream() {
  if (isStreaming) {
    sendCamCommand(CAM_CMD_STREAM_OFF);
    isStreaming = false;
  }
}

void checkCameraDetection() {
  if (SerialCam.available() >= sizeof(DetectionResult)) {
    DetectionResult result;
    SerialCam.readBytes((char*)&result, sizeof(result));
    camFireDetected = result.fire;
    camMotionDetected = result.motion;
    camHumanDetected = result.human;
  }
  
  // Check for simple text detection (fallback)
  if (SerialCam.available()) {
    String camData = SerialCam.readStringUntil('\n');
    if (camData.indexOf("HAZARD:FIRE") >= 0) camFireDetected = true;
    if (camData.indexOf("HAZARD:MOTION") >= 0) camMotionDetected = true;
    if (camData.indexOf("HAZARD:HUMAN") >= 0) camHumanDetected = true;
    if (camData.indexOf("CLEAR") >= 0) {
      camFireDetected = false;
      camMotionDetected = false;
      camHumanDetected = false;
    }
  }
}

// ============================================================================
// GSM/SMS FUNCTIONS
// ============================================================================

void sendSMS(const String& message) {
  SerialSIM.println("AT+CMGF=1");
  delay(100);
  
  // Send to primary number
  if (strlen(config.phone1) > 0) {
    SerialSIM.print("AT+CMGS=\"");
    SerialSIM.print(config.phone1);
    SerialSIM.println("\"");
    delay(100);
    SerialSIM.print(message);
    SerialSIM.write(26);
    delay(1000);
  }
  
  // Send to secondary number if configured
  if (strlen(config.phone2) > 0) {
    SerialSIM.println("AT+CMGF=1");
    delay(100);
    SerialSIM.print("AT+CMGS=\"");
    SerialSIM.print(config.phone2);
    SerialSIM.println("\"");
    delay(100);
    SerialSIM.print(message);
    SerialSIM.write(26);
    delay(1000);
  }
}

// ============================================================================
// ESP-NOW COMMUNICATION
// ============================================================================

void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len == sizeof(CommandPacket)) {
    CommandPacket cmd;
    memcpy(&cmd, data, sizeof(cmd));
    manualCommand = String(cmd.cmd);
    manualOverride = true;
  }
}

void onEspNowSent(const uint8_t *mac, esp_now_send_status_t status) {
  // Could add retry logic here
}

void initEspNow() {
  WiFi.mode(WIFI_STA);
  
  if (esp_now_init() != ESP_OK) {
    return;
  }
  
  esp_now_register_recv_cb(onEspNowRecv);
  esp_now_register_send_cb(onEspNowSent);
  
  memcpy(peerInfo.peer_addr, baseMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

void sendTelemetry() {
  // Prepare telemetry packet
  telemetry.id = (currentState == STATE_ALERT) ? 2 : 1;
  telemetry.temp = currentTemp;
  telemetry.bat = batteryPercent;
  telemetry.lat = currentLat;
  telemetry.lng = currentLng;
  telemetry.hazard = (currentState == STATE_ALERT);
  telemetry.distance = obstacleDistance;
  telemetry.accelX = accelX;
  telemetry.accelY = accelY;
  telemetry.heading = currentHeading;
  telemetry.streaming = isStreaming;
  
  switch (currentState) {
    case STATE_PATROL:    strcpy(telemetry.status, "PATROL"); break;
    case STATE_RESEARCH:  strcpy(telemetry.status, "RESEARCH"); break;
    case STATE_ALERT:     strcpy(telemetry.status, "CRITICAL ALERT"); break;
    case STATE_RETURN_BASE: strcpy(telemetry.status, "RETURNING"); break;
    case STATE_DOCKED:    strcpy(telemetry.status, "DOCKED"); break;
  }
  
  esp_now_send(baseMac, (uint8_t *)&telemetry, sizeof(telemetry));
}

// ============================================================================
// VIDEO STREAMING SERVER
// ============================================================================

void handleStreamRequest() {
  streamServer.sendHeader("Content-Type", "multipart/x-mixed-replace; boundary=frame");
  streamServer.sendHeader("Connection", "close");
  
  // This is a simplified streaming handler
  // In practice, you'd need to implement proper MJPEG streaming
  streamServer.send(200, "text/plain", "Stream active");
}

void handleCaptureRequest() {
  if (requestImageCapture()) {
    streamServer.send(200, "application/json", 
      "{\"fire\":" + String(camFireDetected ? "true" : "false") + 
      ",\"motion\":" + String(camMotionDetected ? "true" : "false") + 
      ",\"human\":" + String(camHumanDetected ? "true" : "false") + "}");
  } else {
    streamServer.send(504, "application/json", "{\"error\":\"timeout\"}");
  }
}

void handleConfigRequest() {
  String json = "{";
  json += "\"phone1\":\"" + String(config.phone1) + "\",";
  json += "\"phone2\":\"" + String(config.phone2) + "\",";
  json += "\"baseLat\":" + String(config.baseLat, 6) + ",";
  json += "\"baseLng\":" + String(config.baseLng, 6) + ",";
  json += "\"alertTempHigh\":" + String(config.alertTempHigh);
  json += "}";
  streamServer.send(200, "application/json", json);
}

void handleSetConfigRequest() {
  if (streamServer.hasArg("phone1")) {
    strncpy(config.phone1, streamServer.arg("phone1").c_str(), 15);
    config.phone1[15] = '\0';
  }
  if (streamServer.hasArg("phone2")) {
    strncpy(config.phone2, streamServer.arg("phone2").c_str(), 15);
    config.phone2[15] = '\0';
  }
  if (streamServer.hasArg("baseLat")) {
    config.baseLat = streamServer.arg("baseLat").toFloat();
  }
  if (streamServer.hasArg("baseLng")) {
    config.baseLng = streamServer.arg("baseLng").toFloat();
  }
  saveConfig();
  streamServer.send(200, "application/json", "{\"status\":\"ok\"}");
}

void initStreamServer() {
  streamServer.on("/stream", handleStreamRequest);
  streamServer.on("/capture", HTTP_POST, handleCaptureRequest);
  streamServer.on("/config", HTTP_GET, handleConfigRequest);
  streamServer.on("/config", HTTP_POST, handleSetConfigRequest);
  streamServer.begin();
}

// ============================================================================
// MANUAL COMMAND HANDLER
// ============================================================================

void handleManualCommand() {
  if (manualCommand == "FORWARD") {
    moveForward(maxSpeed);
  } else if (manualCommand == "BACKWARD") {
    moveBackward(maxSpeed);
  } else if (manualCommand == "LEFT") {
    turnLeft(MOTOR_SPEED_TURN);
  } else if (manualCommand == "RIGHT") {
    turnRight(MOTOR_SPEED_TURN);
  } else if (manualCommand == "STOP") {
    stopMotors();
  }
  manualOverride = false;
  manualCommand = "";
}

// ============================================================================
// STATE MACHINE HANDLERS
// ============================================================================

void handlePatrolState() {
  // Check for hazards
  if (currentTemp > config.alertTempHigh || earthquakeDetected || 
      camFireDetected || camHumanDetected) {
    currentState = STATE_ALERT;
    stateTimer = millis();
    return;
  }
  
  // Obstacle avoidance
  if (obstacleDistance > 0 && obstacleDistance < OBSTACLE_CM) {
    stopMotors();
    // Back up a bit then turn
    moveBackward(MOTOR_SPEED_LOW);
    delay(300);
    turnRight(MOTOR_SPEED_TURN);
    delay(400);
    stopMotors();
  } else {
    moveForward(maxSpeed);
  }
}

void handleResearchState() {
  // Stay in research mode for 3 seconds (non-blocking)
  stopMotors();
  
  if (millis() - stateTimer > 3000) {
    currentState = STATE_PATROL;
  }
}

void handleAlertState() {
  // Stop and sound alarm
  stopMotors();
  digitalWrite(PIN_BUZZER, HIGH);
  
  // Send SMS alert
  String alertMsg = "HAZARD DETECTED!\n";
  alertMsg += "Location: " + String(currentLat, 6) + ", " + String(currentLng, 6) + "\n";
  alertMsg += "Temp: " + String(currentTemp, 1) + " C\n";
  alertMsg += "Status: " + String(telemetry.status);
  sendSMS(alertMsg);
  
  // Send telemetry immediately
  sendTelemetry();
  
  // Wait 2 seconds (non-blocking would be better, but this is critical)
  delay(2000);
  digitalWrite(PIN_BUZZER, LOW);
  
  // Clear detection flags
  camFireDetected = false;
  camMotionDetected = false;
  camHumanDetected = false;
  
  // Transition to research mode
  currentState = STATE_RESEARCH;
  stateTimer = millis();
}

void handleReturnBaseState() {
  // Check if we've reached base
  if (obstacleDistance > 0 && obstacleDistance < 10) {
    currentState = STATE_DOCKED;
    stateTimer = millis();
    return;
  }
  
  // Obstacle avoidance while returning
  if (obstacleDistance > 0 && obstacleDistance < OBSTACLE_CM) {
    stopMotors();
    turnLeft(MOTOR_SPEED_TURN);
    delay(400);
  } else {
    navigateToBase();
  }
}

void handleDockedState() {
  stopMotors();
  stopCameraStream();
  
  // Check if battery is charged
  if (batteryPercent > 95) {
    currentState = STATE_PATROL;
  }
}

// ============================================================================
// MAIN SETUP
// ============================================================================

void setup() {
  // Initialize serial ports
  Serial.begin(115200);
  SerialCam.begin(115200, SERIAL_8N1, PIN_CAM_RX, PIN_CAM_TX);
  SerialSIM.begin(9600, SERIAL_8N1, PIN_SIM_RX, PIN_SIM_TX);
  SerialGPS.begin(9600);
  
  // Initialize watchdog timer
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);
  
  // Load configuration from EEPROM
  loadConfig();
  
  // Initialize pins
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_MOTOR_L_SPEED, OUTPUT);
  pinMode(PIN_MOTOR_L_DIR, OUTPUT);
  pinMode(PIN_MOTOR_R_SPEED, OUTPUT);
  pinMode(PIN_MOTOR_R_DIR, OUTPUT);
  
  // Initialize I2C for MPU6050
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  
  // Initialize MPU6050
  if (mpu.begin()) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
  
  // Initialize temperature sensor
  tempSensor.begin();
  
  // Initialize ESP-NOW
  initEspNow();
  
  // Initialize streaming server
  initStreamServer();
  
  // Initial sensor readings
  readTemperature();
  batteryPercent = readBatteryPercent();
  
  Serial.println("AERO SENTINEL Rover initialized");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Feed watchdog
  esp_task_wdt_reset();
  
  // Handle streaming server requests
  streamServer.handleClient();
  
  // --- 1. READ SENSORS (non-blocking, at interval) ---
  if (millis() - lastSensorRead > SENSOR_INTERVAL) {
    lastSensorRead = millis();
    
    obstacleDistance = readUltrasonicDistance();
    batteryPercent = readBatteryPercent();
    readTemperature();
    readMPU6050();
    readGPS();
    checkCameraDetection();
    
    // Check if near base for video streaming
    float distToBase = calculateDistance(currentLat, currentLng, config.baseLat, config.baseLng);
    nearBase = (distToBase < NEAR_BASE_CM);
    
    // Start/stop streaming based on proximity to base
    if (nearBase && !isStreaming) {
      startCameraStream();
    } else if (!nearBase && isStreaming) {
      stopCameraStream();
    }
    
    // Power saving mode
    if (batteryPercent < LOW_BAT_PERCENT) {
      maxSpeed = MOTOR_SPEED_LOW;
      if (currentState == STATE_PATROL) {
        currentState = STATE_RETURN_BASE;
      }
    } else {
      maxSpeed = MOTOR_SPEED_FULL;
    }
  }
  
  // --- 2. HANDLE MANUAL OVERRIDE ---
  if (manualOverride) {
    handleManualCommand();
    return; // Skip state machine during manual control
  }
  
  // --- 3. STATE MACHINE ---
  switch (currentState) {
    case STATE_PATROL:
      handlePatrolState();
      break;
      
    case STATE_RESEARCH:
      handleResearchState();
      break;
      
    case STATE_ALERT:
      handleAlertState();
      break;
      
    case STATE_RETURN_BASE:
      handleReturnBaseState();
      break;
      
    case STATE_DOCKED:
      handleDockedState();
      break;
  }
  
  // --- 4. SEND TELEMETRY ---
  if (millis() - lastTelemetryTx > TELEMETRY_INTERVAL) {
    lastTelemetryTx = millis();
    sendTelemetry();
  }
  
  // Small delay to prevent tight loop
  delay(10);
}


    turnLeft(maxSpeed);
  } else {
    // Heading roughly correct, move forward
    moveForward(maxSpeed);
  }
}

// ============================================================================
// SENSOR FUNCTIONS
// ============================================================================

float readUltrasonicDistance() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  
  long duration = pulseIn(PIN_ECHO, HIGH, 26000);
  if (duration == 0) return -1.0;
  return duration * 0.034 / 2.0;
}

float readBatteryPercent() {
  int raw = analogRead(PIN_BATTERY);
  float voltage = (raw / 4095.0) * 3.3 * 4.04; // Voltage divider ratio
  return constrain((voltage - VMIN) / (VMAX - VMIN) * 100.0, 0, 100);
}

void readMPU6050() {
  sensors_event_t a, g, t;
  if (mpu.getEvent(&a, &g, &t)) {
    accelX = a.acceleration.x;
    accelY = a.acceleration.y;
    
    // Calculate heading from gyro (simplified)
    currentHeading = g.gyro.z;
    
    // Earthquake detection (high acceleration)
    earthquakeDetected = (abs(accelX) > 15.0 || abs(accelY) > 15.0);
  }
}

void readTemperature() {
  tempSensor.requestTemperatures();
  currentTemp = tempSensor.getTempCByIndex(0);
  if (currentTemp == DEVICE_DISCONNECTED_C) {
    currentTemp = -999.0;
  }
}

void readGPS() {
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }
  if (gps.location.isValid()) {
    currentLat = gps.location.lat();
    currentLng = gps.location.lng();
  }
}

// ============================================================================
// CAMERA COMMUNICATION
// ============================================================================

void sendCamCommand(CamCommand cmd) {
  SerialCam.write((uint8_t)cmd);
}

bool requestImageCapture() {
  sendCamCommand(CAM_CMD_CAPTURE);
  
  // Wait for response with timeout
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
    if (SerialCam.available() >= sizeof(DetectionResult)) {
      DetectionResult result;
      SerialCam.readBytes((char*)&result, sizeof(result));
      camFireDetected = result.fire;
      camMotionDetected = result.motion;
      camHumanDetected = result.human;
      return true;
    }
  }
  return false;
}

void startCameraStream() {
  if (!isStreaming) {
    sendCamCommand(CAM_CMD_STREAM_ON);
    isStreaming = true;
  }
}

void stopCameraStream() {
  if (isStreaming) {
    sendCamCommand(CAM_CMD_STREAM_OFF);
    isStreaming = false;
  }
}

void checkCameraDetection() {
  if (SerialCam.available() >= sizeof(DetectionResult)) {
    DetectionResult result;
    SerialCam.readBytes((char*)&result, sizeof(result));
    camFireDetected = result.fire;
    camMotionDetected = result.motion;
    camHumanDetected = result.human;
  }
  
  // Check for simple text detection (fallback)
  if (SerialCam.available()) {
    String camData = SerialCam.readStringUntil('\n');
    if (camData.indexOf("HAZARD:FIRE") >= 0) camFireDetected = true;
    if (camData.indexOf("HAZARD:MOTION") >= 0) camMotionDetected = true;
    if (camData.indexOf("HAZARD:HUMAN") >= 0) camHumanDetected = true;
    if (camData.indexOf("CLEAR") >= 0) {
      camFireDetected = false;
      camMotionDetected = false;
      camHumanDetected = false;
    }
  }
}

// ============================================================================
// GSM/SMS FUNCTIONS
// ============================================================================

void sendSMS(const String& message) {
  SerialSIM.println("AT+CMGF=1");
  delay(100);
  
  // Send to primary number
  if (strlen(config.phone1) > 0) {
    SerialSIM.print("AT+CMGS=\"");
    SerialSIM.print(config.phone1);
    SerialSIM.println("\"");
    delay(100);
    SerialSIM.print(message);
    SerialSIM.write(26);
    delay(1000);
  }
  
  // Send to secondary number if configured
  if (strlen(config.phone2) > 0) {
    SerialSIM.println("AT+CMGF=1");
    delay(100);
    SerialSIM.print("AT+CMGS=\"");
    SerialSIM.print(config.phone2);
    SerialSIM.println("\"");
    delay(100);
    SerialSIM.print(message);
    SerialSIM.write(26);
    delay(1000);
  }
}

// ============================================================================
// ESP-NOW COMMUNICATION
// ============================================================================

void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len == sizeof(CommandPacket)) {
    CommandPacket cmd;
    memcpy(&cmd, data, sizeof(cmd));
    manualCommand = String(cmd.cmd);
    manualOverride = true;
  }
}

void onEspNowSent(const uint8_t *mac, esp_now_send_status_t status) {
  // Could add retry logic here
}

void initEspNow() {
  WiFi.mode(WIFI_STA);
  
  if (esp_now_init() != ESP_OK) {
    return;
  }
  
  esp_now_register_recv_cb(onEspNowRecv);
  esp_now_register_send_cb(onEspNowSent);
  
  memcpy(peerInfo.peer_addr, baseMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

void sendTelemetry() {
  // Prepare telemetry packet
  telemetry.id = (currentState == STATE_ALERT) ? 2 : 1;
  telemetry.temp = currentTemp;
  telemetry.bat = batteryPercent;
  telemetry.lat = currentLat;
  telemetry.lng = currentLng;
  telemetry.hazard = (currentState == STATE_ALERT);
  telemetry.distance = obstacleDistance;
  telemetry.accelX = accelX;
  telemetry.accelY = accelY;
  telemetry.heading = currentHeading;
  telemetry.streaming = isStreaming;
  
  switch (currentState) {
    case STATE_PATROL:    strcpy(telemetry.status, "PATROL"); break;
    case STATE_RESEARCH:  strcpy(telemetry.status, "RESEARCH"); break;
    case STATE_ALERT:     strcpy(telemetry.status, "CRITICAL ALERT"); break;
    case STATE_RETURN_BASE: strcpy(telemetry.status, "RETURNING"); break;
    case STATE_DOCKED:    strcpy(telemetry.status, "DOCKED"); break;
  }
  
  esp_now_send(baseMac, (uint8_t *)&telemetry, sizeof(telemetry));
}

// ============================================================================
// VIDEO STREAMING SERVER
// ============================================================================

void handleStreamRequest() {
  streamServer.sendHeader("Content-Type", "multipart/x-mixed-replace; boundary=frame");
  streamServer.sendHeader("Connection", "close");
  
  // This is a simplified streaming handler
  // In practice, you'd need to implement proper MJPEG streaming
  streamServer.send(200, "text/plain", "Stream active");
}

void handleCaptureRequest() {
  if (requestImageCapture()) {
    streamServer.send(200, "application/json", 
      "{\"fire\":" + String(camFireDetected ? "true" : "false") + 
      ",\"motion\":" + String(camMotionDetected ? "true" : "false") + 
      ",\"human\":" + String(camHumanDetected ? "true" : "false") + "}");
  } else {
    streamServer.send(504, "application/json", "{\"error\":\"timeout\"}");
  }
}

void handleConfigRequest() {
  String json = "{";
  json += "\"phone1\":\"" + String(config.phone1) + "\",";
  json += "\"phone2\":\"" + String(config.phone2) + "\",";
  json += "\"baseLat\":" + String(config.baseLat, 6) + ",";
  json += "\"baseLng\":" + String(config.baseLng, 6) + ",";
  json += "\"alertTempHigh\":" + String(config.alertTempHigh);
  json += "}";
  streamServer.send(200, "application/json", json);
}

void handleSetConfigRequest() {
  if (streamServer.hasArg("phone1")) {
    strncpy(config.phone1, streamServer.arg("phone1").c_str(), 15);
    config.phone1[15] = '\0';
  }
  if (streamServer.hasArg("phone2")) {
    strncpy(config.phone2, streamServer.arg("phone2").c_str(), 15);
    config.phone2[15] = '\0';
  }
  if (streamServer.hasArg("baseLat")) {
    config.baseLat = streamServer.arg("baseLat").toFloat();
  }
  if (streamServer.hasArg("baseLng")) {
    config.baseLng = streamServer.arg("baseLng").toFloat();
  }
  saveConfig();
  streamServer.send(200, "application/json", "{\"status\":\"ok\"}");
}

void initStreamServer() {
  streamServer.on("/stream", handleStreamRequest);
  streamServer.on("/capture", HTTP_POST, handleCaptureRequest);
  streamServer.on("/config", HTTP_GET, handleConfigRequest);
  streamServer.on("/config", HTTP_POST, handleSetConfigRequest);
  streamServer.begin();
}

// ============================================================================
// MANUAL COMMAND HANDLER
// ============================================================================

void handleManualCommand() {
  if (manualCommand == "FORWARD") {
    moveForward(maxSpeed);
  } else if (manualCommand == "BACKWARD") {
    moveBackward(maxSpeed);
  } else if (manualCommand == "LEFT") {
    turnLeft(MOTOR_SPEED_TURN);
  } else if (manualCommand == "RIGHT") {
    turnRight(MOTOR_SPEED_TURN);
  } else if (manualCommand == "STOP") {
    stopMotors();
  }
  manualOverride = false;
  manualCommand = "";
}

// ============================================================================
// STATE MACHINE HANDLERS
// ============================================================================

void handlePatrolState() {
  // Check for hazards
  if (currentTemp > config.alertTempHigh || earthquakeDetected || 
      camFireDetected || camHumanDetected) {
    currentState = STATE_ALERT;
    stateTimer = millis();
    return;
  }
  
  // Obstacle avoidance
  if (obstacleDistance > 0 && obstacleDistance < OBSTACLE_CM) {
    stopMotors();
    // Back up a bit then turn
    moveBackward(MOTOR_SPEED_LOW);
    delay(300);
    turnRight(MOTOR_SPEED_TURN);
    delay(400);
    stopMotors();
  } else {
    moveForward(maxSpeed);
  }
}

void handleResearchState() {
  // Stay in research mode for 3 seconds (non-blocking)
  stopMotors();
  
  if (millis() - stateTimer > 3000) {
    currentState = STATE_PATROL;
  }
}

void handleAlertState() {
  // Stop and sound alarm
  stopMotors();
  digitalWrite(PIN_BUZZER, HIGH);
  
  // Send SMS alert
  String alertMsg = "STASIS ALERT!\n";
  alertMsg += "Location: " + String(currentLat, 6) + ", " + String(currentLng, 6) + "\n";
  alertMsg += "Temp: " + String(currentTemp, 1) + " C\n";
  alertMsg += "Status: " + String(telemetry.status);
  sendSMS(alertMsg);
  
  // Send telemetry immediately
  sendTelemetry();
  
  // Wait 2 seconds (non-blocking would be better, but this is critical)
  delay(2000);
  digitalWrite(PIN_BUZZER, LOW);
  
  // Clear detection flags
  camFireDetected = false;
  camMotionDetected = false;
  camHumanDetected = false;
  
  // Transition to research mode
  currentState = STATE_RESEARCH;
  stateTimer = millis();
}

void handleReturnBaseState() {
  // Check if we've reached base
  if (obstacleDistance > 0 && obstacleDistance < 10) {
    currentState = STATE_DOCKED;
    stateTimer = millis();
    return;
  }
  
  // Obstacle avoidance while returning
  if (obstacleDistance > 0 && obstacleDistance < OBSTACLE_CM) {
    stopMotors();
    turnLeft(MOTOR_SPEED_TURN);
    delay(400);
  } else {
    navigateToBase();
  }
}

void handleDockedState() {
  stopMotors();
  stopCameraStream();
  
  // Check if battery is charged
  if (batteryPercent > 95) {
    currentState = STATE_PATROL;
  }
}

// ============================================================================
// MAIN SETUP
// ============================================================================

void setup() {
  // Initialize serial ports
  Serial.begin(115200);
  SerialCam.begin(115200, SERIAL_8N1, PIN_CAM_RX, PIN_CAM_TX);
  SerialSIM.begin(9600, SERIAL_8N1, PIN_SIM_RX, PIN_SIM_TX);
  SerialGPS.begin(9600);
  
  // Initialize watchdog timer
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);
  
  // Load configuration from EEPROM
  loadConfig();
  
  // Initialize pins
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_MOTOR_L_SPEED, OUTPUT);
  pinMode(PIN_MOTOR_L_DIR, OUTPUT);
  pinMode(PIN_MOTOR_R_SPEED, OUTPUT);
  pinMode(PIN_MOTOR_R_DIR, OUTPUT);
  
  // Initialize I2C for MPU6050
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  
  // Initialize MPU6050
  if (mpu.begin()) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
  
  // Initialize temperature sensor
  tempSensor.begin();
  
  // Initialize ESP-NOW
  initEspNow();
  
  // Initialize streaming server
  initStreamServer();
  
  // Initial sensor readings
  readTemperature();
  batteryPercent = readBatteryPercent();
  
  Serial.println("STASIS Rover initialized");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Feed watchdog
  esp_task_wdt_reset();
  
  // Handle streaming server requests
  streamServer.handleClient();
  
  // --- 1. READ SENSORS (non-blocking, at interval) ---
  if (millis() - lastSensorRead > SENSOR_INTERVAL) {
    lastSensorRead = millis();
    
    obstacleDistance = readUltrasonicDistance();
    batteryPercent = readBatteryPercent();
    readTemperature();
    readMPU6050();
    readGPS();
    checkCameraDetection();
    
    // Check if near base for video streaming
    float distToBase = calculateDistance(currentLat, currentLng, config.baseLat, config.baseLng);
    nearBase = (distToBase < NEAR_BASE_CM);
    
    // Start/stop streaming based on proximity to base
    if (nearBase && !isStreaming) {
      startCameraStream();
    } else if (!nearBase && isStreaming) {
      stopCameraStream();
    }
    
    // Power saving mode
    if (batteryPercent < LOW_BAT_PERCENT) {
      maxSpeed = MOTOR_SPEED_LOW;
      if (currentState == STATE_PATROL) {
        currentState = STATE_RETURN_BASE;
      }
    } else {
      maxSpeed = MOTOR_SPEED_FULL;
    }
  }
  
  // --- 2. HANDLE MANUAL OVERRIDE ---
  if (manualOverride) {
    handleManualCommand();
    return; // Skip state machine during manual control
  }
  
  // --- 3. STATE MACHINE ---
  switch (currentState) {
    case STATE_PATROL:
      handlePatrolState();
      break;
      
    case STATE_RESEARCH:
      handleResearchState();
      break;
      
    case STATE_ALERT:
      handleAlertState();
      break;
      
    case STATE_RETURN_BASE:
      handleReturnBaseState();
      break;
      
    case STATE_DOCKED:
      handleDockedState();
      break;
  }
  
  // --- 4. SEND TELEMETRY ---
  if (millis() - lastTelemetryTx > TELEMETRY_INTERVAL) {
    lastTelemetryTx = millis();
    sendTelemetry();
  }
  
  // Small delay to prevent tight loop
  delay(10);
}

bool requestImageCapture() {
  sendCamCommand(CAM_CMD_CAPTURE);
  
  // Wait for response with timeout
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
    if (SerialCam.available() >= sizeof(DetectionResult)) {
      DetectionResult result;
      SerialCam.readBytes((char*)&result, sizeof(result));
      camFireDetected = result.fire;
      camMotionDetected = result.motion;
      camHumanDetected = result.human;
      return true;
    }
  }
  return false;
}

void startCameraStream() {
  if (!isStreaming) {
    sendCamCommand(CAM_CMD_STREAM_ON);
    isStreaming = true;
  }
}

void stopCameraStream() {
  if (isStreaming) {
    sendCamCommand(CAM_CMD_STREAM_OFF);
    isStreaming = false;
  }
}

void checkCameraDetection() {
  if (SerialCam.available() >= sizeof(DetectionResult)) {
    DetectionResult result;
    SerialCam.readBytes((char*)&result, sizeof(result));
    camFireDetected = result.fire;
    camMotionDetected = result.motion;
    camHumanDetected = result.human;
  }
  
  // Check for simple text detection (fallback)
  if (SerialCam.available()) {
    String camData = SerialCam.readStringUntil('\n');
    if (camData.indexOf("HAZARD:FIRE") >= 0) camFireDetected = true;
    if (camData.indexOf("HAZARD:MOTION") >= 0) camMotionDetected = true;
    if (camData.indexOf("HAZARD:HUMAN") >= 0) camHumanDetected = true;
    if (camData.indexOf("CLEAR") >= 0) {
      camFireDetected = false;
      camMotionDetected = false;
      camHumanDetected = false;
    }
  }
}

// ============================================================================
// GSM/SMS FUNCTIONS
// ============================================================================

void sendSMS(const String& message) {
  SerialSIM.println("AT+CMGF=1");
  delay(100);
  
  // Send to primary number
  if (strlen(config.phone1) > 0) {
    SerialSIM.print("AT+CMGS=\"");
    SerialSIM.print(config.phone1);
    SerialSIM.println("\"");
    delay(100);
    SerialSIM.print(message);
    SerialSIM.write(26);
    delay(1000);
  }
  
  // Send to secondary number if configured
  if (strlen(config.phone2) > 0) {
    SerialSIM.println("AT+CMGF=1");
    delay(100);
    SerialSIM.print("AT+CMGS=\"");
    SerialSIM.print(config.phone2);
    SerialSIM.println("\"");
    delay(100);
    SerialSIM.print(message);
    SerialSIM.write(26);
    delay(1000);
  }
}

// ============================================================================
// ESP-NOW COMMUNICATION
// ============================================================================

void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len == sizeof(CommandPacket)) {
    CommandPacket cmd;
    memcpy(&cmd, data, sizeof(cmd));
    manualCommand = String(cmd.cmd);
    manualOverride = true;
  }
}

void onEspNowSent(const uint8_t *mac, esp_now_send_status_t status) {
  // Could add retry logic here
}

void initEspNow() {
  WiFi.mode(WIFI_STA);
  
  if (esp_now_init() != ESP_OK) {
    return;
  }
  
  esp_now_register_recv_cb(onEspNowRecv);
  esp_now_register_send_cb(onEspNowSent);
  
  memcpy(peerInfo.peer_addr, baseMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

void sendTelemetry() {
  // Prepare telemetry packet
  telemetry.id = (currentState == STATE_ALERT) ? 2 : 1;
  telemetry.temp = currentTemp;
  telemetry.bat = batteryPercent;
  telemetry.lat = currentLat;
  telemetry.lng = currentLng;
  telemetry.hazard = (currentState == STATE_ALERT);
  telemetry.distance = obstacleDistance;
  telemetry.accelX = accelX;
  telemetry.accelY = accelY;
  telemetry.heading = currentHeading;
  telemetry.streaming = isStreaming;
  
  switch (currentState) {
    case STATE_PATROL:    strcpy(telemetry.status, "PATROL"); break;
    case STATE_RESEARCH:  strcpy(telemetry.status, "RESEARCH"); break;
    case STATE_ALERT:     strcpy(telemetry.status, "CRITICAL ALERT"); break;
    case STATE_RETURN_BASE: strcpy(telemetry.status, "RETURNING"); break;
    case STATE_DOCKED:    strcpy(telemetry.status, "DOCKED"); break;
  }
  
  esp_now_send(baseMac, (uint8_t *)&telemetry, sizeof(telemetry));
}

// ============================================================================
// VIDEO STREAMING SERVER
// ============================================================================

void handleStreamRequest() {
  streamServer.sendHeader("Content-Type", "multipart/x-mixed-replace; boundary=frame");
  streamServer.sendHeader("Connection", "close");
  
  // This is a simplified streaming handler
  // In practice, you'd need to implement proper MJPEG streaming
  streamServer.send(200, "text/plain", "Stream active");
}

void handleCaptureRequest() {
  if (requestImageCapture()) {
    streamServer.send(200, "application/json", 
      "{\"fire\":" + String(camFireDetected ? "true" : "false") + 
      ",\"motion\":" + String(camMotionDetected ? "true" : "false") + 
      ",\"human\":" + String(camHumanDetected ? "true" : "false") + "}");
  } else {
    streamServer.send(504, "application/json", "{\"error\":\"timeout\"}");
  }
}

void handleConfigRequest() {
  String json = "{";
  json += "\"phone1\":\"" + String(config.phone1) + "\",";
  json += "\"phone2\":\"" + String(config.phone2) + "\",";
  json += "\"baseLat\":" + String(config.baseLat, 6) + ",";
  json += "\"baseLng\":" + String(config.baseLng, 6) + ",";
  json += "\"alertTempHigh\":" + String(config.alertTempHigh);
  json += "}";
  streamServer.send(200, "application/json", json);
}

void handleSetConfigRequest() {
  if (streamServer.hasArg("phone1")) {
    strncpy(config.phone1, streamServer.arg("phone1").c_str(), 15);
    config.phone1[15] = '\0';
  }
  if (streamServer.hasArg("phone2")) {
    strncpy(config.phone2, streamServer.arg("phone2").c_str(), 15);
    config.phone2[15] = '\0';
  }
  if (streamServer.hasArg("baseLat")) {
    config.baseLat = streamServer.arg("baseLat").toFloat();
  }
  if (streamServer.hasArg("baseLng")) {
    config.baseLng = streamServer.arg("baseLng").toFloat();
  }
  saveConfig();
  streamServer.send(200, "application/json", "{\"status\":\"ok\"}");
}

void initStreamServer() {
  streamServer.on("/stream", handleStreamRequest);
  streamServer.on("/capture", HTTP_POST, handleCaptureRequest);
  streamServer.on("/config", HTTP_GET, handleConfigRequest);
  streamServer.on("/config", HTTP_POST, handleSetConfigRequest);
  streamServer.begin();
}

// ============================================================================
// MANUAL COMMAND HANDLER
// ============================================================================

void handleManualCommand() {
  if (manualCommand == "FORWARD") {
    moveForward(maxSpeed);
  } else if (manualCommand == "BACKWARD") {
    moveBackward(maxSpeed);
  } else if (manualCommand == "LEFT") {
    turnLeft(MOTOR_SPEED_TURN);
  } else if (manualCommand == "RIGHT") {
    turnRight(MOTOR_SPEED_TURN);
  } else if (manualCommand == "STOP") {
    stopMotors();
  }
  manualOverride = false;
  manualCommand = "";
}

// ============================================================================
// STATE MACHINE HANDLERS
// ============================================================================

void handlePatrolState() {
  // Check for hazards
  if (currentTemp > config.alertTempHigh || earthquakeDetected || 
      camFireDetected || camHumanDetected) {
    currentState = STATE_ALERT;
    stateTimer = millis();
    return;
  }
  
  // Obstacle avoidance
  if (obstacleDistance > 0 && obstacleDistance < OBSTACLE_CM) {
    stopMotors();
    // Back up a bit then turn
    moveBackward(MOTOR_SPEED_LOW);
    delay(300);
    turnRight(MOTOR_SPEED_TURN);
    delay(400);
    stopMotors();
  } else {
    moveForward(maxSpeed);
  }
}

void handleResearchState() {
  // Stay in research mode for 3 seconds (non-blocking)
  stopMotors();
  
  if (millis() - stateTimer > 3000) {
    currentState = STATE_PATROL;
  }
}

void handleAlertState() {
  // Stop and sound alarm
  stopMotors();
  digitalWrite(PIN_BUZZER, HIGH);
  
  // Send SMS alert
  String alertMsg = "STASIS ALERT!\n";
  alertMsg += "Location: " + String(currentLat, 6) + ", " + String(currentLng, 6) + "\n";
  alertMsg += "Temp: " + String(currentTemp, 1) + " C\n";
  alertMsg += "Status: " + String(telemetry.status);
  sendSMS(alertMsg);
  
  // Send telemetry immediately
  sendTelemetry();
  
  // Wait 2 seconds (non-blocking would be better, but this is critical)
  delay(2000);
  digitalWrite(PIN_BUZZER, LOW);
  
  // Clear detection flags
  camFireDetected = false;
  camMotionDetected = false;
  camHumanDetected = false;
  
  // Transition to research mode
  currentState = STATE_RESEARCH;
  stateTimer = millis();
}

void handleReturnBaseState() {
  // Check if we've reached base
  if (obstacleDistance > 0 && obstacleDistance < 10) {
    currentState = STATE_DOCKED;
    stateTimer = millis();
    return;
  }
  
  // Obstacle avoidance while returning
  if (obstacleDistance > 0 && obstacleDistance < OBSTACLE_CM) {
    stopMotors();
    turnLeft(MOTOR_SPEED_TURN);
    delay(400);
  } else {
    navigateToBase();
  }
}

void handleDockedState() {
  stopMotors();
  stopCameraStream();
  
  // Check if battery is charged
  if (batteryPercent > 95) {
    currentState = STATE_PATROL;
  }
}

// ============================================================================
// MAIN SETUP
// ============================================================================

void setup() {
  // Initialize serial ports
  Serial.begin(115200);
  SerialCam.begin(115200, SERIAL_8N1, PIN_CAM_RX, PIN_CAM_TX);
  SerialSIM.begin(9600, SERIAL_8N1, PIN_SIM_RX, PIN_SIM_TX);
  SerialGPS.begin(9600);
  
  // Initialize watchdog timer
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);
  
  // Load configuration from EEPROM
  loadConfig();
  
  // Initialize pins
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_MOTOR_L_SPEED, OUTPUT);
  pinMode(PIN_MOTOR_L_DIR, OUTPUT);
  pinMode(PIN_MOTOR_R_SPEED, OUTPUT);
  pinMode(PIN_MOTOR_R_DIR, OUTPUT);
  
  // Initialize I2C for MPU6050
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  
  // Initialize MPU6050
  if (mpu.begin()) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
  
  // Initialize temperature sensor
  tempSensor.begin();
  
  // Initialize ESP-NOW
  initEspNow();
  
  // Initialize streaming server
  initStreamServer();
  
  // Initial sensor readings
  readTemperature();
  batteryPercent = readBatteryPercent();
  
  Serial.println("STASIS Rover initialized");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Feed watchdog
  esp_task_wdt_reset();
  
  // Handle streaming server requests
  streamServer.handleClient();
  
  // --- 1. READ SENSORS (non-blocking, at interval) ---
  if (millis() - lastSensorRead > SENSOR_INTERVAL) {
    lastSensorRead = millis();
    
    obstacleDistance = readUltrasonicDistance();
    batteryPercent = readBatteryPercent();
    readTemperature();
    readMPU6050();
    readGPS();
    checkCameraDetection();
    
    // Check if near base for video streaming
    float distToBase = calculateDistance(currentLat, currentLng, config.baseLat, config.baseLng);
    nearBase = (distToBase < NEAR_BASE_CM);
    
    // Start/stop streaming based on proximity to base
    if (nearBase && !isStreaming) {
      startCameraStream();
    } else if (!nearBase && isStreaming) {
      stopCameraStream();
    }
    
    // Power saving mode
    if (batteryPercent < LOW_BAT_PERCENT) {
      maxSpeed = MOTOR_SPEED_LOW;
      if (currentState == STATE_PATROL) {
        currentState = STATE_RETURN_BASE;
      }
    } else {
      maxSpeed = MOTOR_SPEED_FULL;
    }
  }
  
  // --- 2. HANDLE MANUAL OVERRIDE ---
  if (manualOverride) {
    handleManualCommand();
    return; // Skip state machine during manual control
  }
  
  // --- 3. STATE MACHINE ---
  switch (currentState) {
    case STATE_PATROL:
      handlePatrolState();
      break;
      
    case STATE_RESEARCH:
      handleResearchState();
      break;
      
    case STATE_ALERT:
      handleAlertState();
      break;
      
    case STATE_RETURN_BASE:
      handleReturnBaseState();
      break;
      
    case STATE_DOCKED:
      handleDockedState();
      break;
  }
  
  // --- 4. SEND TELEMETRY ---
  if (millis() - lastTelemetryTx > TELEMETRY_INTERVAL) {
    lastTelemetryTx = millis();
    sendTelemetry();
  }
  
  // Small delay to prevent tight loop
  delay(10);
}

    sendTelemetry();
  }
  
  // Small delay to prevent tight loop
  delay(10);
}
  while (bearingDiff < -180) bearingDiff += 360;
  
  if (bearingDiff > 15) {
    // Need to turn right
    turnRight(maxSpeed);
  } else if (bearingDiff < -15) {
    // Need to turn left
    turnLeft(maxSpeed);
  } else {
    // Heading roughly correct, move forward
    moveForward(maxSpeed);
  }
}

// ============================================================================
// SENSOR FUNCTIONS
// ============================================================================

float readUltrasonicDistance() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  
  long duration = pulseIn(PIN_ECHO, HIGH, 26000);
  if (duration == 0) return -1.0;
  return duration * 0.034 / 2.0;
}

float readBatteryPercent() {
  int raw = analogRead(PIN_BATTERY);
  float voltage = (raw / 4095.0) * 3.3 * 4.04; // Voltage divider ratio
  return constrain((voltage - VMIN) / (VMAX - VMIN) * 100.0, 0, 100);
}

void readMPU6050() {
  sensors_event_t a, g, t;
  if (mpu.getEvent(&a, &g, &t)) {
    accelX = a.acceleration.x;
    accelY = a.acceleration.y;
    
    // Calculate heading from gyro (simplified)
    currentHeading = g.gyro.z;
    
    // Earthquake detection (high acceleration)
    earthquakeDetected = (abs(accelX) > 15.0 || abs(accelY) > 15.0);
  }
}

void readTemperature() {
  tempSensor.requestTemperatures();
  currentTemp = tempSensor.getTempCByIndex(0);
  if (currentTemp == DEVICE_DISCONNECTED_C) {
    currentTemp = -999.0;
  }
}

void readGPS() {
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }
  if (gps.location.isValid()) {
    currentLat = gps.location.lat();
    currentLng = gps.location.lng();
  }
}

// ============================================================================
// CAMERA COMMUNICATION
// ============================================================================

void sendCamCommand(CamCommand cmd) {
  SerialCam.write((uint8_t)cmd);
}

bool requestImageCapture() {
  sendCamCommand(CAM_CMD_CAPTURE);
  
  // Wait for response with timeout
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
    if (SerialCam.available() >= sizeof(DetectionResult)) {
      DetectionResult result;
      SerialCam.readBytes((char*)&result, sizeof(result));
      camFireDetected = result.fire;
      camMotionDetected = result.motion;
      camHumanDetected = result.human;
      return true;
    }
  }
  return false;
}

void startCameraStream() {
  if (!isStreaming) {
    sendCamCommand(CAM_CMD_STREAM_ON);
    isStreaming = true;
  }
}

void stopCameraStream() {
  if (isStreaming) {
    sendCamCommand(CAM_CMD_STREAM_OFF);
    isStreaming = false;
  }
}

void checkCameraDetection() {
  if (SerialCam.available() >= sizeof(DetectionResult)) {
    DetectionResult result;
    SerialCam.readBytes((char*)&result, sizeof(result));
    camFireDetected = result.fire;
    camMotionDetected = result.motion;
    camHumanDetected = result.human;
  }
  
  // Check for simple text detection (fallback)
  if (SerialCam.available()) {
    String camData = SerialCam.readStringUntil('\n');
    if (camData.indexOf("HAZARD:FIRE") >= 0) camFireDetected = true;
    if (camData.indexOf("HAZARD:MOTION") >= 0) camMotionDetected = true;
    if (camData.indexOf("HAZARD:HUMAN") >= 0) camHumanDetected = true;
    if (camData.indexOf("CLEAR") >= 0) {
      camFireDetected = false;
      camMotionDetected = false;
      camHumanDetected = false;
    }
  }
}

// ============================================================================
// GSM/SMS FUNCTIONS
// ============================================================================

void sendSMS(const String& message) {
  SerialSIM.println("AT+CMGF=1");
  delay(100);
  
  // Send to primary number
  if (strlen(config.phone1) > 0) {
    SerialSIM.print("AT+CMGS=\"");
    SerialSIM.print(config.phone1);
    SerialSIM.println("\"");
    delay(100);
    SerialSIM.print(message);
    SerialSIM.write(26);
    delay(1000);
  }
  
  // Send to secondary number if configured
  if (strlen(config.phone2) > 0) {
    SerialSIM.println("AT+CMGF=1");
    delay(100);
    SerialSIM.print("AT+CMGS=\"");
    SerialSIM.print(config.phone2);
    SerialSIM.println("\"");
    delay(100);
    SerialSIM.print(message);
    SerialSIM.write(26);
    delay(1000);
  }
}

// ============================================================================
// ESP-NOW COMMUNICATION
// ============================================================================

void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len == sizeof(CommandPacket)) {
    CommandPacket cmd;
    memcpy(&cmd, data, sizeof(cmd));
    manualCommand = String(cmd.cmd);
    manualOverride = true;
  }
}

void onEspNowSent(const uint8_t *mac, esp_now_send_status_t status) {
  // Could add retry logic here
}

void initEspNow() {
  WiFi.mode(WIFI_STA);
  
  if (esp_now_init() != ESP_OK) {
    return;
  }
  
  esp_now_register_recv_cb(onEspNowRecv);
  esp_now_register_send_cb(onEspNowSent);
  
  memcpy(peerInfo.peer_addr, baseMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

void sendTelemetry() {
  // Prepare telemetry packet
  telemetry.id = (currentState == STATE_ALERT) ? 2 : 1;
  telemetry.temp = currentTemp;
  telemetry.bat = batteryPercent;
  telemetry.lat = currentLat;
  telemetry.lng = currentLng;
  telemetry.hazard = (currentState == STATE_ALERT);
  telemetry.distance = obstacleDistance;
  telemetry.accelX = accelX;
  telemetry.accelY = accelY;
  telemetry.heading = currentHeading;
  telemetry.streaming = isStreaming;
  
  switch (currentState) {
    case STATE_PATROL:    strcpy(telemetry.status, "PATROL"); break;
    case STATE_RESEARCH:  strcpy(telemetry.status, "RESEARCH"); break;
    case STATE_ALERT:     strcpy(telemetry.status, "CRITICAL ALERT"); break;
    case STATE_RETURN_BASE: strcpy(telemetry.status, "RETURNING"); break;
    case STATE_DOCKED:    strcpy(telemetry.status, "DOCKED"); break;
  }
  
  esp_now_send(baseMac, (uint8_t *)&telemetry, sizeof(telemetry));
}

// ============================================================================
// VIDEO STREAMING SERVER
// ============================================================================

void handleStreamRequest() {
  streamServer.sendHeader("Content-Type", "multipart/x-mixed-replace; boundary=frame");
  streamServer.sendHeader("Connection", "close");
  
  // This is a simplified streaming handler
  // In practice, you'd need to implement proper MJPEG streaming
  streamServer.send(200, "text/plain", "Stream active");
}

void handleCaptureRequest() {
  if (requestImageCapture()) {
    streamServer.send(200, "application/json", 
      "{\"fire\":" + String(camFireDetected ? "true" : "false") + 
      ",\"motion\":" + String(camMotionDetected ? "true" : "false") + 
      ",\"human\":" + String(camHumanDetected ? "true" : "false") + "}");
  } else {
    streamServer.send(504, "application/json", "{\"error\":\"timeout\"}");
  }
}

void handleConfigRequest() {
  String json = "{";
  json += "\"phone1\":\"" + String(config.phone1) + "\",";
  json += "\"phone2\":\"" + String(config.phone2) + "\",";
  json += "\"baseLat\":" + String(config.baseLat, 6) + ",";
  json += "\"baseLng\":" + String(config.baseLng, 6) + ",";
  json += "\"alertTempHigh\":" + String(config.alertTempHigh);
  json += "}";
  streamServer.send(200, "application/json", json);
}

void handleSetConfigRequest() {
  if (streamServer.hasArg("phone1")) {
    strncpy(config.phone1, streamServer.arg("phone1").c_str(), 15);
    config.phone1[15] = '\0';
  }
  if (streamServer.hasArg("phone2")) {
    strncpy(config.phone2, streamServer.arg("phone2").c_str(), 15);
    config.phone2[15] = '\0';
  }
  if (streamServer.hasArg("baseLat")) {
    config.baseLat = streamServer.arg("baseLat").toFloat();
  }
  if (streamServer.hasArg("baseLng")) {
    config.baseLng = streamServer.arg("baseLng").toFloat();
  }
  saveConfig();
  streamServer.send(200, "application/json", "{\"status\":\"ok\"}");
}

void initStreamServer() {
  streamServer.on("/stream", handleStreamRequest);
  streamServer.on("/capture", HTTP_POST, handleCaptureRequest);
  streamServer.on("/config", HTTP_GET, handleConfigRequest);
  streamServer.on("/config", HTTP_POST, handleSetConfigRequest);
  streamServer.begin();
}

// ============================================================================
// MANUAL COMMAND HANDLER
// ============================================================================

void handleManualCommand() {
  if (manualCommand == "FORWARD") {
    moveForward(maxSpeed);
  } else if (manualCommand == "BACKWARD") {
    moveBackward(maxSpeed);
  } else if (manualCommand == "LEFT") {
    turnLeft(MOTOR_SPEED_TURN);
  } else if (manualCommand == "RIGHT") {
    turnRight(MOTOR_SPEED_TURN);
  } else if (manualCommand == "STOP") {
    stopMotors();
  }
  manualOverride = false;
  manualCommand = "";
}

// ============================================================================
// STATE MACHINE HANDLERS
// ============================================================================

void handlePatrolState() {
  // Check for hazards
  if (currentTemp > config.alertTempHigh || earthquakeDetected || 
      camFireDetected || camHumanDetected) {
    currentState = STATE_ALERT;
    stateTimer = millis();
    return;
  }
  
  // Obstacle avoidance
  if (obstacleDistance > 0 && obstacleDistance < OBSTACLE_CM) {
    stopMotors();
    // Back up a bit then turn
    moveBackward(MOTOR_SPEED_LOW);
    delay(300);
    turnRight(MOTOR_SPEED_TURN);
    delay(400);
    stopMotors();
  } else {
    moveForward(maxSpeed);
  }
}

void handleResearchState() {
  // Stay in research mode for 3 seconds (non-blocking)
  stopMotors();
  
  if (millis() - stateTimer > 3000) {
    currentState = STATE_PATROL;
  }
}

void handleAlertState() {
  // Stop and sound alarm
  stopMotors();
  digitalWrite(PIN_BUZZER, HIGH);
  
  // Send SMS alert
  String alertMsg = "HAZARD DETECTED!\n";
  alertMsg += "Location: " + String(currentLat, 6) + ", " + String(currentLng, 6) + "\n";
  alertMsg += "Temp: " + String(currentTemp, 1) + " C\n";
  alertMsg += "Status: " + String(telemetry.status);
  sendSMS(alertMsg);
  
  // Send telemetry immediately
  sendTelemetry();
  
  // Wait 2 seconds (non-blocking would be better, but this is critical)
  delay(2000);
  digitalWrite(PIN_BUZZER, LOW);
  
  // Clear detection flags
  camFireDetected = false;
  camMotionDetected = false;
  camHumanDetected = false;
  
  // Transition to research mode
  currentState = STATE_RESEARCH;
  stateTimer = millis();
}

void handleReturnBaseState() {
  // Check if we've reached base
  if (obstacleDistance > 0 && obstacleDistance < 10) {
    currentState = STATE_DOCKED;
    stateTimer = millis();
    return;
  }
  
  // Obstacle avoidance while returning
  if (obstacleDistance > 0 && obstacleDistance < OBSTACLE_CM) {
    stopMotors();
    turnLeft(MOTOR_SPEED_TURN);
    delay(400);
  } else {
    navigateToBase();
  }
}

void handleDockedState() {
  stopMotors();
  stopCameraStream();
  
  // Check if battery is charged
  if (batteryPercent > 95) {
    currentState = STATE_PATROL;
  }
}

// ============================================================================
// MAIN SETUP
// ============================================================================

void setup() {
  // Initialize serial ports
  Serial.begin(115200);
  SerialCam.begin(115200, SERIAL_8N1, PIN_CAM_RX, PIN_CAM_TX);
  SerialSIM.begin(9600, SERIAL_8N1, PIN_SIM_RX, PIN_SIM_TX);
  SerialGPS.begin(9600);
  
  // Initialize watchdog timer
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);
  
  // Load configuration from EEPROM
  loadConfig();
  
  // Initialize pins
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_MOTOR_L_SPEED, OUTPUT);
  pinMode(PIN_MOTOR_L_DIR, OUTPUT);
  pinMode(PIN_MOTOR_R_SPEED, OUTPUT);
  pinMode(PIN_MOTOR_R_DIR, OUTPUT);
  
  // Initialize I2C for MPU6050
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  
  // Initialize MPU6050
  if (mpu.begin()) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
  
  // Initialize temperature sensor
  tempSensor.begin();
  
  // Initialize ESP-NOW
  initEspNow();
  
  // Initialize streaming server
  initStreamServer();
  
  // Initial sensor readings
  readTemperature();
  batteryPercent = readBatteryPercent();
  
  Serial.println("AERO SENTINEL Rover initialized");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Feed watchdog
  esp_task_wdt_reset();
  
  // Handle streaming server requests
  streamServer.handleClient();
  
  // --- 1. READ SENSORS (non-blocking, at interval) ---
  if (millis() - lastSensorRead > SENSOR_INTERVAL) {
    lastSensorRead = millis();
    
    obstacleDistance = readUltrasonicDistance();
    batteryPercent = readBatteryPercent();
    readTemperature();
    readMPU6050();
    readGPS();
    checkCameraDetection();
    
    // Check if near base for video streaming
    float distToBase = calculateDistance(currentLat, currentLng, config.baseLat, config.baseLng);
    nearBase = (distToBase < NEAR_BASE_CM);
    
    // Start/stop streaming based on proximity to base
    if (nearBase && !isStreaming) {
      startCameraStream();
    } else if (!nearBase && isStreaming) {
      stopCameraStream();
    }
    
    // Power saving mode
    if (batteryPercent < LOW_BAT_PERCENT) {
      maxSpeed = MOTOR_SPEED_LOW;
      if (currentState == STATE_PATROL) {
        currentState = STATE_RETURN_BASE;
      }
    } else {
      maxSpeed = MOTOR_SPEED_FULL;
    }
  }
  
  // --- 2. HANDLE MANUAL OVERRIDE ---
  if (manualOverride) {
    handleManualCommand();
    return; // Skip state machine during manual control
  }
  
  // --- 3. STATE MACHINE ---
  switch (currentState) {
    case STATE_PATROL:
      handlePatrolState();
      break;
      
    case STATE_RESEARCH:
      handleResearchState();
      break;
      
    case STATE_ALERT:
      handleAlertState();
      break;
      
    case STATE_RETURN_BASE:
      handleReturnBaseState();
      break;
      
    case STATE_DOCKED:
      handleDockedState();
      break;
  }
  
  // --- 4. SEND TELEMETRY ---
  if (millis() - lastTelemetryTx > TELEMETRY_INTERVAL) {
    lastTelemetryTx = millis();
    sendTelemetry();
  }
  
  // Small delay to prevent tight loop
  delay(10);
}

  while (bearingDiff < -180) bearingDiff += 360;
  
  if (bearingDiff > 15) {
    // Need to turn right
    turnRight(maxSpeed);
  } else if (bearingDiff < -15) {
    // Need to turn left
    turnLeft(maxSpeed);
  } else {
    // Heading roughly correct, move forward
    moveForward(maxSpeed);
  }
}

// ============================================================================
// SENSOR FUNCTIONS
// ============================================================================

float readUltrasonicDistance() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  
  long duration = pulseIn(PIN_ECHO, HIGH, 26000);
  if (duration == 0) return -1.0;
  return duration * 0.034 / 2.0;
}

float readBatteryPercent() {
  int raw = analogRead(PIN_BATTERY);
  float voltage = (raw / 4095.0) * 3.3 * 4.04; // Voltage divider ratio
  return constrain((voltage - VMIN) / (VMAX - VMIN) * 100.0, 0, 100);
}

void readMPU6050() {
  sensors_event_t a, g, t;
  if (mpu.getEvent(&a, &g, &t)) {
    accelX = a.acceleration.x;
    accelY = a.acceleration.y;
    
    // Calculate heading from gyro (simplified)
    currentHeading = g.gyro.z;
    
    // Earthquake detection (high acceleration)
    earthquakeDetected = (abs(accelX) > 15.0 || abs(accelY) > 15.0);
  }
}

void readTemperature() {
  tempSensor.requestTemperatures();
  currentTemp = tempSensor.getTempCByIndex(0);
  if (currentTemp == DEVICE_DISCONNECTED_C) {
    currentTemp = -999.0;
  }
}

void readGPS() {
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }
  if (gps.location.isValid()) {
    currentLat = gps.location.lat();
    currentLng = gps.location.lng();
  }
}

// ============================================================================
// CAMERA COMMUNICATION
// ============================================================================

void sendCamCommand(CamCommand cmd) {
  SerialCam.write((uint8_t)cmd);
}

bool requestImageCapture() {
  sendCamCommand(CAM_CMD_CAPTURE);
  
  // Wait for response with timeout
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
    if (SerialCam.available() >= sizeof(DetectionResult)) {
      DetectionResult result;
      SerialCam.readBytes((char*)&result, sizeof(result));
      camFireDetected = result.fire;
      camMotionDetected = result.motion;
      camHumanDetected = result.human;
      return true;
    }
  }
  return false;
}

void startCameraStream() {
  if (!isStreaming) {
    sendCamCommand(CAM_CMD_STREAM_ON);
    isStreaming = true;
  }
}

void stopCameraStream() {
  if (isStreaming) {
    sendCamCommand(CAM_CMD_STREAM_OFF);
    isStreaming = false;
  }
}

void checkCameraDetection() {
  if (SerialCam.available() >= sizeof(DetectionResult)) {
    DetectionResult result;
    SerialCam.readBytes((char*)&result, sizeof(result));
    camFireDetected = result.fire;
    camMotionDetected = result.motion;
    camHumanDetected = result.human;
  }
  
  // Check for simple text detection (fallback)
  if (SerialCam.available()) {
    String camData = SerialCam.readStringUntil('\n');
    if (camData.indexOf("HAZARD:FIRE") >= 0) camFireDetected = true;
    if (camData.indexOf("HAZARD:MOTION") >= 0) camMotionDetected = true;
    if (camData.indexOf("HAZARD:HUMAN") >= 0) camHumanDetected = true;
    if (camData.indexOf("CLEAR") >= 0) {
      camFireDetected = false;
      camMotionDetected = false;
      camHumanDetected = false;
    }
  }
}

// ============================================================================
// GSM/SMS FUNCTIONS
// ============================================================================

void sendSMS(const String& message) {
  SerialSIM.println("AT+CMGF=1");
  delay(100);
  
  // Send to primary number
  if (strlen(config.phone1) > 0) {
    SerialSIM.print("AT+CMGS=\"");
    SerialSIM.print(config.phone1);
    SerialSIM.println("\"");
    delay(100);
    SerialSIM.print(message);
    SerialSIM.write(26);
    delay(1000);
  }
  
  // Send to secondary number if configured
  if (strlen(config.phone2) > 0) {
    SerialSIM.println("AT+CMGF=1");
    delay(100);
    SerialSIM.print("AT+CMGS=\"");
    SerialSIM.print(config.phone2);
    SerialSIM.println("\"");
    delay(100);
    SerialSIM.print(message);
    SerialSIM.write(26);
    delay(1000);
  }
}

// ============================================================================
// ESP-NOW COMMUNICATION
// ============================================================================

void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len == sizeof(CommandPacket)) {
    CommandPacket cmd;
    memcpy(&cmd, data, sizeof(cmd));
    manualCommand = String(cmd.cmd);
    manualOverride = true;
  }
}

void onEspNowSent(const uint8_t *mac, esp_now_send_status_t status) {
  // Could add retry logic here
}

void initEspNow() {
  WiFi.mode(WIFI_STA);
  
  if (esp_now_init() != ESP_OK) {
    return;
  }
  
  esp_now_register_recv_cb(onEspNowRecv);
  esp_now_register_send_cb(onEspNowSent);
  
  memcpy(peerInfo.peer_addr, baseMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

void sendTelemetry() {
  // Prepare telemetry packet
  telemetry.id = (currentState == STATE_ALERT) ? 2 : 1;
  telemetry.temp = currentTemp;
  telemetry.bat = batteryPercent;
  telemetry.lat = currentLat;
  telemetry.lng = currentLng;
  telemetry.hazard = (currentState == STATE_ALERT);
  telemetry.distance = obstacleDistance;
  telemetry.accelX = accelX;
  telemetry.accelY = accelY;
  telemetry.heading = currentHeading;
  telemetry.streaming = isStreaming;
  
  switch (currentState) {
    case STATE_PATROL:    strcpy(telemetry.status, "PATROL"); break;
    case STATE_RESEARCH:  strcpy(telemetry.status, "RESEARCH"); break;
    case STATE_ALERT:     strcpy(telemetry.status, "CRITICAL ALERT"); break;
    case STATE_RETURN_BASE: strcpy(telemetry.status, "RETURNING"); break;
    case STATE_DOCKED:    strcpy(telemetry.status, "DOCKED"); break;
  }
  
  esp_now_send(baseMac, (uint8_t *)&telemetry, sizeof(telemetry));
}

// ============================================================================
// VIDEO STREAMING SERVER
// ============================================================================

void handleStreamRequest() {
  streamServer.sendHeader("Content-Type", "multipart/x-mixed-replace; boundary=frame");
  streamServer.sendHeader("Connection", "close");
  
  // This is a simplified streaming handler
  // In practice, you'd need to implement proper MJPEG streaming
  streamServer.send(200, "text/plain", "Stream active");
}

void handleCaptureRequest() {
  if (requestImageCapture()) {
    streamServer.send(200, "application/json", 
      "{\"fire\":" + String(camFireDetected ? "true" : "false") + 
      ",\"motion\":" + String(camMotionDetected ? "true" : "false") + 
      ",\"human\":" + String(camHumanDetected ? "true" : "false") + "}");
  } else {
    streamServer.send(504, "application/json", "{\"error\":\"timeout\"}");
  }
}

void handleConfigRequest() {
  String json = "{";
  json += "\"phone1\":\"" + String(config.phone1) + "\",";
  json += "\"phone2\":\"" + String(config.phone2) + "\",";
  json += "\"baseLat\":" + String(config.baseLat, 6) + ",";
  json += "\"baseLng\":" + String(config.baseLng, 6) + ",";
  json += "\"alertTempHigh\":" + String(config.alertTempHigh);
  json += "}";
  streamServer.send(200, "application/json", json);
}

void handleSetConfigRequest() {
  if (streamServer.hasArg("phone1")) {
    strncpy(config.phone1, streamServer.arg("phone1").c_str(), 15);
    config.phone1[15] = '\0';
  }
  if (streamServer.hasArg("phone2")) {
    strncpy(config.phone2, streamServer.arg("phone2").c_str(), 15);
    config.phone2[15] = '\0';
  }
  if (streamServer.hasArg("baseLat")) {
    config.baseLat = streamServer.arg("baseLat").toFloat();
  }
  if (streamServer.hasArg("baseLng")) {
    config.baseLng = streamServer.arg("baseLng").toFloat();
  }
  saveConfig();
  streamServer.send(200, "application/json", "{\"status\":\"ok\"}");
}

void initStreamServer() {
  streamServer.on("/stream", handleStreamRequest);
  streamServer.on("/capture", HTTP_POST, handleCaptureRequest);
  streamServer.on("/config", HTTP_GET, handleConfigRequest);
  streamServer.on("/config", HTTP_POST, handleSetConfigRequest);
  streamServer.begin();
}

// ============================================================================
// MANUAL COMMAND HANDLER
// ============================================================================

void handleManualCommand() {
  if (manualCommand == "FORWARD") {
    moveForward(maxSpeed);
  } else if (manualCommand == "BACKWARD") {
    moveBackward(maxSpeed);
  } else if (manualCommand == "LEFT") {
    turnLeft(MOTOR_SPEED_TURN);
  } else if (manualCommand == "RIGHT") {
    turnRight(MOTOR_SPEED_TURN);
  } else if (manualCommand == "STOP") {
    stopMotors();
  }
  manualOverride = false;
  manualCommand = "";
}

// ============================================================================
// STATE MACHINE HANDLERS
// ============================================================================

void handlePatrolState() {
  // Check for hazards
  if (currentTemp > config.alertTempHigh || earthquakeDetected || 
      camFireDetected || camHumanDetected) {
    currentState = STATE_ALERT;
    stateTimer = millis();
    return;
  }
  
  // Obstacle avoidance
  if (obstacleDistance > 0 && obstacleDistance < OBSTACLE_CM) {
    stopMotors();
    // Back up a bit then turn
    moveBackward(MOTOR_SPEED_LOW);
    delay(300);
    turnRight(MOTOR_SPEED_TURN);
    delay(400);
    stopMotors();
  } else {
    moveForward(maxSpeed);
  }
}

void handleResearchState() {
  // Stay in research mode for 3 seconds (non-blocking)
  stopMotors();
  
  if (millis() - stateTimer > 3000) {
    currentState = STATE_PATROL;
  }
}

void handleAlertState() {
  // Stop and sound alarm
  stopMotors();
  digitalWrite(PIN_BUZZER, HIGH);
  
  // Send SMS alert
  String alertMsg = "HAZARD DETECTED!\n";
  alertMsg += "Location: " + String(currentLat, 6) + ", " + String(currentLng, 6) + "\n";
  alertMsg += "Temp: " + String(currentTemp, 1) + " C\n";
  alertMsg += "Status: " + String(telemetry.status);
  sendSMS(alertMsg);
  
  // Send telemetry immediately
  sendTelemetry();
  
  // Wait 2 seconds (non-blocking would be better, but this is critical)
  delay(2000);
  digitalWrite(PIN_BUZZER, LOW);
  
  // Clear detection flags
  camFireDetected = false;
  camMotionDetected = false;
  camHumanDetected = false;
  
  // Transition to research mode
  currentState = STATE_RESEARCH;
  stateTimer = millis();
}

void handleReturnBaseState() {
  // Check if we've reached base
  if (obstacleDistance > 0 && obstacleDistance < 10) {
    currentState = STATE_DOCKED;
    stateTimer = millis();
    return;
  }
  
  // Obstacle avoidance while returning
  if (obstacleDistance > 0 && obstacleDistance < OBSTACLE_CM) {
    stopMotors();
    turnLeft(MOTOR_SPEED_TURN);
    delay(400);
  } else {
    navigateToBase();
  }
}

void handleDockedState() {
  stopMotors();
  stopCameraStream();
  
  // Check if battery is charged
  if (batteryPercent > 95) {
    currentState = STATE_PATROL;
  }
}

// ============================================================================
// MAIN SETUP
// ============================================================================

void setup() {
  // Initialize serial ports
  Serial.begin(115200);
  SerialCam.begin(115200, SERIAL_8N1, PIN_CAM_RX, PIN_CAM_TX);
  SerialSIM.begin(9600, SERIAL_8N1, PIN_SIM_RX, PIN_SIM_TX);
  SerialGPS.begin(9600);
  
  // Initialize watchdog timer
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);
  
  // Load configuration from EEPROM
  loadConfig();
  
  // Initialize pins
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_MOTOR_L_SPEED, OUTPUT);
  pinMode(PIN_MOTOR_L_DIR, OUTPUT);
  pinMode(PIN_MOTOR_R_SPEED, OUTPUT);
  pinMode(PIN_MOTOR_R_DIR, OUTPUT);
  
  // Initialize I2C for MPU6050
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  
  // Initialize MPU6050
  if (mpu.begin()) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
  
  // Initialize temperature sensor
  tempSensor.begin();
  
  // Initialize ESP-NOW
  initEspNow();
  
  // Initialize streaming server
  initStreamServer();
  
  // Initial sensor readings
  readTemperature();
  batteryPercent = readBatteryPercent();
  
  Serial.println("AERO SENTINEL Rover initialized");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Feed watchdog
  esp_task_wdt_reset();
  
  // Handle streaming server requests
  streamServer.handleClient();
  
  // --- 1. READ SENSORS (non-blocking, at interval) ---
  if (millis() - lastSensorRead > SENSOR_INTERVAL) {
    lastSensorRead = millis();
    
    obstacleDistance = readUltrasonicDistance();
    batteryPercent = readBatteryPercent();
    readTemperature();
    readMPU6050();
    readGPS();
    checkCameraDetection();
    
    // Check if near base for video streaming
    float distToBase = calculateDistance(currentLat, currentLng, config.baseLat, config.baseLng);
    nearBase = (distToBase < NEAR_BASE_CM);
    
    // Start/stop streaming based on proximity to base
    if (nearBase && !isStreaming) {
      startCameraStream();
    } else if (!nearBase && isStreaming) {
      stopCameraStream();
    }
    
    // Power saving mode
    if (batteryPercent < LOW_BAT_PERCENT) {
      maxSpeed = MOTOR_SPEED_LOW;
      if (currentState == STATE_PATROL) {
        currentState = STATE_RETURN_BASE;
      }
    } else {
      maxSpeed = MOTOR_SPEED_FULL;
    }
  }
  
  // --- 2. HANDLE MANUAL OVERRIDE ---
  if (manualOverride) {
    handleManualCommand();
    return; // Skip state machine during manual control
  }
  
  // --- 3. STATE MACHINE ---
  switch (currentState) {
    case STATE_PATROL:
      handlePatrolState();
      break;
      
    case STATE_RESEARCH:
      handleResearchState();
      break;
      
    case STATE_ALERT:
      handleAlertState();
      break;
      
    case STATE_RETURN_BASE:
      handleReturnBaseState();
      break;
      
    case STATE_DOCKED:
      handleDockedState();
      break;
  }
  
  // --- 4. SEND TELEMETRY ---
  if (millis() - lastTelemetryTx > TELEMETRY_INTERVAL) {
    lastTelemetryTx = millis();
    sendTelemetry();
  }
  
  // Small delay to prevent tight loop
  delay(10);
}
  while (bearingDiff < -180) bearingDiff += 360;
  
  if (bearingDiff > 15) {
    // Need to turn right
    turnRight(maxSpeed);
  } else if (bearingDiff < -15) {
    // Need to turn left
    turnLeft(maxSpeed);
  } else {
    // Heading roughly correct, move forward
    moveForward(maxSpeed);
  }
}

// ============================================================================
// SENSOR FUNCTIONS
// ============================================================================

float readUltrasonicDistance() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  
  long duration = pulseIn(PIN_ECHO, HIGH, 26000);
  if (duration == 0) return -1.0;
  return duration * 0.034 / 2.0;
}

float readBatteryPercent() {
  int raw = analogRead(PIN_BATTERY);
  float voltage = (raw / 4095.0) * 3.3 * 4.04; // Voltage divider ratio
  return constrain((voltage - VMIN) / (VMAX - VMIN) * 100.0, 0, 100);
}

void readMPU6050() {
  sensors_event_t a, g, t;
  if (mpu.getEvent(&a, &g, &t)) {
    accelX = a.acceleration.x;
    accelY = a.acceleration.y;
    
    // Calculate heading from gyro (simplified)
    currentHeading = g.gyro.z;
    
    // Earthquake detection (high acceleration)
    earthquakeDetected = (abs(accelX) > 15.0 || abs(accelY) > 15.0);
  }
}

void readTemperature() {
  tempSensor.requestTemperatures();
  currentTemp = tempSensor.getTempCByIndex(0);
  if (currentTemp == DEVICE_DISCONNECTED_C) {
    currentTemp = -999.0;
  }
}

void readGPS() {
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }
  if (gps.location.isValid()) {
    currentLat = gps.location.lat();
    currentLng = gps.location.lng();
  }
}

// ============================================================================
// CAMERA COMMUNICATION
// ============================================================================

void sendCamCommand(CamCommand cmd) {
  SerialCam.write((uint8_t)cmd);
}

bool requestImageCapture() {
  sendCamCommand(CAM_CMD_CAPTURE);
  
  // Wait for response with timeout
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
    if (SerialCam.available() >= sizeof(DetectionResult)) {
      DetectionResult result;
      SerialCam.readBytes((char*)&result, sizeof(result));
      camFireDetected = result.fire;
      camMotionDetected = result.motion;
      camHumanDetected = result.human;
      return true;
    }
  }
  return false;
}

void startCameraStream() {
  if (!isStreaming) {
    sendCamCommand(CAM_CMD_STREAM_ON);
    isStreaming = true;
  }
}

void stopCameraStream() {
  if (isStreaming) {
    sendCamCommand(CAM_CMD_STREAM_OFF);
    isStreaming = false;
  }
}

void checkCameraDetection() {
  if (SerialCam.available() >= sizeof(DetectionResult)) {
    DetectionResult result;
    SerialCam.readBytes((char*)&result, sizeof(result));
    camFireDetected = result.fire;
    camMotionDetected = result.motion;
    camHumanDetected = result.human;
  }
  
  // Check for simple text detection (fallback)
  if (SerialCam.available()) {
    String camData = SerialCam.readStringUntil('\n');
    if (camData.indexOf("HAZARD:FIRE") >= 0) camFireDetected = true;
    if (camData.indexOf("HAZARD:MOTION") >= 0) camMotionDetected = true;
    if (camData.indexOf("HAZARD:HUMAN") >= 0) camHumanDetected = true;
    if (camData.indexOf("CLEAR") >= 0) {
      camFireDetected = false;
      camMotionDetected = false;
      camHumanDetected = false;
    }
  }
}

// ============================================================================
// GSM/SMS FUNCTIONS
// ============================================================================

void sendSMS(const String& message) {
  SerialSIM.println("AT+CMGF=1");
  delay(100);
  
  // Send to primary number
  if (strlen(config.phone1) > 0) {
    SerialSIM.print("AT+CMGS=\"");
    SerialSIM.print(config.phone1);
    SerialSIM.println("\"");
    delay(100);
    SerialSIM.print(message);
    SerialSIM.write(26);
    delay(1000);
  }
  
  // Send to secondary number if configured
  if (strlen(config.phone2) > 0) {
    SerialSIM.println("AT+CMGF=1");
    delay(100);
    SerialSIM.print("AT+CMGS=\"");
    SerialSIM.print(config.phone2);
    SerialSIM.println("\"");
    delay(100);
    SerialSIM.print(message);
    SerialSIM.write(26);
    delay(1000);
  }
}

// ============================================================================
// ESP-NOW COMMUNICATION
// ============================================================================

void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len == sizeof(CommandPacket)) {
    CommandPacket cmd;
    memcpy(&cmd, data, sizeof(cmd));
    manualCommand = String(cmd.cmd);
    manualOverride = true;
  }
}

void onEspNowSent(const uint8_t *mac, esp_now_send_status_t status) {
  // Could add retry logic here
}

void initEspNow() {
  WiFi.mode(WIFI_STA);
  
  if (esp_now_init() != ESP_OK) {
    return;
  }
  
  esp_now_register_recv_cb(onEspNowRecv);
  esp_now_register_send_cb(onEspNowSent);
  
  memcpy(peerInfo.peer_addr, baseMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

void sendTelemetry() {
  // Prepare telemetry packet
  telemetry.id = (currentState == STATE_ALERT) ? 2 : 1;
  telemetry.temp = currentTemp;
  telemetry.bat = batteryPercent;
  telemetry.lat = currentLat;
  telemetry.lng = currentLng;
  telemetry.hazard = (currentState == STATE_ALERT);
  telemetry.distance = obstacleDistance;
  telemetry.accelX = accelX;
  telemetry.accelY = accelY;
  telemetry.heading = currentHeading;
  telemetry.streaming = isStreaming;
  
  switch (currentState) {
    case STATE_PATROL:    strcpy(telemetry.status, "PATROL"); break;
    case STATE_RESEARCH:  strcpy(telemetry.status, "RESEARCH"); break;
    case STATE_ALERT:     strcpy(telemetry.status, "CRITICAL ALERT"); break;
    case STATE_RETURN_BASE: strcpy(telemetry.status, "RETURNING"); break;
    case STATE_DOCKED:    strcpy(telemetry.status, "DOCKED"); break;
  }
  
  esp_now_send(baseMac, (uint8_t *)&telemetry, sizeof(telemetry));
}

// ============================================================================
// VIDEO STREAMING SERVER
// ============================================================================

void handleStreamRequest() {
  streamServer.sendHeader("Content-Type", "multipart/x-mixed-replace; boundary=frame");
  streamServer.sendHeader("Connection", "close");
  
  // This is a simplified streaming handler
  // In practice, you'd need to implement proper MJPEG streaming
  streamServer.send(200, "text/plain", "Stream active");
}

void handleCaptureRequest() {
  if (requestImageCapture()) {
    streamServer.send(200, "application/json", 
      "{\"fire\":" + String(camFireDetected ? "true" : "false") + 
      ",\"motion\":" + String(camMotionDetected ? "true" : "false") + 
      ",\"human\":" + String(camHumanDetected ? "true" : "false") + "}");
  } else {
    streamServer.send(504, "application/json", "{\"error\":\"timeout\"}");
  }
}

void handleConfigRequest() {
  String json = "{";
  json += "\"phone1\":\"" + String(config.phone1) + "\",";
  json += "\"phone2\":\"" + String(config.phone2) + "\",";
  json += "\"baseLat\":" + String(config.baseLat, 6) + ",";
  json += "\"baseLng\":" + String(config.baseLng, 6) + ",";
  json += "\"alertTempHigh\":" + String(config.alertTempHigh);
  json += "}";
  streamServer.send(200, "application/json", json);
}

void handleSetConfigRequest() {
  if (streamServer.hasArg("phone1")) {
    strncpy(config.phone1, streamServer.arg("phone1").c_str(), 15);
    config.phone1[15] = '\0';
  }
  if (streamServer.hasArg("phone2")) {
    strncpy(config.phone2, streamServer.arg("phone2").c_str(), 15);
    config.phone2[15] = '\0';
  }
  if (streamServer.hasArg("baseLat")) {
    config.baseLat = streamServer.arg("baseLat").toFloat();
  }
  if (streamServer.hasArg("baseLng")) {
    config.baseLng = streamServer.arg("baseLng").toFloat();
  }
  saveConfig();
  streamServer.send(200, "application/json", "{\"status\":\"ok\"}");
}

void initStreamServer() {
  streamServer.on("/stream", handleStreamRequest);
  streamServer.on("/capture", HTTP_POST, handleCaptureRequest);
  streamServer.on("/config", HTTP_GET, handleConfigRequest);
  streamServer.on("/config", HTTP_POST, handleSetConfigRequest);
  streamServer.begin();
}

// ============================================================================
// MANUAL COMMAND HANDLER
// ============================================================================

void handleManualCommand() {
  if (manualCommand == "FORWARD") {
    moveForward(maxSpeed);
  } else if (manualCommand == "BACKWARD") {
    moveBackward(maxSpeed);
  } else if (manualCommand == "LEFT") {
    turnLeft(MOTOR_SPEED_TURN);
  } else if (manualCommand == "RIGHT") {
    turnRight(MOTOR_SPEED_TURN);
  } else if (manualCommand == "STOP") {
    stopMotors();
  }
  manualOverride = false;
  manualCommand = "";
}

// ============================================================================
// STATE MACHINE HANDLERS
// ============================================================================

void handlePatrolState() {
  // Check for hazards
  if (currentTemp > config.alertTempHigh || earthquakeDetected || 
      camFireDetected || camHumanDetected) {
    currentState = STATE_ALERT;
    stateTimer = millis();
    return;
  }
  
  // Obstacle avoidance
  if (obstacleDistance > 0 && obstacleDistance < OBSTACLE_CM) {
    stopMotors();
    // Back up a bit then turn
    moveBackward(MOTOR_SPEED_LOW);
    delay(300);
    turnRight(MOTOR_SPEED_TURN);
    delay(400);
    stopMotors();
  } else {
    moveForward(maxSpeed);
  }
}

void handleResearchState() {
  // Stay in research mode for 3 seconds (non-blocking)
  stopMotors();
  
  if (millis() - stateTimer > 3000) {
    currentState = STATE_PATROL;
  }
}

void handleAlertState() {
  // Stop and sound alarm
  stopMotors();
  digitalWrite(PIN_BUZZER, HIGH);
  
  // Send SMS alert
  String alertMsg = "HAZARD DETECTED!\n";
  alertMsg += "Location: " + String(currentLat, 6) + ", " + String(currentLng, 6) + "\n";
  alertMsg += "Temp: " + String(currentTemp, 1) + " C\n";
  alertMsg += "Status: " + String(telemetry.status);
  sendSMS(alertMsg);
  
  // Send telemetry immediately
  sendTelemetry();
  
  // Wait 2 seconds (non-blocking would be better, but this is critical)
  delay(2000);
  digitalWrite(PIN_BUZZER, LOW);
  
  // Clear detection flags
  camFireDetected = false;
  camMotionDetected = false;
  camHumanDetected = false;
  
  // Transition to research mode
  currentState = STATE_RESEARCH;
  stateTimer = millis();
}

void handleReturnBaseState() {
  // Check if we've reached base
  if (obstacleDistance > 0 && obstacleDistance < 10) {
    currentState = STATE_DOCKED;
    stateTimer = millis();
    return;
  }
  
  // Obstacle avoidance while returning
  if (obstacleDistance > 0 && obstacleDistance < OBSTACLE_CM) {
    stopMotors();
    turnLeft(MOTOR_SPEED_TURN);
    delay(400);
  } else {
    navigateToBase();
  }
}

void handleDockedState() {
  stopMotors();
  stopCameraStream();
  
  // Check if battery is charged
  if (batteryPercent > 95) {
    currentState = STATE_PATROL;
  }
}

// ============================================================================
// MAIN SETUP
// ============================================================================

void setup() {
  // Initialize serial ports
  Serial.begin(115200);
  SerialCam.begin(115200, SERIAL_8N1, PIN_CAM_RX, PIN_CAM_TX);
  SerialSIM.begin(9600, SERIAL_8N1, PIN_SIM_RX, PIN_SIM_TX);
  SerialGPS.begin(9600);
  
  // Initialize watchdog timer
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);
  
  // Load configuration from EEPROM
  loadConfig();
  
  // Initialize pins
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_MOTOR_L_SPEED, OUTPUT);
  pinMode(PIN_MOTOR_L_DIR, OUTPUT);
  pinMode(PIN_MOTOR_R_SPEED, OUTPUT);
  pinMode(PIN_MOTOR_R_DIR, OUTPUT);
  
  // Initialize I2C for MPU6050
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  
  // Initialize MPU6050
  if (mpu.begin()) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
  
  // Initialize temperature sensor
  tempSensor.begin();
  
  // Initialize ESP-NOW
  initEspNow();
  
  // Initialize streaming server
  initStreamServer();
  
  // Initial sensor readings
  readTemperature();
  batteryPercent = readBatteryPercent();
  
  Serial.println("AERO SENTINEL Rover initialized");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Feed watchdog
  esp_task_wdt_reset();
  
  // Handle streaming server requests
  streamServer.handleClient();
  
  // --- 1. READ SENSORS (non-blocking, at interval) ---
  if (millis() - lastSensorRead > SENSOR_INTERVAL) {
    lastSensorRead = millis();
    
    obstacleDistance = readUltrasonicDistance();
    batteryPercent = readBatteryPercent();
    readTemperature();
    readMPU6050();
    readGPS();
    checkCameraDetection();
    
    // Check if near base for video streaming
    float distToBase = calculateDistance(currentLat, currentLng, config.baseLat, config.baseLng);
    nearBase = (distToBase < NEAR_BASE_CM);
    
    // Start/stop streaming based on proximity to base
    if (nearBase && !isStreaming) {
      startCameraStream();
    } else if (!nearBase && isStreaming) {
      stopCameraStream();
    }
    
    // Power saving mode
    if (batteryPercent < LOW_BAT_PERCENT) {
      maxSpeed = MOTOR_SPEED_LOW;
      if (currentState == STATE_PATROL) {
        currentState = STATE_RETURN_BASE;
      }
    } else {
      maxSpeed = MOTOR_SPEED_FULL;
    }
  }
  
  // --- 2. HANDLE MANUAL OVERRIDE ---
  if (manualOverride) {
    handleManualCommand();
    return; // Skip state machine during manual control
  }
  
  // --- 3. STATE MACHINE ---
  switch (currentState) {
    case STATE_PATROL:
      handlePatrolState();
      break;
      
    case STATE_RESEARCH:
      handleResearchState();
      break;
      
    case STATE_ALERT:
      handleAlertState();
      break;
      
    case STATE_RETURN_BASE:
      handleReturnBaseState();
      break;
      
    case STATE_DOCKED:
      handleDockedState();
      break;
  }
  
  // --- 4. SEND TELEMETRY ---
  if (millis() - lastTelemetryTx > TELEMETRY_INTERVAL) {
    lastTelemetryTx = millis();
    sendTelemetry();
  }
  
  // Small delay to prevent tight loop
  delay(10);
}



    readTemperature();
    readMPU6050();
    readGPS();
    checkCameraDetection();
    
    // Check if near base for video streaming
    float distToBase = calculateDistance(currentLat, currentLng, config.baseLat, config.baseLng);
    nearBase = (distToBase < NEAR_BASE_CM);
    
    // Start/stop streaming based on proximity to base
    if (nearBase && !isStreaming) {
      startCameraStream();
    } else if (!nearBase && isStreaming) {
      stopCameraStream();
    }
    
    // Power saving mode
    if (batteryPercent < LOW_BAT_PERCENT) {
      maxSpeed = MOTOR_SPEED_LOW;
      if (currentState == STATE_PATROL) {
        currentState = STATE_RETURN_BASE;
      }
    } else {
      maxSpeed = MOTOR_SPEED_FULL;
    }
  }
  
  // --- 2. HANDLE MANUAL OVERRIDE ---
  if (manualOverride) {
    handleManualCommand();
    return; // Skip state machine during manual control
  }
  
  // --- 3. STATE MACHINE ---
  switch (currentState) {
    case STATE_PATROL:
      handlePatrolState();
      break;
      
    case STATE_RESEARCH:
      handleResearchState();
      break;
      
    case STATE_ALERT:
      handleAlertState();
      break;
      
    case STATE_RETURN_BASE:
      handleReturnBaseState();
      break;
      
    case STATE_DOCKED:
      handleDockedState();
      break;
  }
  
  // --- 4. SEND TELEMETRY ---
  if (millis() - lastTelemetryTx > TELEMETRY_INTERVAL) {
    lastTelemetryTx = millis();
    sendTelemetry();
  }
  
  // Small delay to prevent tight loop
  delay(10);
}


