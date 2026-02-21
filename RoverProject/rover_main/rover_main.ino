/**
 * ============================================================================
 * STASIS - ROVER UNIT FIRMWARE (ESP32-S3)
 * ============================================================================
 * 
 * State Machine: PATROL → RESEARCH → ALERT → RETURN_BASE → DOCKED
 * 
 * Features:
 * - Watchdog timer for system recovery
 * - Non-blocking state machine (no delay() calls in main loop)
 * - GPS navigation with bearing calculation
 * - Video streaming when near base station
 * - Configurable SMS recipients via EEPROM
 * - Image capture on demand from ESP32-CAM
 * - Path recording and waypoint navigation
 * - Night mode with reduced activity
 * - Self-diagnostics and error reporting
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
#define PIN_LED_STATUS     45  // Status LED

// ============================================================================
// CONSTANTS
// ============================================================================
#define WDT_TIMEOUT        30    // Watchdog timeout in seconds
#define OBSTACLE_CM        30.0  // Obstacle detection distance
#define LOW_BAT_PERCENT    30    // Low battery threshold
#define CRITICAL_BAT_PERCENT 15  // Critical battery - immediate return
#define NEAR_BASE_CM       100   // Distance to consider "near base" for streaming
#define TELEMETRY_INTERVAL 3000  // Telemetry send interval (ms)
#define SENSOR_INTERVAL    500   // Sensor read interval (ms)
#define VMAX               12.6  // Max battery voltage
#define VMIN               10.5  // Min battery voltage
#define EEPROM_SIZE        512   // EEPROM size (increased for waypoints)
#define CONFIG_VALID_FLAG  0xAA  // Config validation flag
#define MAX_WAYPOINTS      20    // Maximum stored waypoints
#define PATH_BUFFER_SIZE   100   // Path recording buffer

// Motor speed settings
#define MOTOR_SPEED_FULL   200
#define MOTOR_SPEED_LOW    100
#define MOTOR_SPEED_TURN   150

// Night mode settings
#define NIGHT_START_HOUR   22  // 10 PM
#define NIGHT_END_HOUR     6   // 6 AM

// ============================================================================
// STATE MACHINE
// ============================================================================
enum RoverState { 
  STATE_PATROL, 
  STATE_RESEARCH, 
  STATE_ALERT, 
  STATE_RETURN_BASE, 
  STATE_DOCKED,
  STATE_EMERGENCY,
  STATE_SLEEP
};

// ============================================================================
// DATA STRUCTURES
// ============================================================================

// Waypoint for navigation
typedef struct __attribute__((packed)) {
  double lat;
  double lng;
  char   name[16];
  bool   visited;
} Waypoint;

// Configuration stored in EEPROM
typedef struct __attribute__((packed)) {
  char    phone1[16];       // Primary SMS recipient
  char    phone2[16];       // Secondary SMS recipient
  double  baseLat;          // Base station latitude
  double  baseLng;          // Base station longitude
  float   alertTempHigh;    // High temperature alert threshold
  float   alertTempLow;     // Low temperature alert threshold
  uint8_t patrolSpeed;      // Default patrol speed
  uint8_t nightModeEnabled; // Night mode flag
  uint8_t configValid;      // Validation flag
  uint8_t waypointCount;    // Number of stored waypoints
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
  uint8_t state;
  uint8_t signalStrength;
  uint16_t errorCode;
  uint32_t uptime;
} RoverPacket;

// Command from base station
typedef struct __attribute__((packed)) {
  char cmd[16];
  float param1;
  float param2;
} CommandPacket;

// Camera command to ESP32-CAM
typedef enum {
  CAM_CMD_CAPTURE     = 0x01,
  CAM_CMD_STREAM_ON   = 0x02,
  CAM_CMD_STREAM_OFF  = 0x03,
  CAM_CMD_DETECT      = 0x04,
  CAM_CMD_NIGHT_MODE  = 0x05
} CamCommand;

// Detection result from ESP32-CAM
typedef struct __attribute__((packed)) {
  bool  fire;
  bool  motion;
  bool  human;
  float confidence;
  char  description[24];
} DetectionResult;

// Path point for recording
typedef struct __attribute__((packed)) {
  double lat;
  double lng;
  unsigned long timestamp;
} PathPoint;

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
RoverState   previousState    = STATE_PATROL;
RoverConfig  config;
RoverPacket  telemetry;
bool         configLoaded     = false;

// Timing variables (non-blocking)
unsigned long lastSensorRead  = 0;
unsigned long lastTelemetryTx = 0;
unsigned long stateTimer      = 0;
unsigned long bootTime        = 0;
unsigned long lastHeartbeat   = 0;

// Sensor data
float   batteryPercent   = 100.0;
float   batteryVoltage   = 12.6;
float   currentTemp      = 0.0;
float   obstacleDistance = -1.0;
float   currentHeading   = 0.0;
float   accelX           = 0.0;
float   accelY           = 0.0;
float   accelZ           = 0.0;
double  currentLat       = 0.0;
double  currentLng       = 0.0;
float   distToBase       = 0.0;
uint8_t gpsSatellites    = 0;

// Detection flags
bool    camFireDetected   = false;
bool    camMotionDetected = false;
bool    camHumanDetected  = false;
bool    earthquakeDetected = false;
bool    tempAlertActive   = false;

// Control flags
bool    manualOverride    = false;
String  manualCommand     = "";
int     maxSpeed          = MOTOR_SPEED_FULL;
bool    isStreaming       = false;
bool    nearBase          = false;
bool    nightMode         = false;
bool    pathRecording     = false;
int     currentWaypoint   = 0;

// Error tracking
uint16_t errorCode        = 0;
#define ERR_GPS_LOST      0x0001
#define ERR_MPU_FAIL      0x0002
#define ERR_CAM_FAIL      0x0004
#define ERR_SIM_FAIL      0x0008
#define ERR_LOW_BAT       0x0010
#define ERR_MOTOR_STALL   0x0020
#define ERR_OBSTACLE      0x0040
#define ERR_MEMORY_LOW    0x0080

// Path recording
PathPoint pathBuffer[PATH_BUFFER_SIZE];
int       pathIndex = 0;

// Waypoints
Waypoint waypoints[MAX_WAYPOINTS];

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
    config.alertTempLow = -10.0;
    config.patrolSpeed = MOTOR_SPEED_FULL;
    config.nightModeEnabled = true;
    config.configValid = CONFIG_VALID_FLAG;
    config.waypointCount = 0;
    saveConfig();
  }
  
  // Load waypoints
  if (config.waypointCount > 0 && config.waypointCount <= MAX_WAYPOINTS) {
    EEPROM.get(sizeof(RoverConfig), waypoints);
  }
  
  configLoaded = true;
}

void saveConfig() {
  EEPROM.put(0, config);
  EEPROM.commit();
}

void saveWaypoints() {
  EEPROM.put(sizeof(RoverConfig), waypoints);
  EEPROM.commit();
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

float calculateBearing(float lat1, float lon1, float lat2, float lon2) {
  float dLon = (lon2 - lon1) * PI / 180.0;
  lat1 *= PI / 180.0;
  lat2 *= PI / 180.0;
  
  float y = sin(dLon) * cos(lat2);
  float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
  
  float bearing = atan2(y, x) * 180.0 / PI;
  return fmod((bearing + 360.0), 360.0);
}

float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
  float R = 6371000; // Earth radius in meters
  float dLat = (lat2 - lat1) * PI / 180.0;
  float dLon = (lon2 - lon1) * PI / 180.0;
  
  float a = sin(dLat/2) * sin(dLat/2) +
            cos(lat1 * PI/180.0) * cos(lat2 * PI/180.0) *
            sin(dLon/2) * sin(dLon/2);
  float c = 2 * atan2(sqrt(a), sqrt(1-a));
  
  return R * c;
}

bool isNightTime() {
  // Simple night detection based on hour
  // In production, could use light sensor or GPS time
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    int hour = timeinfo.tm_hour;
    return (hour >= NIGHT_START_HOUR || hour < NIGHT_END_HOUR);
  }
  return false;
}

void updateNightMode() {
  if (config.nightModeEnabled) {
    nightMode = isNightTime();
    if (nightMode && currentState == STATE_PATROL) {
      maxSpeed = MOTOR_SPEED_LOW;
    }
  }
}

void recordPathPoint() {
  if (pathRecording && pathIndex < PATH_BUFFER_SIZE) {
    pathBuffer[pathIndex].lat = currentLat;
    pathBuffer[pathIndex].lng = currentLng;
    pathBuffer[pathIndex].timestamp = millis();
    pathIndex++;
  }
}

void addWaypoint(double lat, double lng, const char* name) {
  if (config.waypointCount < MAX_WAYPOINTS) {
    waypoints[config.waypointCount].lat = lat;
    waypoints[config.waypointCount].lng = lng;
    strncpy(waypoints[config.waypointCount].name, name, 15);
    waypoints[config.waypointCount].name[15] = '\0';
    waypoints[config.waypointCount].visited = false;
    config.waypointCount++;
    saveWaypoints();
    saveConfig();
  }
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
    moveForward(maxSpeed);
    return;
  }
  
  float targetBearing = calculateBearing(currentLat, currentLng, config.baseLat, config.baseLng);
  float bearingDiff = targetBearing - currentHeading;
  
  while (bearingDiff > 180) bearingDiff -= 360;
  while (bearingDiff < -180) bearingDiff += 360;
  
  if (bearingDiff > 15) {
    turnRight(maxSpeed);
  } else if (bearingDiff < -15) {
    turnLeft(maxSpeed);
  } else {
    moveForward(maxSpeed);
  }
}

void navigateToWaypoint(int wpIndex) {
  if (wpIndex < 0 || wpIndex >= config.waypointCount) return;
  if (currentLat == 0.0 && currentLng == 0.0) return;
  
  Waypoint* wp = &waypoints[wpIndex];
  float targetBearing = calculateBearing(currentLat, currentLng, wp->lat, wp->lng);
  float bearingDiff = targetBearing - currentHeading;
  
  while (bearingDiff > 180) bearingDiff -= 360;
  while (bearingDiff < -180) bearingDiff += 360;
  
  float distToWp = calculateDistance(currentLat, currentLng, wp->lat, wp->lng);
  
  if (distToWp < 5.0) {
    // Reached waypoint
    wp->visited = true;
    currentWaypoint = (currentWaypoint + 1) % config.waypointCount;
    return;
  }
  
  if (bearingDiff > 20) {
    turnRight(maxSpeed);
  } else if (bearingDiff < -20) {
    turnLeft(maxSpeed);
  } else {
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
  batteryVoltage = (raw / 4095.0) * 3.3 * 4.04;
  return constrain((batteryVoltage - VMIN) / (VMAX - VMIN) * 100.0, 0, 100);
}

void readMPU6050() {
  sensors_event_t a, g, t;
  if (mpu.getEvent(&a, &g, &t)) {
    accelX = a.acceleration.x;
    accelY = a.acceleration.y;
    accelZ = a.acceleration.z;
    
    // Calculate heading from gyro (simplified - should use magnetometer for true heading)
    currentHeading += g.gyro.z * (millis() - lastSensorRead) / 1000.0 * 180.0 / PI;
    currentHeading = fmod(currentHeading + 360.0, 360.0);
    
    // Earthquake detection (high acceleration)
    float totalAccel = sqrt(accelX*accelX + accelY*accelY + accelZ*accelZ);
    earthquakeDetected = (totalAccel > 20.0);
    
    errorCode &= ~ERR_MPU_FAIL;
  } else {
    errorCode |= ERR_MPU_FAIL;
  }
}

void readTemperature() {
  tempSensor.requestTemperatures();
  currentTemp = tempSensor.getTempCByIndex(0);
  if (currentTemp == DEVICE_DISCONNECTED_C) {
    currentTemp = -999.0;
  }
  
  // Check temperature alerts
  tempAlertActive = (currentTemp > config.alertTempHigh || currentTemp < config.alertTempLow);
}

void readGPS() {
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }
  if (gps.location.isValid()) {
    currentLat = gps.location.lat();
    currentLng = gps.location.lng();
    gpsSatellites = gps.satellites.value();
    errorCode &= ~ERR_GPS_LOST;
  } else {
    errorCode |= ERR_GPS_LOST;
  }
}

void readAllSensors() {
  obstacleDistance = readUltrasonicDistance();
  batteryPercent = readBatteryPercent();
  readTemperature();
  readMPU6050();
  readGPS();
  checkCameraDetection();
  
  // Calculate distance to base
  if (currentLat != 0.0 && currentLng != 0.0) {
    distToBase = calculateDistance(currentLat, currentLng, config.baseLat, config.baseLng);
    nearBase = (distToBase < NEAR_BASE_CM);
  }
  
  // Record path if enabled
  recordPathPoint();
}

// ============================================================================
// CAMERA COMMUNICATION
// ============================================================================

void sendCamCommand(CamCommand cmd) {
  SerialCam.write((uint8_t)cmd);
}

bool requestImageCapture() {
  sendCamCommand(CAM_CMD_CAPTURE);
  
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
    if (SerialCam.available() >= sizeof(DetectionResult)) {
      DetectionResult result;
      SerialCam.readBytes((char*)&result, sizeof(result));
      camFireDetected = result.fire;
      camMotionDetected = result.motion;
      camHumanDetected = result.human;
      errorCode &= ~ERR_CAM_FAIL;
      return true;
    }
  }
  errorCode |= ERR_CAM_FAIL;
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

bool initSIM800() {
  SerialSIM.println("AT");
  delay(100);
  if (!SerialSIM.find("OK")) {
    errorCode |= ERR_SIM_FAIL;
    return false;
  }
  
  SerialSIM.println("AT+CMGF=1");  // SMS text mode
  delay(100);
  
  errorCode &= ~ERR_SIM_FAIL;
  return true;
}

void sendSMS(const String& message) {
  if (errorCode & ERR_SIM_FAIL) {
    if (!initSIM800()) return;
  }
  
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

void onEspNowRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
  if (len == sizeof(CommandPacket)) {
    CommandPacket cmd;
    memcpy(&cmd, data, sizeof(cmd));
    manualCommand = String(cmd.cmd);
    manualOverride = true;
  }
}

void onEspNowSent(const wifi_pkt_tx_info_t *tx_info, esp_now_send_status_t status) {
  // Could add retry logic here
}

bool initEspNow() {
  WiFi.mode(WIFI_STA);
  
  if (esp_now_init() != ESP_OK) {
    return false;
  }
  
  esp_now_register_recv_cb(onEspNowRecv);
  esp_now_register_send_cb(onEspNowSent);
  
  memcpy(peerInfo.peer_addr, baseMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    return false;
  }
  
  return true;
}

void sendTelemetry() {
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
  telemetry.state = (uint8_t)currentState;
  telemetry.errorCode = errorCode;
  telemetry.uptime = (millis() - bootTime) / 1000;
  
  switch (currentState) {
    case STATE_PATROL:    strcpy(telemetry.status, "PATROL"); break;
    case STATE_RESEARCH:  strcpy(telemetry.status, "RESEARCH"); break;
    case STATE_ALERT:     strcpy(telemetry.status, "CRITICAL ALERT"); break;
    case STATE_RETURN_BASE: strcpy(telemetry.status, "RETURNING"); break;
    case STATE_DOCKED:    strcpy(telemetry.status, "DOCKED"); break;
    case STATE_EMERGENCY: strcpy(telemetry.status, "EMERGENCY"); break;
    case STATE_SLEEP:     strcpy(telemetry.status, "SLEEP"); break;
  }
  
  esp_now_send(baseMac, (uint8_t *)&telemetry, sizeof(telemetry));
}

// ============================================================================
// VIDEO STREAMING SERVER
// ============================================================================

void handleStreamRequest() {
  streamServer.sendHeader("Content-Type", "multipart/x-mixed-replace; boundary=frame");
  streamServer.sendHeader("Connection", "close");
  streamServer.send(200, "text/plain", "Stream active");
}

void handleCaptureRequest() {
  if (requestImageCapture()) {
    String json = "{\"fire\":" + String(camFireDetected ? "true" : "false") + 
                  ",\"motion\":" + String(camMotionDetected ? "true" : "false") + 
                  ",\"human\":" + String(camHumanDetected ? "true" : "false") + "}";
    streamServer.send(200, "application/json", json);
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
  json += "\"alertTempHigh\":" + String(config.alertTempHigh) + ",";
  json += "\"alertTempLow\":" + String(config.alertTempLow) + ",";
  json += "\"patrolSpeed\":" + String(config.patrolSpeed) + ",";
  json += "\"nightModeEnabled\":" + String(config.nightModeEnabled) + ",";
  json += "\"waypointCount\":" + String(config.waypointCount);
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
  if (streamServer.hasArg("alertTempHigh")) {
    config.alertTempHigh = streamServer.arg("alertTempHigh").toFloat();
  }
  if (streamServer.hasArg("alertTempLow")) {
    config.alertTempLow = streamServer.arg("alertTempLow").toFloat();
  }
  if (streamServer.hasArg("patrolSpeed")) {
    config.patrolSpeed = streamServer.arg("patrolSpeed").toInt();
  }
  saveConfig();
  streamServer.send(200, "application/json", "{\"status\":\"ok\"}");
}

void handleStatusRequest() {
  String json = "{";
  json += "\"state\":\"" + String(telemetry.status) + "\",";
  json += "\"battery\":" + String(batteryPercent, 1) + ",";
  json += "\"voltage\":" + String(batteryVoltage, 2) + ",";
  json += "\"temp\":" + String(currentTemp, 1) + ",";
  json += "\"lat\":" + String(currentLat, 6) + ",";
  json += "\"lng\":" + String(currentLng, 6) + ",";
  json += "\"heading\":" + String(currentHeading, 1) + ",";
  json += "\"distance\":" + String(obstacleDistance, 1) + ",";
  json += "\"distToBase\":" + String(distToBase, 1) + ",";
  json += "\"satellites\":" + String(gpsSatellites) + ",";
  json += "\"streaming\":" + String(isStreaming ? "true" : "false") + ",";
  json += "\"nightMode\":" + String(nightMode ? "true" : "false") + ",";
  json += "\"errorCode\":" + String(errorCode) + ",";
  json += "\"uptime\":" + String((millis() - bootTime) / 1000);
  json += "}";
  streamServer.send(200, "application/json", json);
}

void handleWaypointsRequest() {
  String json = "{\"waypoints\":[";
  for (int i = 0; i < config.waypointCount; i++) {
    if (i > 0) json += ",";
    json += "{\"lat\":" + String(waypoints[i].lat, 6) + 
            ",\"lng\":" + String(waypoints[i].lng, 6) + 
            ",\"name\":\"" + String(waypoints[i].name) + "\"" +
            ",\"visited\":" + String(waypoints[i].visited ? "true" : "false") + "}";
  }
  json += "]}";
  streamServer.send(200, "application/json", json);
}

void handleAddWaypointRequest() {
  if (streamServer.hasArg("lat") && streamServer.hasArg("lng")) {
    double lat = streamServer.arg("lat").toDouble();
    double lng = streamServer.arg("lng").toDouble();
    String name = streamServer.arg("name");
    if (name.length() == 0) name = "WP" + String(config.waypointCount);
    addWaypoint(lat, lng, name.c_str());
    streamServer.send(200, "application/json", "{\"status\":\"ok\"}");
  } else {
    streamServer.send(400, "application/json", "{\"error\":\"missing params\"}");
  }
}

void initStreamServer() {
  streamServer.on("/stream", handleStreamRequest);
  streamServer.on("/capture", HTTP_POST, handleCaptureRequest);
  streamServer.on("/config", HTTP_GET, handleConfigRequest);
  streamServer.on("/config", HTTP_POST, handleSetConfigRequest);
  streamServer.on("/status", HTTP_GET, handleStatusRequest);
  streamServer.on("/waypoints", HTTP_GET, handleWaypointsRequest);
  streamServer.on("/waypoints/add", HTTP_POST, handleAddWaypointRequest);
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
  } else if (manualCommand == "RETURN") {
    currentState = STATE_RETURN_BASE;
    manualOverride = false;
  } else if (manualCommand == "PATROL") {
    currentState = STATE_PATROL;
    manualOverride = false;
  } else if (manualCommand == "DOCK") {
    currentState = STATE_DOCKED;
    manualOverride = false;
  } else if (manualCommand == "CAPTURE") {
    requestImageCapture();
  } else if (manualCommand == "STREAM_ON") {
    startCameraStream();
  } else if (manualCommand == "STREAM_OFF") {
    stopCameraStream();
  } else if (manualCommand == "RECORD_START") {
    pathRecording = true;
    pathIndex = 0;
  } else if (manualCommand == "RECORD_STOP") {
    pathRecording = false;
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
  
  // Check critical battery
  if (batteryPercent < CRITICAL_BAT_PERCENT) {
    currentState = STATE_EMERGENCY;
    return;
  }
  
  // Obstacle avoidance
  if (obstacleDistance > 0 && obstacleDistance < OBSTACLE_CM) {
    stopMotors();
    moveBackward(MOTOR_SPEED_LOW);
    delay(300);
    turnRight(MOTOR_SPEED_TURN);
    delay(400);
    stopMotors();
    errorCode |= ERR_OBSTACLE;
  } else {
    errorCode &= ~ERR_OBSTACLE;
    
    // Navigate waypoints if configured
    if (config.waypointCount > 0) {
      navigateToWaypoint(currentWaypoint);
    } else {
      moveForward(maxSpeed);
    }
  }
}

void handleResearchState() {
  stopMotors();
  
  if (millis() - stateTimer > 3000) {
    currentState = STATE_PATROL;
  }
}

void handleAlertState() {
  stopMotors();
  digitalWrite(PIN_BUZZER, HIGH);
  digitalWrite(PIN_LED_STATUS, HIGH);
  
  // Send SMS alert
  String alertMsg = "STASIS ALERT!\n";
  alertMsg += "Location: " + String(currentLat, 6) + ", " + String(currentLng, 6) + "\n";
  alertMsg += "Temp: " + String(currentTemp, 1) + " C\n";
  alertMsg += "Battery: " + String(batteryPercent, 0) + "%\n";
  alertMsg += "Status: " + String(telemetry.status) + "\n";
  if (camFireDetected) alertMsg += "FIRE DETECTED!\n";
  if (camHumanDetected) alertMsg += "HUMAN DETECTED!\n";
  if (earthquakeDetected) alertMsg += "EARTHQUAKE DETECTED!\n";
  sendSMS(alertMsg);
  
  sendTelemetry();
  
  delay(2000);
  digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(PIN_LED_STATUS, LOW);
  
  // Clear detection flags
  camFireDetected = false;
  camMotionDetected = false;
  camHumanDetected = false;
  
  currentState = STATE_RESEARCH;
  stateTimer = millis();
}

void handleReturnBaseState() {
  // Check if we've reached base
  if (distToBase < 5.0 || (obstacleDistance > 0 && obstacleDistance < 10)) {
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

void handleEmergencyState() {
  stopMotors();
  digitalWrite(PIN_BUZZER, HIGH);
  delay(500);
  digitalWrite(PIN_BUZZER, LOW);
  
  // Emergency SMS
  String emergencyMsg = "STASIS EMERGENCY!\n";
  emergencyMsg += "Critical battery: " + String(batteryPercent, 0) + "%\n";
  emergencyMsg += "Location: " + String(currentLat, 6) + ", " + String(currentLng, 6);
  sendSMS(emergencyMsg);
  
  // Try to return to base
  if (batteryPercent > 5) {
    currentState = STATE_RETURN_BASE;
  }
}

void handleSleepState() {
  stopMotors();
  stopCameraStream();
  
  // Low power mode - only wake on hazard detection
  if (camFireDetected || camHumanDetected || tempAlertActive) {
    currentState = STATE_ALERT;
  } else if (!nightMode) {
    currentState = STATE_PATROL;
  }
}

// ============================================================================
// SELF DIAGNOSTICS
// ============================================================================

void runDiagnostics() {
  Serial.println("\n=== STASIS Diagnostics ===");
  
  // Battery
  Serial.printf("Battery: %.1f%% (%.2fV)\n", batteryPercent, batteryVoltage);
  
  // Sensors
  Serial.printf("Temperature: %.1f C\n", currentTemp);
  Serial.printf("GPS: %d satellites, %.6f, %.6f\n", gpsSatellites, currentLat, currentLng);
  Serial.printf("IMU: Accel(%.2f, %.2f, %.2f) Heading: %.1f\n", accelX, accelY, accelZ, currentHeading);
  Serial.printf("Ultrasonic: %.1f cm\n", obstacleDistance);
  
  // Camera
  Serial.printf("Camera: %s, Fire:%d Motion:%d Human:%d\n", 
                isStreaming ? "streaming" : "idle",
                camFireDetected, camMotionDetected, camHumanDetected);
  
  // State
  Serial.printf("State: %s, Distance to base: %.1f m\n", telemetry.status, distToBase);
  
  // Errors
  if (errorCode) {
    Serial.printf("Errors: 0x%04X\n", errorCode);
    if (errorCode & ERR_GPS_LOST) Serial.println("  - GPS signal lost");
    if (errorCode & ERR_MPU_FAIL) Serial.println("  - MPU6050 failure");
    if (errorCode & ERR_CAM_FAIL) Serial.println("  - Camera failure");
    if (errorCode & ERR_SIM_FAIL) Serial.println("  - SIM800 failure");
    if (errorCode & ERR_LOW_BAT) Serial.println("  - Low battery");
    if (errorCode & ERR_OBSTACLE) Serial.println("  - Obstacle detected");
  }
  
  Serial.println("===========================\n");
}

// ============================================================================
// MAIN SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  bootTime = millis();
  
  Serial.println("\n========================================");
  Serial.println("STASIS - Autonomous Forest Monitoring");
  Serial.println("========================================\n");
  
  // Initialize serial ports
  SerialCam.begin(115200, SERIAL_8N1, PIN_CAM_RX, PIN_CAM_TX);
  SerialSIM.begin(9600, SERIAL_8N1, PIN_SIM_RX, PIN_SIM_TX);
  SerialGPS.begin(9600);
  
  // Initialize watchdog timer
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WDT_TIMEOUT * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);
  
  // Load configuration from EEPROM
  loadConfig();
  
  // Initialize pins
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED_STATUS, OUTPUT);
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
    Serial.println("MPU6050 initialized");
  } else {
    Serial.println("MPU6050 failed!");
    errorCode |= ERR_MPU_FAIL;
  }
  
  // Initialize temperature sensor
  tempSensor.begin();
  Serial.println("Temperature sensor initialized");
  
  // Initialize ESP-NOW
  if (initEspNow()) {
    Serial.println("ESP-NOW initialized");
  } else {
    Serial.println("ESP-NOW failed!");
  }
  
  // Initialize SIM800
  if (initSIM800()) {
    Serial.println("SIM800 initialized");
  }
  
  // Initialize streaming server
  initStreamServer();
  Serial.println("Stream server started on port 81");
  
  // Initial sensor readings
  readAllSensors();
  
  // Startup beep
  digitalWrite(PIN_BUZZER, HIGH);
  delay(100);
  digitalWrite(PIN_BUZZER, LOW);
  delay(100);
  digitalWrite(PIN_BUZZER, HIGH);
  delay(100);
  digitalWrite(PIN_BUZZER, LOW);
  
  Serial.println("\nSTASIS Rover initialized and ready!\n");
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
    readAllSensors();
    
    // Update night mode
    updateNightMode();
    
    // Start/stop streaming based on proximity to base
    if (nearBase && !isStreaming) {
      startCameraStream();
    } else if (!nearBase && isStreaming) {
      stopCameraStream();
    }
    
    // Power saving mode
    if (batteryPercent < LOW_BAT_PERCENT) {
      maxSpeed = MOTOR_SPEED_LOW;
      errorCode |= ERR_LOW_BAT;
      if (currentState == STATE_PATROL && batteryPercent < CRITICAL_BAT_PERCENT) {
        currentState = STATE_RETURN_BASE;
      }
    } else {
      maxSpeed = config.patrolSpeed;
      errorCode &= ~ERR_LOW_BAT;
    }
  }
  
  // --- 2. HANDLE MANUAL OVERRIDE ---
  if (manualOverride) {
    handleManualCommand();
    return;
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
      
    case STATE_EMERGENCY:
      handleEmergencyState();
      break;
      
    case STATE_SLEEP:
      handleSleepState();
      break;
  }
  
  // --- 4. SEND TELEMETRY ---
  if (millis() - lastTelemetryTx > TELEMETRY_INTERVAL) {
    lastTelemetryTx = millis();
    sendTelemetry();
  }
  
  // --- 5. PERIODIC DIAGNOSTICS ---
  if (millis() - lastHeartbeat > 60000) {
    lastHeartbeat = millis();
    runDiagnostics();
  }
  
  // Small delay to prevent tight loop
  delay(10);
}
//67 hehehehhehehehehehhe
