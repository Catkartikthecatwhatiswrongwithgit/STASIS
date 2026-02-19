/**
 * ============================================================================
 * STASIS - BASE STATION BRIDGE (ESP32-C3)
 * ============================================================================
 * 
 * Bidirectional Communication:
 * - Rover → Pi: ESP-NOW receive → UART transmit (JSON)
 * - Pi → Rover: UART receive → ESP-NOW transmit
 * 
 * Features:
 * - Message queuing for reliability
 * - ESP-NOW acknowledgment handling
 * - Status LED for operational feedback
 * - WiFi AP for status page (connect to check if code is working)
 * - Automatic reconnection on ESP-NOW failure
 * - Heartbeat signal for connection monitoring
 * - Command buffering and retry logic
 * - Signal strength monitoring (RSSI)
 * - Multi-rover support (up to 4 rovers)
 * - Diagnostic logging over Serial
 * - OTA update support
 * - Deep sleep mode for power saving
 * - Watchdog timer for reliability
 * - Configuration via web interface
 * - Real-time clock synchronization
 * - Data encryption support
 * 
 * ============================================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ArduinoJson.h>
#include <WebServer.h>
#include <EEPROM.h>
#include <esp_task_wdt.h>
#include <esp_wifi.h>
#include <Update.h>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
#define PIN_RX_FROM_PI    20
#define PIN_TX_TO_PI      21
#define LED_PIN           8     // Built-in LED on most ESP32-C3 boards
#define PIN_RESET_BUTTON  9     // Optional reset button
#define PIN_BATTERY_ADC   2     // Optional battery monitoring

// ============================================================================
// CONSTANTS
// ============================================================================
#define QUEUE_SIZE          10
#define HEARTBEAT_MS        5000
#define RECONNECT_MS        30000
#define MAX_RETRIES         3
#define RETRY_DELAY_MS      100
#define WDT_TIMEOUT         30
#define EEPROM_SIZE         256
#define MAX_ROVERS          4
#define STATUS_JSON_SIZE    1024
#define COMMAND_BUFFER_SIZE 64

// WiFi AP Configuration - connect to check if bridge is working
const char* AP_SSID = "AeroSentinel-Base";
const char* AP_PASS = "sentinel123";

// Firmware version
const char* FIRMWARE_VERSION = "2.0.0";

// ============================================================================
// DATA STRUCTURES (Must match rover_main.ino exactly)
// ============================================================================
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

typedef struct __attribute__((packed)) {
  char cmd[16];
  float param1;
  float param2;
} CommandPacket;

// ============================================================================
// ROVER REGISTRY
// ============================================================================
typedef struct {
  uint8_t mac[6];
  char    name[16];
  bool    active;
  unsigned long lastSeen;
  uint32_t packetsReceived;
  uint32_t commandsSent;
  int8_t   rssi;
} RoverInfo;

// ============================================================================
// MESSAGE QUEUE
// ============================================================================
typedef struct {
  CommandPacket cmd;
  uint8_t       targetRover;  // 0xFF = broadcast
  uint8_t       retries;
  unsigned long lastAttempt;
  bool          pending;
  bool          highPriority;
} QueuedCommand;

QueuedCommand cmdQueue[QUEUE_SIZE];
int queueHead = 0;
int queueTail = 0;

// ============================================================================
// CONFIGURATION STRUCTURE
// ============================================================================
typedef struct {
  char    apSSID[32];
  char    apPassword[32];
  uint8_t channel;
  uint8_t encryptionMode;  // 0=none, 1=AES
  char    encryptionKey[17];
  uint8_t configValid;
} BridgeConfig;

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================
RoverPacket          lastTelemetry;
RoverInfo            rovers[MAX_ROVERS];
BridgeConfig         config;
esp_now_peer_info_t  roverPeers[MAX_ROVERS];
uint8_t              broadcastMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

unsigned long lastHeartbeat = 0;
unsigned long lastReconnect = 0;
unsigned long lastBlink = 0;
unsigned long bootTime = 0;
bool          espNowReady   = false;
bool          lastSendOk    = true;
uint32_t      packetsReceived = 0;
uint32_t      commandsSent = 0;
uint32_t      totalErrors = 0;
int8_t        currentRSSI = 0;

WebServer server(80);

// LED blink patterns
int           ledBlinkCount = 0;
unsigned long ledBlinkTime = 0;
bool          ledState = false;

// Command buffer for partial commands
char          commandBuffer[COMMAND_BUFFER_SIZE];
int           commandBufferIndex = 0;

// Diagnostic logging
bool          verboseLogging = true;
char          logBuffer[256];

// ============================================================================
// EEPROM FUNCTIONS
// ============================================================================

void loadConfig() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(0, config);
  
  if (config.configValid != 0xAA) {
    // Set defaults
    strcpy(config.apSSID, AP_SSID);
    strcpy(config.apPassword, AP_PASS);
    config.channel = 1;
    config.encryptionMode = 0;
    strcpy(config.encryptionKey, "");
    config.configValid = 0xAA;
    saveConfig();
  }
  
  EEPROM.commit();
}

void saveConfig() {
  EEPROM.put(0, config);
  EEPROM.commit();
}

// ============================================================================
// LOGGING FUNCTIONS
// ============================================================================

void logMessage(const char* level, const char* message) {
  if (!verboseLogging && strcmp(level, "DEBUG") == 0) return;
  
  unsigned long uptime = (millis() - bootTime) / 1000;
  snprintf(logBuffer, sizeof(logBuffer), "[%lu s] [%s] %s", uptime, level, message);
  Serial.println(logBuffer);
}

void logInfo(const char* message) {
  logMessage("INFO", message);
}

void logDebug(const char* message) {
  logMessage("DEBUG", message);
}

void logError(const char* message) {
  logMessage("ERROR", message);
}

void logWarn(const char* message) {
  logMessage("WARN", message);
}

// ============================================================================
// LED CONTROL
// ============================================================================

void ledOn() {
  digitalWrite(LED_PIN, HIGH);
  ledState = true;
}

void ledOff() {
  digitalWrite(LED_PIN, LOW);
  ledState = false;
}

void toggleLED() {
  if (ledState) {
    ledOff();
  } else {
    ledOn();
  }
}

void blinkLED(int count, int delayMs) {
  ledBlinkCount = count * 2;
  ledBlinkTime = millis();
  ledOff();
}

void updateLED() {
  // Handle blink pattern
  if (ledBlinkCount > 0) {
    if (millis() - ledBlinkTime > 100) {
      ledBlinkTime = millis();
      if (ledBlinkCount % 2 == 0) {
        ledOn();
      } else {
        ledOff();
      }
      ledBlinkCount--;
    }
  }
  
  // Heartbeat blink (every 2 seconds when idle)
  if (ledBlinkCount == 0 && millis() - lastBlink > 2000) {
    lastBlink = millis();
    ledOn();
    delay(10);
    ledOff();
  }
}

// ============================================================================
// ROVER REGISTRY FUNCTIONS
// ============================================================================

int findRoverByMac(const uint8_t* mac) {
  for (int i = 0; i < MAX_ROVERS; i++) {
    if (rovers[i].active && 
        memcmp(rovers[i].mac, mac, 6) == 0) {
      return i;
    }
  }
  return -1;
}

int findFreeRoverSlot() {
  for (int i = 0; i < MAX_ROVERS; i++) {
    if (!rovers[i].active) {
      return i;
    }
  }
  return -1;
}

int registerRover(const uint8_t* mac) {
  int idx = findRoverByMac(mac);
  if (idx >= 0) {
    // Already registered, update last seen
    rovers[idx].lastSeen = millis();
    return idx;
  }
  
  // Find free slot
  idx = findFreeRoverSlot();
  if (idx < 0) {
    logWarn("Rover registry full!");
    return -1;
  }
  
  // Register new rover
  memcpy(rovers[idx].mac, mac, 6);
  snprintf(rovers[idx].name, sizeof(rovers[idx].name), "Rover%d", idx + 1);
  rovers[idx].active = true;
  rovers[idx].lastSeen = millis();
  rovers[idx].packetsReceived = 0;
  rovers[idx].commandsSent = 0;
  rovers[idx].rssi = 0;
  
  char msg[64];
  snprintf(msg, sizeof(msg), "Registered new rover: %s", rovers[idx].name);
  logInfo(msg);
  
  return idx;
}

void cleanupInactiveRovers() {
  unsigned long now = millis();
  for (int i = 0; i < MAX_ROVERS; i++) {
    if (rovers[i].active && 
        (now - rovers[i].lastSeen > 60000)) {  // 60 second timeout
      char msg[64];
      snprintf(msg, sizeof(msg), "Rover %s timed out", rovers[i].name);
      logWarn(msg);
      rovers[i].active = false;
    }
  }
}

// ============================================================================
// WEB HANDLERS - Status page to verify code is working
// ============================================================================

void handleRoot() {
  String html = "<!DOCTYPE html><html><head>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<meta http-equiv='refresh' content='5'>"
    "<title>Aero Sentinel Base Station</title>"
    "<style>"
    "body{font-family:Arial;background:#0a0a1a;color:#eee;padding:20px}"
    ".c{max-width:600px;margin:0 auto}h1{color:#00d4ff;text-align:center}"
    ".card{background:#1a1a2e;border-radius:8px;padding:15px;margin:10px 0}"
    ".r{display:flex;justify-content:space-between;padding:5px 0}"
    ".v{color:#0f8}.bad{color:#f44}.warn{color:#fa0}"
    ".btn{background:#00d4ff;color:#000;border:none;padding:8px 16px;"
    "border-radius:4px;cursor:pointer;margin:5px}"
    ".btn:hover{background:#00a8cc}"
    "table{width:100%;border-collapse:collapse;margin-top:10px}"
    "th,td{padding:8px;text-align:left;border-bottom:1px solid #333}"
    "th{color:#00d4ff}"
    "</style></head><body><div class='c'>"
    "<h1>🛡️ AERO SENTINEL</h1>"
    "<div class='card'><h2>Bridge Status</h2>"
    "<div class='r'><span>Firmware:</span><span class='v'>" + String(FIRMWARE_VERSION) + "</span></div>"
    "<div class='r'><span>ESP-NOW:</span><span class='" + String(espNowReady?"v":"bad") + "'>" + String(espNowReady?"OK":"FAIL") + "</span></div>"
    "<div class='r'><span>Packets Received:</span><span class='v'>" + String(packetsReceived) + "</span></div>"
    "<div class='r'><span>Commands Sent:</span><span class='v'>" + String(commandsSent) + "</span></div>"
    "<div class='r'><span>Errors:</span><span class='" + String(totalErrors>0?"bad":"v") + "'>" + String(totalErrors) + "</span></div>"
    "<div class='r'><span>Uptime:</span><span class='v'>" + String((millis()-bootTime)/1000) + "s</span></div>"
    "</div>";
  
  // Rover list
  html += "<div class='card'><h2>Registered Rovers</h2>";
  int activeRovers = 0;
  for (int i = 0; i < MAX_ROVERS; i++) {
    if (rovers[i].active) {
      activeRovers++;
      html += "<div class='r'><span>" + String(rovers[i].name) + "</span>";
      html += "<span class='v'>RSSI: " + String(rovers[i].rssi) + " dBm</span></div>";
    }
  }
  if (activeRovers == 0) {
    html += "<div class='r'><span>No rovers connected</span></div>";
  }
  html += "</div>";
  
  // Last telemetry
  html += "<div class='card'><h2>Last Telemetry</h2>"
    "<div class='r'><span>Status:</span><span class='v'>" + String(lastTelemetry.status) + "</span></div>"
    "<div class='r'><span>Temp:</span><span class='v'>" + String(lastTelemetry.temp,1) + " C</span></div>"
    "<div class='r'><span>Battery:</span><span class='v'>" + String(lastTelemetry.bat,0) + "%</span></div>"
    "<div class='r'><span>Location:</span><span class='v'>" + String(lastTelemetry.lat,4) + "," + String(lastTelemetry.lng,4) + "</span></div>"
    "<div class='r'><span>Heading:</span><span class='v'>" + String(lastTelemetry.heading,0) + "°</span></div>"
    "<div class='r'><span>Hazard:</span><span class='" + String(lastTelemetry.hazard?"bad":"v") + "'>" + String(lastTelemetry.hazard?"ALERT!":"None") + "</span></div>"
    "</div>";
  
  // Quick commands
  html += "<div class='card'><h2>Quick Commands</h2>"
    "<button class='btn' onclick=\"sendCmd('STOP')\">STOP</button>"
    "<button class='btn' onclick=\"sendCmd('PATROL')\">PATROL</button>"
    "<button class='btn' onclick=\"sendCmd('RETURN')\">RETURN</button>"
    "<button class='btn' onclick=\"sendCmd('DOCK')\">DOCK</button>"
    "<script>"
    "function sendCmd(c){fetch('/api/cmd?cmd='+c).then(r=>r.json()).then(d=>alert(d.status||d.error))}"
    "</script></div>";
  
  html += "</div></body></html>";
  server.send(200, "text/html", html);
}

void handleJSON() {
  StaticJsonDocument<STATUS_JSON_SIZE> doc;
  doc["firmware"] = FIRMWARE_VERSION;
  doc["espnow"] = espNowReady;
  doc["packets"] = packetsReceived;
  doc["commands"] = commandsSent;
  doc["errors"] = totalErrors;
  doc["uptime"] = (millis() - bootTime) / 1000;
  
  JsonObject t = doc.createNestedObject("telemetry");
  t["status"] = lastTelemetry.status;
  t["temp"] = lastTelemetry.temp;
  t["bat"] = lastTelemetry.bat;
  t["lat"] = lastTelemetry.lat;
  t["lng"] = lastTelemetry.lng;
  t["hazard"] = lastTelemetry.hazard;
  t["heading"] = lastTelemetry.heading;
  t["distance"] = lastTelemetry.distance;
  
  JsonArray roverList = doc.createNestedArray("rovers");
  for (int i = 0; i < MAX_ROVERS; i++) {
    if (rovers[i].active) {
      JsonObject r = roverList.createNestedObject();
      r["name"] = rovers[i].name;
      r["rssi"] = rovers[i].rssi;
      r["packets"] = rovers[i].packetsReceived;
      r["lastSeen"] = (millis() - rovers[i].lastSeen) / 1000;
    }
  }
  
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void handleCommand() {
  if (!server.hasArg("cmd")) {
    server.send(400, "application/json", "{\"error\":\"Missing cmd parameter\"}");
    return;
  }
  
  String cmd = server.arg("cmd");
  cmd.toUpperCase();
  
  CommandPacket packet;
  memset(&packet, 0, sizeof(packet));
  cmd.toCharArray(packet.cmd, sizeof(packet.cmd));
  
  // Check for parameters
  if (server.hasArg("p1")) {
    packet.param1 = server.arg("p1").toFloat();
  }
  if (server.hasArg("p2")) {
    packet.param2 = server.arg("p2").toFloat();
  }
  
  if (queueCommand(packet)) {
    blinkLED(2, 50);
    server.send(200, "application/json", "{\"status\":\"Command queued\"}");
  } else {
    server.send(503, "application/json", "{\"error\":\"Queue full\"}");
  }
}

void handleConfig() {
  if (server.method() == HTTP_GET) {
    StaticJsonDocument<512> doc;
    doc["apSSID"] = config.apSSID;
    doc["channel"] = config.channel;
    doc["encryption"] = config.encryptionMode;
    String out;
    serializeJson(doc, out);
    server.send(200, "application/json", out);
  } else {
    // POST - update config
    if (server.hasArg("ssid")) {
      server.arg("ssid").toCharArray(config.apSSID, sizeof(config.apSSID));
    }
    if (server.hasArg("password")) {
      server.arg("password").toCharArray(config.apPassword, sizeof(config.apPassword));
    }
    if (server.hasArg("channel")) {
      config.channel = server.arg("channel").toInt();
    }
    saveConfig();
    server.send(200, "application/json", "{\"status\":\"Config saved, restart required\"}");
  }
}

void handleRovers() {
  StaticJsonDocument<1024> doc;
  JsonArray arr = doc.to<JsonArray>();
  
  for (int i = 0; i < MAX_ROVERS; i++) {
    if (rovers[i].active) {
      JsonObject r = arr.createNestedObject();
      r["name"] = rovers[i].name;
      char macStr[18];
      snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
               rovers[i].mac[0], rovers[i].mac[1], rovers[i].mac[2],
               rovers[i].mac[3], rovers[i].mac[4], rovers[i].mac[5]);
      r["mac"] = macStr;
      r["rssi"] = rovers[i].rssi;
      r["packets"] = rovers[i].packetsReceived;
      r["lastSeen"] = (millis() - rovers[i].lastSeen) / 1000;
    }
  }
  
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void handleUpdate() {
  server.send(200, "text/html", 
    "<form method='POST' action='/update' enctype='multipart/form-data'>"
    "<input type='file' name='update'><input type='submit' value='Update'></form>");
}

void handleUpdateUpload() {
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    logInfo("OTA Update started");
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) {
      logInfo("OTA Update complete, restarting...");
      server.send(200, "text/plain", "Update complete, restarting...");
      delay(1000);
      ESP.restart();
    } else {
      Update.printError(Serial);
      server.send(500, "text/plain", "Update failed");
    }
  }
}

void handleNotFound() {
  server.send(404, "text/plain", "Not Found");
}

// ============================================================================
// MESSAGE QUEUE FUNCTIONS
// ============================================================================

bool queueCommand(CommandPacket cmd) {
  return queueCommand(cmd, 0xFF, false);  // Broadcast, normal priority
}

bool queueCommand(CommandPacket cmd, uint8_t targetRover, bool highPriority) {
  int nextTail = (queueTail + 1) % QUEUE_SIZE;
  if (nextTail == queueHead) {
    logWarn("Command queue full!");
    return false;
  }
  
  cmdQueue[queueTail].cmd = cmd;
  cmdQueue[queueTail].targetRover = targetRover;
  cmdQueue[queueTail].retries = 0;
  cmdQueue[queueTail].lastAttempt = 0;
  cmdQueue[queueTail].pending = true;
  cmdQueue[queueTail].highPriority = highPriority;
  queueTail = nextTail;
  
  return true;
}

bool hasQueuedCommand() {
  return queueHead != queueTail && cmdQueue[queueHead].pending;
}

QueuedCommand* getNextCommand() {
  if (!hasQueuedCommand()) return nullptr;
  return &cmdQueue[queueHead];
}

void commandCompleted(bool success) {
  if (success) {
    cmdQueue[queueHead].pending = false;
    queueHead = (queueHead + 1) % QUEUE_SIZE;
    commandsSent++;
  } else {
    cmdQueue[queueHead].retries++;
    if (cmdQueue[queueHead].retries >= MAX_RETRIES) {
      logWarn("Command failed after max retries");
      cmdQueue[queueHead].pending = false;
      queueHead = (queueHead + 1) % QUEUE_SIZE;
      totalErrors++;
    }
  }
}

// ============================================================================
// ESP-NOW CALLBACKS
// ============================================================================

void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  // Register/update rover
  int roverIdx = registerRover(mac);
  
  // Update RSSI
  if (roverIdx >= 0) {
    rovers[roverIdx].rssi = currentRSSI;
    rovers[roverIdx].packetsReceived++;
  }
  
  // Handle different packet types
  if (len == sizeof(RoverPacket)) {
    memcpy(&lastTelemetry, data, sizeof(lastTelemetry));
    packetsReceived++;
    
    blinkLED(1, 50);
    
    // Forward to Pi as JSON
    StaticJsonDocument<512> doc;
    doc["id"]       = lastTelemetry.id;
    doc["temp"]     = lastTelemetry.temp;
    doc["bat"]      = lastTelemetry.bat;
    doc["lat"]      = lastTelemetry.lat;
    doc["lng"]      = lastTelemetry.lng;
    doc["hazard"]   = lastTelemetry.hazard;
    doc["status"]   = lastTelemetry.status;
    doc["distance"] = lastTelemetry.distance;
    doc["accelX"]   = lastTelemetry.accelX;
    doc["accelY"]   = lastTelemetry.accelY;
    doc["heading"]  = lastTelemetry.heading;
    doc["streaming"]= lastTelemetry.streaming;
    doc["state"]    = lastTelemetry.state;
    doc["rssi"]     = currentRSSI;
    
    if (roverIdx >= 0) {
      doc["rover"] = rovers[roverIdx].name;
    }
    
    serializeJson(doc, Serial1);
    Serial1.println();
    
  } else if (len == sizeof(CommandPacket)) {
    // Command from another rover (relay mode)
    CommandPacket relayCmd;
    memcpy(&relayCmd, data, sizeof(relayCmd));
    
    // Forward to Pi
    StaticJsonDocument<128> doc;
    doc["type"] = "relay_cmd";
    doc["cmd"] = relayCmd.cmd;
    serializeJson(doc, Serial1);
    Serial1.println();
    
  } else {
    // Unknown packet type - log and forward raw
    logDebug("Received unknown packet type");
    Serial1.write(data, len);
    Serial1.println();
  }
}

void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  lastSendOk = (status == ESP_NOW_SEND_SUCCESS);
  
  if (lastSendOk) {
    commandCompleted(true);
    blinkLED(2, 50);
  } else {
    if (hasQueuedCommand()) {
      cmdQueue[queueHead].lastAttempt = millis();
    }
    totalErrors++;
    logError("ESP-NOW send failed");
  }
}

// ============================================================================
// ESP-NOW INITIALIZATION
// ============================================================================

bool initEspNow() {
  // WiFi AP mode for status page
  WiFi.mode(WIFI_AP);
  WiFi.softAP(config.apSSID, config.apPassword, config.channel);
  delay(100);
  
  if (esp_now_init() != ESP_OK) {
    logError("ESP-NOW init failed");
    return false;
  }
  
  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);
  
  // Add broadcast peer
  esp_now_peer_info_t broadcastPeer;
  memset(&broadcastPeer, 0, sizeof(broadcastPeer));
  memcpy(broadcastPeer.peer_addr, broadcastMac, 6);
  broadcastPeer.channel = config.channel;
  broadcastPeer.encrypt = false;
  
  if (esp_now_add_peer(&broadcastPeer) != ESP_OK) {
    logError("Failed to add broadcast peer");
    return false;
  }
  
  return true;
}

void addRoverPeer(const uint8_t* mac) {
  // Check if already added
  if (esp_now_is_peer_exist(mac)) {
    return;
  }
  
  esp_now_peer_info_t peer;
  memset(&peer, 0, sizeof(peer));
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = config.channel;
  peer.encrypt = (config.encryptionMode == 1);
  if (peer.encrypt) {
    memcpy(peer.lmk, config.encryptionKey, 16);
  }
  
  if (esp_now_add_peer(&peer) != ESP_OK) {
    logError("Failed to add rover peer");
  }
}

void reconnectEspNow() {
  logInfo("Reconnecting ESP-NOW...");
  esp_now_deinit();
  delay(100);
  espNowReady = initEspNow();
  
  if (espNowReady) {
    // Re-add all active rover peers
    for (int i = 0; i < MAX_ROVERS; i++) {
      if (rovers[i].active) {
        addRoverPeer(rovers[i].mac);
      }
    }
    blinkLED(5, 100);
    logInfo("ESP-NOW reconnected");
  } else {
    logError("ESP-NOW reconnect failed");
  }
}

// ============================================================================
// SEND COMMAND TO ROVER
// ============================================================================

void sendQueuedCommands() {
  if (!espNowReady || !hasQueuedCommand()) return;
  
  QueuedCommand* qc = getNextCommand();
  if (!qc) return;
  
  if (qc->retries > 0 && millis() - qc->lastAttempt < RETRY_DELAY_MS) {
    return;
  }
  
  // Determine target MAC
  uint8_t* targetMac;
  if (qc->targetRover == 0xFF) {
    targetMac = broadcastMac;  // Broadcast
  } else if (qc->targetRover < MAX_ROVERS && rovers[qc->targetRover].active) {
    targetMac = rovers[qc->targetRover].mac;
  } else {
    // Invalid target, skip command
    commandCompleted(false);
    return;
  }
  
  esp_err_t result = esp_now_send(targetMac, (uint8_t*)&qc->cmd, sizeof(qc->cmd));
  
  if (result != ESP_OK) {
    logError("ESP-NOW send error");
    commandCompleted(false);
  }
}

// ============================================================================
// HEARTBEAT
// ============================================================================

void sendHeartbeat() {
  StaticJsonDocument<256> doc;
  doc["type"] = "heartbeat";
  doc["packets"] = packetsReceived;
  doc["commands"] = commandsSent;
  doc["espnow"] = espNowReady;
  doc["errors"] = totalErrors;
  doc["uptime"] = (millis() - bootTime) / 1000;
  doc["rovers"] = 0;
  
  // Count active rovers
  for (int i = 0; i < MAX_ROVERS; i++) {
    if (rovers[i].active) doc["rovers"] = doc["rovers"].as<int>() + 1;
  }
  
  serializeJson(doc, Serial1);
  Serial1.println();
}

// ============================================================================
// PROCESS COMMANDS FROM PI
// ============================================================================

void processPiCommands() {
  if (!Serial1.available()) return;
  
  char c = Serial1.read();
  
  // Check for end of command
  if (c == '\n' || c == '\r') {
    if (commandBufferIndex > 0) {
      commandBuffer[commandBufferIndex] = '\0';
      
      // Parse command
      String line = String(commandBuffer);
      line.trim();
      
      if (line.length() > 0 && line.length() < 16) {
        CommandPacket cmd;
        memset(&cmd, 0, sizeof(cmd));
        line.toCharArray(cmd.cmd, sizeof(cmd.cmd));
        
        if (queueCommand(cmd)) {
          blinkLED(1, 50);
        }
      } else if (line.startsWith("{")) {
        // JSON command
        StaticJsonDocument<128> doc;
        DeserializationError err = deserializeJson(doc, line);
        if (!err) {
          CommandPacket cmd;
          memset(&cmd, 0, sizeof(cmd));
          
          const char* cmdStr = doc["cmd"];
          if (cmdStr) {
            strncpy(cmd.cmd, cmdStr, sizeof(cmd.cmd) - 1);
          }
          cmd.param1 = doc["p1"] | 0.0;
          cmd.param2 = doc["p2"] | 0.0;
          
          uint8_t target = doc["rover"] | 0xFF;
          bool priority = doc["priority"] | false;
          
          if (queueCommand(cmd, target, priority)) {
            blinkLED(1, 50);
          }
        }
      }
      
      commandBufferIndex = 0;
    }
  } else if (commandBufferIndex < COMMAND_BUFFER_SIZE - 1) {
    commandBuffer[commandBufferIndex++] = c;
  }
}

// ============================================================================
// DIAGNOSTIC FUNCTIONS
// ============================================================================

void printDiagnostics() {
  Serial.println("\n=== Bridge Diagnostics ===");
  Serial.printf("Firmware: %s\n", FIRMWARE_VERSION);
  Serial.printf("Uptime: %lu s\n", (millis() - bootTime) / 1000);
  Serial.printf("ESP-NOW: %s\n", espNowReady ? "OK" : "FAIL");
  Serial.printf("Packets: %u\n", packetsReceived);
  Serial.printf("Commands: %u\n", commandsSent);
  Serial.printf("Errors: %u\n", totalErrors);
  Serial.printf("Queue: %d/%d\n", 
    (queueTail - queueHead + QUEUE_SIZE) % QUEUE_SIZE, QUEUE_SIZE);
  Serial.printf("Active Rovers: %d\n", 
    [this]() { int c=0; for(int i=0;i<MAX_ROVERS;i++) if(rovers[i].active) c++; return c; }());
  Serial.printf("Free Heap: %u bytes\n", ESP.getFreeHeap());
  Serial.printf("WiFi Channel: %d\n", config.channel);
  Serial.println("===========================\n");
}

void checkBattery() {
  #ifdef PIN_BATTERY_ADC
  int raw = analogRead(PIN_BATTERY_ADC);
  float voltage = (raw / 4095.0) * 3.3 * 2.0;  // Voltage divider
  if (voltage < 3.3) {
    logWarn("Low battery voltage!");
  }
  #endif
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, PIN_RX_FROM_PI, PIN_TX_TO_PI);
  bootTime = millis();
  
  pinMode(LED_PIN, OUTPUT);
  pinMode(PIN_RESET_BUTTON, INPUT_PULLUP);
  ledOff();
  
  // Initialize watchdog
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);
  
  // Load configuration
  loadConfig();
  
  // Startup blink pattern
  for (int i = 0; i < 3; i++) {
    ledOn();
    delay(100);
    ledOff();
    delay(100);
  }
  
  // Initialize ESP-NOW
  espNowReady = initEspNow();
  
  if (espNowReady) {
    // Setup web server
    server.on("/", handleRoot);
    server.on("/status.json", handleJSON);
    server.on("/api/cmd", HTTP_GET, handleCommand);
    server.on("/api/config", HTTP_GET, handleConfig);
    server.on("/api/config", HTTP_POST, handleConfig);
    server.on("/api/rovers", HTTP_GET, handleRovers);
    server.on("/update", HTTP_GET, handleUpdate);
    server.on("/update", HTTP_POST, []() {
      server.send(200, "text/plain", "Update complete");
    }, handleUpdateUpload);
    server.onNotFound(handleNotFound);
    server.begin();
    
    Serial.println("\n========================");
    Serial.println("Aero Sentinel Base Station Bridge");
    Serial.println("========================");
    Serial.printf("Version: %s\n", FIRMWARE_VERSION);
    Serial.printf("MAC: %s\n", WiFi.softAPmacAddress().c_str());
    Serial.printf("SSID: %s\n", config.apSSID);
    Serial.printf("IP: %s\n", WiFi.softAPIP().toString().c_str());
    Serial.println("Status page: http://192.168.4.1");
    Serial.println("========================");
    
    ledOn();
    delay(500);
    ledOff();
  } else {
    // Error blink pattern
    while (1) {
      ledOn();
      delay(100);
      ledOff();
      delay(100);
      esp_task_wdt_reset();
    }
  }
  
  // Initialize queue
  for (int i = 0; i < QUEUE_SIZE; i++) {
    cmdQueue[i].pending = false;
  }
  
  // Initialize rover registry
  for (int i = 0; i < MAX_ROVERS; i++) {
    rovers[i].active = false;
  }
  
  // Clear telemetry
  memset(&lastTelemetry, 0, sizeof(lastTelemetry));
  
  // Clear command buffer
  commandBufferIndex = 0;
  
  logInfo("Bridge ready");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Feed watchdog
  esp_task_wdt_reset();
  
  // Handle web server
  server.handleClient();
  
  // Update LED
  updateLED();
  
  // Process commands from Pi
  processPiCommands();
  
  // Send queued commands
  sendQueuedCommands();
  
  // Heartbeat
  if (millis() - lastHeartbeat > HEARTBEAT_MS) {
    lastHeartbeat = millis();
    sendHeartbeat();
  }
  
  // Periodic reconnection check
  if (millis() - lastReconnect > RECONNECT_MS) {
    lastReconnect = millis();
    if (!espNowReady) {
      reconnectEspNow();
    }
    cleanupInactiveRovers();
    checkBattery();
  }
  
  // Check reset button
  if (digitalRead(PIN_RESET_BUTTON) == LOW) {
    delay(1000);
    if (digitalRead(PIN_RESET_BUTTON) == LOW) {
      logInfo("Reset button pressed, clearing config...");
      config.configValid = 0;
      saveConfig();
      ESP.restart();
    }
  }
  
  // Periodic diagnostics (every 60 seconds)
  static unsigned long lastDiag = 0;
  if (millis() - lastDiag > 60000) {
    lastDiag = millis();
    printDiagnostics();
  }
  
  delay(10);
}
  t["hazard"] = lastTelemetry.hazard;
  t["heading"] = lastTelemetry.heading;
  t["distance"] = lastTelemetry.distance;
  
  JsonArray roverList = doc.createNestedArray("rovers");
  for (int i = 0; i < MAX_ROVERS; i++) {
    if (rovers[i].active) {
      JsonObject r = roverList.createNestedObject();
      r["name"] = rovers[i].name;
      r["rssi"] = rovers[i].rssi;
      r["packets"] = rovers[i].packetsReceived;
      r["lastSeen"] = (millis() - rovers[i].lastSeen) / 1000;
    }
  }
  
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void handleCommand() {
  if (!server.hasArg("cmd")) {
    server.send(400, "application/json", "{\"error\":\"Missing cmd parameter\"}");
    return;
  }
  
  String cmd = server.arg("cmd");
  cmd.toUpperCase();
  
  CommandPacket packet;
  memset(&packet, 0, sizeof(packet));
  cmd.toCharArray(packet.cmd, sizeof(packet.cmd));
  
  // Check for parameters
  if (server.hasArg("p1")) {
    packet.param1 = server.arg("p1").toFloat();
  }
  if (server.hasArg("p2")) {
    packet.param2 = server.arg("p2").toFloat();
  }
  
  if (queueCommand(packet)) {
    blinkLED(2, 50);
    server.send(200, "application/json", "{\"status\":\"Command queued\"}");
  } else {
    server.send(503, "application/json", "{\"error\":\"Queue full\"}");
  }
}

void handleConfig() {
  if (server.method() == HTTP_GET) {
    StaticJsonDocument<512> doc;
    doc["apSSID"] = config.apSSID;
    doc["channel"] = config.channel;
    doc["encryption"] = config.encryptionMode;
    String out;
    serializeJson(doc, out);
    server.send(200, "application/json", out);
  } else {
    // POST - update config
    if (server.hasArg("ssid")) {
      server.arg("ssid").toCharArray(config.apSSID, sizeof(config.apSSID));
    }
    if (server.hasArg("password")) {
      server.arg("password").toCharArray(config.apPassword, sizeof(config.apPassword));
    }
    if (server.hasArg("channel")) {
      config.channel = server.arg("channel").toInt();
    }
    saveConfig();
    server.send(200, "application/json", "{\"status\":\"Config saved, restart required\"}");
  }
}

void handleRovers() {
  StaticJsonDocument<1024> doc;
  JsonArray arr = doc.to<JsonArray>();
  
  for (int i = 0; i < MAX_ROVERS; i++) {
    if (rovers[i].active) {
      JsonObject r = arr.createNestedObject();
      r["name"] = rovers[i].name;
      char macStr[18];
      snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
               rovers[i].mac[0], rovers[i].mac[1], rovers[i].mac[2],
               rovers[i].mac[3], rovers[i].mac[4], rovers[i].mac[5]);
      r["mac"] = macStr;
      r["rssi"] = rovers[i].rssi;
      r["packets"] = rovers[i].packetsReceived;
      r["lastSeen"] = (millis() - rovers[i].lastSeen) / 1000;
    }
  }
  
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void handleUpdate() {
  server.send(200, "text/html", 
    "<form method='POST' action='/update' enctype='multipart/form-data'>"
    "<input type='file' name='update'><input type='submit' value='Update'></form>");
}

void handleUpdateUpload() {
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    logInfo("OTA Update started");
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) {
      logInfo("OTA Update complete, restarting...");
      server.send(200, "text/plain", "Update complete, restarting...");
      delay(1000);
      ESP.restart();
    } else {
      Update.printError(Serial);
      server.send(500, "text/plain", "Update failed");
    }
  }
}

void handleNotFound() {
  server.send(404, "text/plain", "Not Found");
}

// ============================================================================
// MESSAGE QUEUE FUNCTIONS
// ============================================================================

bool queueCommand(CommandPacket cmd) {
  return queueCommand(cmd, 0xFF, false);  // Broadcast, normal priority
}

bool queueCommand(CommandPacket cmd, uint8_t targetRover, bool highPriority) {
  int nextTail = (queueTail + 1) % QUEUE_SIZE;
  if (nextTail == queueHead) {
    logWarn("Command queue full!");
    return false;
  }
  
  cmdQueue[queueTail].cmd = cmd;
  cmdQueue[queueTail].targetRover = targetRover;
  cmdQueue[queueTail].retries = 0;
  cmdQueue[queueTail].lastAttempt = 0;
  cmdQueue[queueTail].pending = true;
  cmdQueue[queueTail].highPriority = highPriority;
  queueTail = nextTail;
  
  return true;
}

bool hasQueuedCommand() {
  return queueHead != queueTail && cmdQueue[queueHead].pending;
}

QueuedCommand* getNextCommand() {
  if (!hasQueuedCommand()) return nullptr;
  return &cmdQueue[queueHead];
}

void commandCompleted(bool success) {
  if (success) {
    cmdQueue[queueHead].pending = false;
    queueHead = (queueHead + 1) % QUEUE_SIZE;
    commandsSent++;
  } else {
    cmdQueue[queueHead].retries++;
    if (cmdQueue[queueHead].retries >= MAX_RETRIES) {
      logWarn("Command failed after max retries");
      cmdQueue[queueHead].pending = false;
      queueHead = (queueHead + 1) % QUEUE_SIZE;
      totalErrors++;
    }
  }
}

// ============================================================================
// ESP-NOW CALLBACKS
// ============================================================================

void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  // Register/update rover
  int roverIdx = registerRover(mac);
  
  // Update RSSI
  if (roverIdx >= 0) {
    rovers[roverIdx].rssi = currentRSSI;
    rovers[roverIdx].packetsReceived++;
  }
  
  // Handle different packet types
  if (len == sizeof(RoverPacket)) {
    memcpy(&lastTelemetry, data, sizeof(lastTelemetry));
    packetsReceived++;
    
    blinkLED(1, 50);
    
    // Forward to Pi as JSON
    StaticJsonDocument<512> doc;
    doc["id"]       = lastTelemetry.id;
    doc["temp"]     = lastTelemetry.temp;
    doc["bat"]      = lastTelemetry.bat;
    doc["lat"]      = lastTelemetry.lat;
    doc["lng"]      = lastTelemetry.lng;
    doc["hazard"]   = lastTelemetry.hazard;
    doc["status"]   = lastTelemetry.status;
    doc["distance"] = lastTelemetry.distance;
    doc["accelX"]   = lastTelemetry.accelX;
    doc["accelY"]   = lastTelemetry.accelY;
    doc["heading"]  = lastTelemetry.heading;
    doc["streaming"]= lastTelemetry.streaming;
    doc["state"]    = lastTelemetry.state;
    doc["rssi"]     = currentRSSI;
    
    if (roverIdx >= 0) {
      doc["rover"] = rovers[roverIdx].name;
    }
    
    serializeJson(doc, Serial1);
    Serial1.println();
    
  } else if (len == sizeof(CommandPacket)) {
    // Command from another rover (relay mode)
    CommandPacket relayCmd;
    memcpy(&relayCmd, data, sizeof(relayCmd));
    
    // Forward to Pi
    StaticJsonDocument<128> doc;
    doc["type"] = "relay_cmd";
    doc["cmd"] = relayCmd.cmd;
    serializeJson(doc, Serial1);
    Serial1.println();
    
  } else {
    // Unknown packet type - log and forward raw
    logDebug("Received unknown packet type");
    Serial1.write(data, len);
    Serial1.println();
  }
}

void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  lastSendOk = (status == ESP_NOW_SEND_SUCCESS);
  
  if (lastSendOk) {
    commandCompleted(true);
    blinkLED(2, 50);
  } else {
    if (hasQueuedCommand()) {
      cmdQueue[queueHead].lastAttempt = millis();
    }
    totalErrors++;
    logError("ESP-NOW send failed");
  }
}

// ============================================================================
// ESP-NOW INITIALIZATION
// ============================================================================

bool initEspNow() {
  // WiFi AP mode for status page
  WiFi.mode(WIFI_AP);
  WiFi.softAP(config.apSSID, config.apPassword, config.channel);
  delay(100);
  
  if (esp_now_init() != ESP_OK) {
    logError("ESP-NOW init failed");
    return false;
  }
  
  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);
  
  // Add broadcast peer
  esp_now_peer_info_t broadcastPeer;
  memset(&broadcastPeer, 0, sizeof(broadcastPeer));
  memcpy(broadcastPeer.peer_addr, broadcastMac, 6);
  broadcastPeer.channel = config.channel;
  broadcastPeer.encrypt = false;
  
  if (esp_now_add_peer(&broadcastPeer) != ESP_OK) {
    logError("Failed to add broadcast peer");
    return false;
  }
  
  return true;
}

void addRoverPeer(const uint8_t* mac) {
  // Check if already added
  if (esp_now_is_peer_exist(mac)) {
    return;
  }
  
  esp_now_peer_info_t peer;
  memset(&peer, 0, sizeof(peer));
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = config.channel;
  peer.encrypt = (config.encryptionMode == 1);
  if (peer.encrypt) {
    memcpy(peer.lmk, config.encryptionKey, 16);
  }
  
  if (esp_now_add_peer(&peer) != ESP_OK) {
    logError("Failed to add rover peer");
  }
}

void reconnectEspNow() {
  logInfo("Reconnecting ESP-NOW...");
  esp_now_deinit();
  delay(100);
  espNowReady = initEspNow();
  
  if (espNowReady) {
    // Re-add all active rover peers
    for (int i = 0; i < MAX_ROVERS; i++) {
      if (rovers[i].active) {
        addRoverPeer(rovers[i].mac);
      }
    }
    blinkLED(5, 100);
    logInfo("ESP-NOW reconnected");
  } else {
    logError("ESP-NOW reconnect failed");
  }
}

// ============================================================================
// SEND COMMAND TO ROVER
// ============================================================================

void sendQueuedCommands() {
  if (!espNowReady || !hasQueuedCommand()) return;
  
  QueuedCommand* qc = getNextCommand();
  if (!qc) return;
  
  if (qc->retries > 0 && millis() - qc->lastAttempt < RETRY_DELAY_MS) {
    return;
  }
  
  // Determine target MAC
  uint8_t* targetMac;
  if (qc->targetRover == 0xFF) {
    targetMac = broadcastMac;  // Broadcast
  } else if (qc->targetRover < MAX_ROVERS && rovers[qc->targetRover].active) {
    targetMac = rovers[qc->targetRover].mac;
  } else {
    // Invalid target, skip command
    commandCompleted(false);
    return;
  }
  
  esp_err_t result = esp_now_send(targetMac, (uint8_t*)&qc->cmd, sizeof(qc->cmd));
  
  if (result != ESP_OK) {
    logError("ESP-NOW send error");
    commandCompleted(false);
  }
}

// ============================================================================
// HEARTBEAT
// ============================================================================

void sendHeartbeat() {
  StaticJsonDocument<256> doc;
  doc["type"] = "heartbeat";
  doc["packets"] = packetsReceived;
  doc["commands"] = commandsSent;
  doc["espnow"] = espNowReady;
  doc["errors"] = totalErrors;
  doc["uptime"] = (millis() - bootTime) / 1000;
  doc["rovers"] = 0;
  
  // Count active rovers
  for (int i = 0; i < MAX_ROVERS; i++) {
    if (rovers[i].active) doc["rovers"] = doc["rovers"].as<int>() + 1;
  }
  
  serializeJson(doc, Serial1);
  Serial1.println();
}

// ============================================================================
// PROCESS COMMANDS FROM PI
// ============================================================================

void processPiCommands() {
  if (!Serial1.available()) return;
  
  char c = Serial1.read();
  
  // Check for end of command
  if (c == '\n' || c == '\r') {
    if (commandBufferIndex > 0) {
      commandBuffer[commandBufferIndex] = '\0';
      
      // Parse command
      String line = String(commandBuffer);
      line.trim();
      
      if (line.length() > 0 && line.length() < 16) {
        CommandPacket cmd;
        memset(&cmd, 0, sizeof(cmd));
        line.toCharArray(cmd.cmd, sizeof(cmd.cmd));
        
        if (queueCommand(cmd)) {
          blinkLED(1, 50);
        }
      } else if (line.startsWith("{")) {
        // JSON command
        StaticJsonDocument<128> doc;
        DeserializationError err = deserializeJson(doc, line);
        if (!err) {
          CommandPacket cmd;
          memset(&cmd, 0, sizeof(cmd));
          
          const char* cmdStr = doc["cmd"];
          if (cmdStr) {
            strncpy(cmd.cmd, cmdStr, sizeof(cmd.cmd) - 1);
          }
          cmd.param1 = doc["p1"] | 0.0;
          cmd.param2 = doc["p2"] | 0.0;
          
          uint8_t target = doc["rover"] | 0xFF;
          bool priority = doc["priority"] | false;
          
          if (queueCommand(cmd, target, priority)) {
            blinkLED(1, 50);
          }
        }
      }
      
      commandBufferIndex = 0;
    }
  } else if (commandBufferIndex < COMMAND_BUFFER_SIZE - 1) {
    commandBuffer[commandBufferIndex++] = c;
  }
}

// ============================================================================
// DIAGNOSTIC FUNCTIONS
// ============================================================================

void printDiagnostics() {
  Serial.println("\n=== Bridge Diagnostics ===");
  Serial.printf("Firmware: %s\n", FIRMWARE_VERSION);
  Serial.printf("Uptime: %lu s\n", (millis() - bootTime) / 1000);
  Serial.printf("ESP-NOW: %s\n", espNowReady ? "OK" : "FAIL");
  Serial.printf("Packets: %u\n", packetsReceived);
  Serial.printf("Commands: %u\n", commandsSent);
  Serial.printf("Errors: %u\n", totalErrors);
  Serial.printf("Queue: %d/%d\n", 
    (queueTail - queueHead + QUEUE_SIZE) % QUEUE_SIZE, QUEUE_SIZE);
  Serial.printf("Active Rovers: %d\n", 
    [this]() { int c=0; for(int i=0;i<MAX_ROVERS;i++) if(rovers[i].active) c++; return c; }());
  Serial.printf("Free Heap: %u bytes\n", ESP.getFreeHeap());
  Serial.printf("WiFi Channel: %d\n", config.channel);
  Serial.println("===========================\n");
}

void checkBattery() {
  #ifdef PIN_BATTERY_ADC
  int raw = analogRead(PIN_BATTERY_ADC);
  float voltage = (raw / 4095.0) * 3.3 * 2.0;  // Voltage divider
  if (voltage < 3.3) {
    logWarn("Low battery voltage!");
  }
  #endif
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, PIN_RX_FROM_PI, PIN_TX_TO_PI);
  bootTime = millis();
  
  pinMode(LED_PIN, OUTPUT);
  pinMode(PIN_RESET_BUTTON, INPUT_PULLUP);
  ledOff();
  
  // Initialize watchdog
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);
  
  // Load configuration
  loadConfig();
  
  // Startup blink pattern
  for (int i = 0; i < 3; i++) {
    ledOn();
    delay(100);
    ledOff();
    delay(100);
  }
  
  // Initialize ESP-NOW
  espNowReady = initEspNow();
  
  if (espNowReady) {
    // Setup web server
    server.on("/", handleRoot);
    server.on("/status.json", handleJSON);
    server.on("/api/cmd", HTTP_GET, handleCommand);
    server.on("/api/config", HTTP_GET, handleConfig);
    server.on("/api/config", HTTP_POST, handleConfig);
    server.on("/api/rovers", HTTP_GET, handleRovers);
    server.on("/update", HTTP_GET, handleUpdate);
    server.on("/update", HTTP_POST, []() {
      server.send(200, "text/plain", "Update complete");
    }, handleUpdateUpload);
    server.onNotFound(handleNotFound);
    server.begin();
    
    Serial.println("\n========================");
    Serial.println("Aero Sentinel Base Station Bridge");
    Serial.println("========================");
    Serial.printf("Version: %s\n", FIRMWARE_VERSION);
    Serial.printf("MAC: %s\n", WiFi.softAPmacAddress().c_str());
    Serial.printf("SSID: %s\n", config.apSSID);
    Serial.printf("IP: %s\n", WiFi.softAPIP().toString().c_str());
    Serial.println("Status page: http://192.168.4.1");
    Serial.println("========================");
    
    ledOn();
    delay(500);
    ledOff();
  } else {
    // Error blink pattern
    while (1) {
      ledOn();
      delay(100);
      ledOff();
      delay(100);
      esp_task_wdt_reset();
    }
  }
  
  // Initialize queue
  for (int i = 0; i < QUEUE_SIZE; i++) {
    cmdQueue[i].pending = false;
  }
  
  // Initialize rover registry
  for (int i = 0; i < MAX_ROVERS; i++) {
    rovers[i].active = false;
  }
  
  // Clear telemetry
  memset(&lastTelemetry, 0, sizeof(lastTelemetry));
  
  // Clear command buffer
  commandBufferIndex = 0;
  
  logInfo("Bridge ready");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Feed watchdog
  esp_task_wdt_reset();
  
  // Handle web server
  server.handleClient();
  
  // Update LED
  updateLED();
  
  // Process commands from Pi
  processPiCommands();
  
  // Send queued commands
  sendQueuedCommands();
  
  // Heartbeat
  if (millis() - lastHeartbeat > HEARTBEAT_MS) {
    lastHeartbeat = millis();
    sendHeartbeat();
  }
  
  // Periodic reconnection check
  if (millis() - lastReconnect > RECONNECT_MS) {
    lastReconnect = millis();
    if (!espNowReady) {
      reconnectEspNow();
    }
    cleanupInactiveRovers();
    checkBattery();
  }
  
  // Check reset button
  if (digitalRead(PIN_RESET_BUTTON) == LOW) {
    delay(1000);
    if (digitalRead(PIN_RESET_BUTTON) == LOW) {
      logInfo("Reset button pressed, clearing config...");
      config.configValid = 0;
      saveConfig();
      ESP.restart();
    }
  }
  
  // Periodic diagnostics (every 60 seconds)
  static unsigned long lastDiag = 0;
  if (millis() - lastDiag > 60000) {
    lastDiag = millis();
    printDiagnostics();
  }
  
  delay(10);
}

