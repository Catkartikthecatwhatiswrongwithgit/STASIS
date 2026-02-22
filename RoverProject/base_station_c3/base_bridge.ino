/**
 * ============================================================================
 * AERO SENTINEL - BASE STATION BRIDGE (ESP32-C3)
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
 * - Automatic reconnection on ESP-NOW failure
 * - Heartbeat signal for connection monitoring
 * - WiFi Access Point for status indication
 * - Web server for real-time status display
 * 
 * ============================================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ArduinoJson.h>
#include <WebServer.h>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
#define PIN_RX_FROM_PI  20
#define PIN_TX_TO_PI    21
#define LED_PIN         8   // Built-in LED on most ESP32-C3 boards

// ============================================================================
// CONSTANTS
// ============================================================================
#define QUEUE_SIZE        10
#define HEARTBEAT_MS      5000
#define RECONNECT_MS      30000
#define MAX_RETRIES       3
#define RETRY_DELAY_MS    100

// WiFi AP Configuration
const char* AP_SSID = "AeroSentinel-Base";
const char* AP_PASSWORD = "sentinel123";

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
} RoverPacket;

typedef struct __attribute__((packed)) {
  char cmd[16];
} CommandPacket;

// ============================================================================
// MESSAGE QUEUE
// ============================================================================
typedef struct {
  CommandPacket cmd;
  uint8_t       retries;
  unsigned long lastAttempt;
  bool          pending;
} QueuedCommand;

QueuedCommand cmdQueue[QUEUE_SIZE];
int queueHead = 0;
int queueTail = 0;

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================
RoverPacket          incomingData;
RoverPacket          lastTelemetry;
esp_now_peer_info_t  roverPeer;
uint8_t              roverMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

unsigned long lastHeartbeat = 0;
unsigned long lastReconnect = 0;
bool          espNowReady   = false;
bool          lastSendOk    = true;
uint32_t      packetsReceived = 0;
uint32_t      commandsSent = 0;

// LED blink patterns
int           ledBlinkCount = 0;
unsigned long ledBlinkTime = 0;

// Web Server for status page
WebServer statusServer(80);

// ============================================================================
// LED CONTROL
// ============================================================================
void ledOn() {
  digitalWrite(LED_PIN, HIGH);
}

void ledOff() {
  digitalWrite(LED_PIN, LOW);
}

void blinkLED(int count, int delayMs) {
  ledBlinkCount = count * 2;
  ledBlinkTime = millis();
  ledOff();
}

void updateLED() {
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
}

// ============================================================================
// WEB SERVER HANDLERS
// ============================================================================
void handleRoot() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>Aero Sentinel - Base Station Status</title>";
  html += "<style>";
  html += "body{font-family:Arial,sans-serif;margin:20px;background:#1a1a2e;color:#eee;}";
  html += ".container{max-width:600px;margin:0 auto;}";
  html += "h1{color:#00d4ff;text-align:center;}";
  html += ".card{background:#16213e;border-radius:10px;padding:20px;margin:10px 0;}";
  html += ".status{display:flex;justify-content:space-between;padding:10px 0;border-bottom:1px solid #333;}";
  html += ".label{color:#888;}.value{color:#00ff88;font-weight:bold;}";
  html += ".online{color:#00ff88;}.offline{color:#ff4444;}";
  html += ".refresh-btn{background:#00d4ff;color:#000;border:none;padding:10px 20px;border-radius:5px;cursor:pointer;}";
  html += ".refresh-btn:hover{background:#00a8cc;}";
  html += "meta{http-equiv:'refresh';content:'5';}";
  html += "</style></head><body>";
  html += "<div class='container'>";
  html += "<h1>Aero Sentinel</h1>";
  
  // Connection Status Card
  html += "<div class='card'>";
  html += "<h2>Connection Status</h2>";
  html += "<div class='status'><span class='label'>ESP-NOW:</span>";
  html += "<span class='" + String(espNowReady ? "online" : "offline") + "'>";
  html += String(espNowReady ? "Connected" : "Disconnected") + "</span></div>";
  html += "<div class='status'><span class='label'>Packets Received:</span>";
  html += "<span class='value'>" + String(packetsReceived) + "</span></div>";
  html += "<div class='status'><span class='label'>Commands Sent:</span>";
  html += "<span class='value'>" + String(commandsSent) + "</span></div>";
  html += "</div>";
  
  // Last Telemetry Card
  html += "<div class='card'>";
  html += "<h2>Last Telemetry</h2>";
  html += "<div class='status'><span class='label'>Status:</span>";
  html += "<span class='value'>" + String(lastTelemetry.status) + "</span></div>";
  html += "<div class='status'><span class='label'>Temperature:</span>";
  html += "<span class='value'>" + String(lastTelemetry.temp, 1) + " C</span></div>";
  html += "<div class='status'><span class='label'>Battery:</span>";
  html += "<span class='value'>" + String(lastTelemetry.bat, 1) + "%</span></div>";
  html += "<div class='status'><span class='label'>Location:</span>";
  html += "<span class='value'>" + String(lastTelemetry.lat, 6) + ", " + String(lastTelemetry.lng, 6) + "</span></div>";
  html += "<div class='status'><span class='label'>Hazard:</span>";
  html += "<span class='" + String(lastTelemetry.hazard ? "offline" : "online") + "'>";
  html += String(lastTelemetry.hazard ? "ALERT!" : "None") + "</span></div>";
  html += "<div class='status'><span class='label'>Distance:</span>";
  html += "<span class='value'>" + String(lastTelemetry.distance, 1) + " cm</span></div>";
  html += "</div>";
  
  // System Info Card
  html += "<div class='card'>";
  html += "<h2>System Info</h2>";
  html += "<div class='status'><span class='label'>Device:</span>";
  html += "<span class='value'>ESP32-C3 Bridge</span></div>";
  html += "<div class='status'><span class='label'>AP SSID:</span>";
  html += "<span class='value'>" + String(AP_SSID) + "</span></div>";
  html += "<div class='status'><span class='label'>Uptime:</span>";
  html += "<span class='value'>" + String(millis() / 1000) + " sec</span></div>";
  html += "</div>";
  
  html += "<button class='refresh-btn' onclick='location.reload()'>Refresh</button>";
  html += "</div></body></html>";
  
  statusServer.send(200, "text/html", html);
}

void handleStatusJSON() {
  StaticJsonDocument<512> doc;
  doc["espnow_ready"] = espNowReady;
  doc["packets_received"] = packetsReceived;
  doc["commands_sent"] = commandsSent;
  doc["uptime_ms"] = millis();
  
  JsonObject telemetry = doc.createNestedObject("last_telemetry");
  telemetry["status"] = lastTelemetry.status;
  telemetry["temp"] = lastTelemetry.temp;
  telemetry["bat"] = lastTelemetry.bat;
  telemetry["lat"] = lastTelemetry.lat;
  telemetry["lng"] = lastTelemetry.lng;
  telemetry["hazard"] = lastTelemetry.hazard;
  telemetry["distance"] = lastTelemetry.distance;
  
  String response;
  serializeJson(doc, response);
  statusServer.send(200, "application/json", response);
}

void handleNotFound() {
  statusServer.send(404, "text/plain", "Not Found");
}

void initWebServer() {
  statusServer.on("/", handleRoot);
  statusServer.on("/status.json", handleStatusJSON);
  statusServer.onNotFound(handleNotFound);
  statusServer.begin();
}

// ============================================================================
// MESSAGE QUEUE FUNCTIONS
// ============================================================================
bool queueCommand(CommandPacket cmd) {
  int nextTail = (queueTail + 1) % QUEUE_SIZE;
  if (nextTail == queueHead) {
    // Queue full
    return false;
  }
  
  cmdQueue[queueTail].cmd = cmd;
  cmdQueue[queueTail].retries = 0;
  cmdQueue[queueTail].lastAttempt = 0;
  cmdQueue[queueTail].pending = true;
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
    // Remove from queue
    cmdQueue[queueHead].pending = false;
    queueHead = (queueHead + 1) % QUEUE_SIZE;
    commandsSent++;
  } else {
    // Increment retry count
    cmdQueue[queueHead].retries++;
    if (cmdQueue[queueHead].retries >= MAX_RETRIES) {
      // Give up, remove from queue
      cmdQueue[queueHead].pending = false;
      queueHead = (queueHead + 1) % QUEUE_SIZE;
    }
  }
}

// ============================================================================
// ESP-NOW CALLBACKS
// ============================================================================

void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len != sizeof(RoverPacket)) return;
  
  memcpy(&incomingData, data, sizeof(incomingData));
  memcpy(&lastTelemetry, &incomingData, sizeof(RoverPacket));
  packetsReceived++;
  
  // Blink LED to indicate data received
  blinkLED(1, 50);
  
  // Create JSON document
  StaticJsonDocument<512> doc;
  doc["id"]       = incomingData.id;
  doc["temp"]     = incomingData.temp;
  doc["bat"]      = incomingData.bat;
  doc["lat"]      = incomingData.lat;
  doc["lng"]      = incomingData.lng;
  doc["hazard"]   = incomingData.hazard;
  doc["status"]   = incomingData.status;
  doc["distance"] = incomingData.distance;
  doc["accelX"]   = incomingData.accelX;
  doc["accelY"]   = incomingData.accelY;
  doc["heading"]  = incomingData.heading;
  doc["streaming"] = incomingData.streaming;
  
  // Send to Pi via UART
  serializeJson(doc, Serial1);
  Serial1.println();
}

void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  lastSendOk = (status == ESP_NOW_SEND_SUCCESS);
  
  if (lastSendOk) {
    commandCompleted(true);
    blinkLED(2, 50);  // Two blinks for successful send
  } else {
    // Will retry
    if (hasQueuedCommand()) {
      cmdQueue[queueHead].lastAttempt = millis();
    }
  }
}

// ============================================================================
// ESP-NOW INITIALIZATION
// ============================================================================
bool initEspNow() {
  // ESP-NOW requires STA mode but we'll use AP+STA for combined functionality
  // First, disconnect any existing WiFi
  WiFi.disconnect();
  
  // Set to AP mode for status page (ESP-NOW can work alongside)
  WiFi.mode(WIFI_AP);
  
  // Create AP
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  
  // Small delay to let AP settle
  delay(100);
  
  if (esp_now_init() != ESP_OK) {
    return false;
  }
  
  // Register callbacks
  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);
  
  // Add rover as peer
  memcpy(roverPeer.peer_addr, roverMac, 6);
  roverPeer.channel = 0;
  roverPeer.encrypt = false;
  
  if (esp_now_add_peer(&roverPeer) != ESP_OK) {
    return false;
  }
  
  return true;
}

void reconnectEspNow() {
  esp_now_deinit();
  delay(100);
  espNowReady = initEspNow();
  
  if (espNowReady) {
    blinkLED(5, 100);  // 5 blinks for reconnection
  }
}

// ============================================================================
// SEND COMMAND TO ROVER
// ============================================================================
void sendQueuedCommands() {
  if (!espNowReady || !hasQueuedCommand()) return;
  
  QueuedCommand* qc = getNextCommand();
  if (!qc) return;
  
  // Check retry delay
  if (qc->retries > 0 && millis() - qc->lastAttempt < RETRY_DELAY_MS) {
    return;
  }
  
  // Send command
  esp_err_t result = esp_now_send(roverMac, (uint8_t*)&qc->cmd, sizeof(qc->cmd));
  
  if (result != ESP_OK) {
    // Send failed immediately
    commandCompleted(false);
  }
}

// ============================================================================
// HEARTBEAT
// ============================================================================
void sendHeartbeat() {
  StaticJsonDocument<128> doc;
  doc["type"] = "heartbeat";
  doc["packets"] = packetsReceived;
  doc["commands"] = commandsSent;
  doc["espnow"] = espNowReady;
  doc["uptime"] = millis() / 1000;
  
  serializeJson(doc, Serial1);
  Serial1.println();
}

// ============================================================================
// PROCESS COMMANDS FROM PI
// ============================================================================
void processPiCommands() {
  if (!Serial1.available()) return;
  
  String line = Serial1.readStringUntil('\n');
  line.trim();
  
  if (line.length() == 0 || line.length() >= 16) return;
  
  // Create command packet
  CommandPacket cmd;
  memset(&cmd, 0, sizeof(cmd));
  line.toCharArray(cmd.cmd, 16);
  
  // Queue for sending
  if (queueCommand(cmd)) {
    blinkLED(1, 50);
  }
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  // Initialize serial ports
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, PIN_RX_FROM_PI, PIN_TX_TO_PI);
  
  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  ledOff();
  
  // Startup blink pattern
  for (int i = 0; i < 3; i++) {
    ledOn();
    delay(100);
    ledOff();
    delay(100);
  }
  
  // Initialize ESP-NOW and WiFi AP
  espNowReady = initEspNow();
  
  if (espNowReady) {
    // Initialize web server
    initWebServer();
    
    // Success: long blink
    ledOn();
    delay(500);
    ledOff();
    
    // Print MAC address
    Serial.println("");
    Serial.println("========================================");
    Serial.println("Aero Sentinel - Base Station Bridge");
    Serial.println("========================================");
    Serial.print("MAC Address: ");
    Serial.println(WiFi.softAPmacAddress());
    Serial.print("AP SSID: ");
    Serial.println(AP_SSID);
    Serial.print("AP Password: ");
    Serial.println(AP_PASSWORD);
    Serial.print("Status Page: http://");
    Serial.println(WiFi.softAPIP());
    Serial.println("========================================");
  } else {
    // Error: continuous fast blink
    while (1) {
      ledOn();
      delay(100);
      ledOff();
      delay(100);
    }
  }
  
  // Clear queue
  for (int i = 0; i < QUEUE_SIZE; i++) {
    cmdQueue[i].pending = false;
  }
  
  // Initialize lastTelemetry with defaults
  memset(&lastTelemetry, 0, sizeof(RoverPacket));
  strcpy(lastTelemetry.status, "WAITING");
  
  Serial.println("Bridge ready");
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  // Update LED blink pattern
  updateLED();
  
  // Handle web server requests
  statusServer.handleClient();
  
  // Process commands from Pi
  processPiCommands();
  
  // Send queued commands to rover
  sendQueuedCommands();
  
  // Periodic heartbeat
  if (millis() - lastHeartbeat > HEARTBEAT_MS) {
    lastHeartbeat = millis();
    sendHeartbeat();
  }
  
  // Periodic ESP-NOW reconnection check
  if (millis() - lastReconnect > RECONNECT_MS) {
    lastReconnect = millis();
    if (!espNowReady) {
      reconnectEspNow();
    }
  }
  
  // Small delay to prevent tight loop
  delay(10);
}
