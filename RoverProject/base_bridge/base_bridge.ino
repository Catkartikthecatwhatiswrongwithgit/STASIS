/**
 * STASIS - BASE STATION BRIDGE (ESP32-C3)
 * 
 * Bidirectional Communication:
 * - Rover → Pi: ESP-NOW receive → UART transmit (JSON)
 * - Pi → Rover: UART receive → ESP-NOW transmit
 * 
 * Features:
 * - Message queuing for reliability
 * - Status LED for operational feedback
 * - WiFi AP for status page
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ArduinoJson.h>
#include <WebServer.h>

// --- PINS ---
#define PIN_RX  20
#define PIN_TX  21
#define LED_PIN 8

// --- CONSTANTS ---
#define QUEUE_SIZE 10
#define HEARTBEAT_MS 5000

const char* AP_SSID = "STASIS-Base";
const char* AP_PASS = "stasis123";

// --- DATA STRUCTURES (must match rover_main.ino) ---
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
} RoverPacket;

typedef struct __attribute__((packed)) {
  char cmd[16];
} CommandPacket;

// --- QUEUE ---
typedef struct {
  CommandPacket cmd;
  uint8_t retries;
  bool pending;
} QueuedCommand;

QueuedCommand cmdQueue[QUEUE_SIZE];
int qHead = 0, qTail = 0;

// --- GLOBALS ---
RoverPacket lastPkt;
esp_now_peer_info_t roverPeer;
uint8_t roverMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

bool espNowOk = false;
uint32_t pktCount = 0, cmdCount = 0;
unsigned long lastBeat = 0;

WebServer server(80);

// --- LED ---
void blink(int n) {
  for (int i=0; i<n; i++) {
    digitalWrite(LED_PIN, HIGH); delay(50);
    digitalWrite(LED_PIN, LOW); delay(50);
  }
}

// --- WEB HANDLERS ---
void handleRoot() {
  String html = "<!DOCTYPE html><html><head>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<meta http-equiv='refresh' content='5'>"
    "<title>STASIS Base Station</title>"
    "<style>"
    "body{font-family:Arial;background:#0a0a1a;color:#eee;padding:20px}"
    ".c{max-width:500px;margin:0 auto}h1{color:#00d4ff;text-align:center}"
    ".card{background:#1a1a2e;border-radius:8px;padding:15px;margin:10px 0}"
    ".r{display:flex;justify-content:space-between;padding:5px 0}"
    ".v{color:#0f8}.bad{color:#f44}"
    "</style></head><body><div class='c'>"
    "<h1>STASIS</h1>"
    "<div class='card'><h2>Status</h2>"
    "<div class='r'><span>ESP-NOW:</span><span class='" + String(espNowOk?"v":"bad") + "'>" + String(espNowOk?"OK":"FAIL") + "</span></div>"
    "<div class='r'><span>Packets:</span><span class='v'>" + String(pktCount) + "</span></div>"
    "<div class='r'><span>Commands:</span><span class='v'>" + String(cmdCount) + "</span></div>"
    "</div><div class='card'><h2>Last Telemetry</h2>"
    "<div class='r'><span>Status:</span><span class='v'>" + String(lastPkt.status) + "</span></div>"
    "<div class='r'><span>Temp:</span><span class='v'>" + String(lastPkt.temp,1) + " C</span></div>"
    "<div class='r'><span>Battery:</span><span class='v'>" + String(lastPkt.bat,0) + "%</span></div>"
    "<div class='r'><span>Location:</span><span class='v'>" + String(lastPkt.lat,4) + "," + String(lastPkt.lng,4) + "</span></div>"
    "<div class='r'><span>Hazard:</span><span class='" + String(lastPkt.hazard?"bad":"v") + "'>" + String(lastPkt.hazard?"ALERT!":"None") + "</span></div>"
    "</div></div></body></html>";
  server.send(200, "text/html", html);
}

void handleJSON() {
  StaticJsonDocument<400> doc;
  doc["espnow"] = espNowOk;
  doc["packets"] = pktCount;
  doc["commands"] = cmdCount;
  doc["uptime"] = millis()/1000;
  JsonObject t = doc.createNestedObject("telemetry");
  t["status"] = lastPkt.status;
  t["temp"] = lastPkt.temp;
  t["bat"] = lastPkt.bat;
  t["lat"] = lastPkt.lat;
  t["lng"] = lastPkt.lng;
  t["hazard"] = lastPkt.hazard;
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

// --- QUEUE FUNCTIONS ---
bool queueCmd(CommandPacket c) {
  int next = (qTail+1) % QUEUE_SIZE;
  if (next == qHead) return false;
  cmdQueue[qTail].cmd = c;
  cmdQueue[qTail].retries = 0;
  cmdQueue[qTail].pending = true;
  qTail = next;
  return true;
}

bool hasCmd() { return qHead != qTail && cmdQueue[qHead].pending; }

void cmdDone(bool ok) {
  if (ok) {
    cmdQueue[qHead].pending = false;
    qHead = (qHead+1) % QUEUE_SIZE;
    cmdCount++;
  } else {
    cmdQueue[qHead].retries++;
    if (cmdQueue[qHead].retries > 3) {
      cmdQueue[qHead].pending = false;
      qHead = (qHead+1) % QUEUE_SIZE;
    }
  }
}

// --- ESP-NOW CALLBACKS ---
void onRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len != sizeof(RoverPacket)) return;
  memcpy(&lastPkt, data, sizeof(lastPkt));
  pktCount++;
  blink(1);
  
  // Send JSON to Pi
  StaticJsonDocument<400> doc;
  doc["id"] = lastPkt.id;
  doc["temp"] = lastPkt.temp;
  doc["bat"] = lastPkt.bat;
  doc["lat"] = lastPkt.lat;
  doc["lng"] = lastPkt.lng;
  doc["hazard"] = lastPkt.hazard;
  doc["status"] = lastPkt.status;
  doc["distance"] = lastPkt.distance;
  doc["accelX"] = lastPkt.accelX;
  doc["accelY"] = lastPkt.accelY;
  serializeJson(doc, Serial1);
  Serial1.println();
}

void onSent(const uint8_t *mac, esp_now_send_status_t status) {
  cmdDone(status == ESP_NOW_SEND_SUCCESS);
  if (status == ESP_NOW_SEND_SUCCESS) blink(2);
}

// --- INIT ESP-NOW ---
bool initEN() {
  WiFi.disconnect();
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(100);
  
  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_recv_cb(onRecv);
  esp_now_register_send_cb(onSent);
  
  memcpy(roverPeer.peer_addr, roverMac, 6);
  roverPeer.channel = 0;
  roverPeer.encrypt = false;
  return esp_now_add_peer(&roverPeer) == ESP_OK;
}

// --- SEND QUEUED COMMANDS ---
void sendCmds() {
  if (!espNowOk || !hasCmd()) return;
  esp_now_send(roverMac, (uint8_t*)&cmdQueue[qHead].cmd, sizeof(CommandPacket));
}

// --- PROCESS PI COMMANDS ---
void procPi() {
  if (!Serial1.available()) return;
  String line = Serial1.readStringUntil('\n');
  line.trim();
  if (line.length() == 0 || line.length() >= 16) return;
  
  CommandPacket c;
  memset(&c, 0, sizeof(c));
  line.toCharArray(c.cmd, 16);
  queueCmd(c);
  blink(1);
}

// --- HEARTBEAT ---
void heartbeat() {
  StaticJsonDocument<128> doc;
  doc["type"] = "heartbeat";
  doc["packets"] = pktCount;
  doc["uptime"] = millis()/1000;
  serializeJson(doc, Serial1);
  Serial1.println();
}

// --- SETUP ---
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, PIN_RX, PIN_TX);
  
  pinMode(LED_PIN, OUTPUT);
  blink(3);
  
  espNowOk = initEN();
  
  if (espNowOk) {
    server.on("/", handleRoot);
    server.on("/json", handleJSON);
    server.begin();
    
    Serial.println("\n========================");
    Serial.println("STASIS Base Station Bridge");
    Serial.println("========================");
    Serial.printf("MAC: %s\n", WiFi.softAPmacAddress().c_str());
    Serial.printf("SSID: %s\n", AP_SSID);
    Serial.printf("IP: %s\n", WiFi.softAPIP().toString().c_str());
    Serial.println("========================");
    
    digitalWrite(LED_PIN, HIGH); delay(500); digitalWrite(LED_PIN, LOW);
  } else {
    while(1) { digitalWrite(LED_PIN, HIGH); delay(100); digitalWrite(LED_PIN, LOW); delay(100); }
  }
  
  memset(&lastPkt, 0, sizeof(lastPkt));
}

// --- LOOP ---
void loop() {
  server.handleClient();
  
  // Process commands from Pi
  procPi();
  
  // Send queued commands
  sendCmds();
  
  // Heartbeat
  if (millis() - lastBeat > HEARTBEAT_MS) {
    lastBeat = millis();
    heartbeat();
  }
  
  delay(10);
}
    ".r{display:flex;justify-content:space-between;padding:5px 0}"
    ".v{color:#0f8}.bad{color:#f44}"
    "</style></head><body><div class='c'>"
    "<h1>STASIS</h1>"
    "<div class='card'><h2>Status</h2>"
    "<div class='r'><span>ESP-NOW:</span><span class='" + String(espNowOk?"v":"bad") + "'>" + String(espNowOk?"OK":"FAIL") + "</span></div>"
    "<div class='r'><span>Packets:</span><span class='v'>" + String(pktCount) + "</span></div>"
    "<div class='r'><span>Commands:</span><span class='v'>" + String(cmdCount) + "</span></div>"
    "</div><div class='card'><h2>Last Telemetry</h2>"
    "<div class='r'><span>Status:</span><span class='v'>" + String(lastPkt.status) + "</span></div>"
    "<div class='r'><span>Temp:</span><span class='v'>" + String(lastPkt.temp,1) + " C</span></div>"
    "<div class='r'><span>Battery:</span><span class='v'>" + String(lastPkt.bat,0) + "%</span></div>"
    "<div class='r'><span>Location:</span><span class='v'>" + String(lastPkt.lat,4) + "," + String(lastPkt.lng,4) + "</span></div>"
    "<div class='r'><span>Hazard:</span><span class='" + String(lastPkt.hazard?"bad":"v") + "'>" + String(lastPkt.hazard?"ALERT!":"None") + "</span></div>"
    "</div></div></body></html>";
  server.send(200, "text/html", html);
}

void handleJSON() {
  StaticJsonDocument<400> doc;
  doc["espnow"] = espNowOk;
  doc["packets"] = pktCount;
  doc["commands"] = cmdCount;
  doc["uptime"] = millis()/1000;
  JsonObject t = doc.createNestedObject("telemetry");
  t["status"] = lastPkt.status;
  t["temp"] = lastPkt.temp;
  t["bat"] = lastPkt.bat;
  t["lat"] = lastPkt.lat;
  t["lng"] = lastPkt.lng;
  t["hazard"] = lastPkt.hazard;
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

// --- QUEUE FUNCTIONS ---
bool queueCmd(CommandPacket c) {
  int next = (qTail+1) % QUEUE_SIZE;
  if (next == qHead) return false;
  cmdQueue[qTail].cmd = c;
  cmdQueue[qTail].retries = 0;
  cmdQueue[qTail].pending = true;
  qTail = next;
  return true;
}

bool hasCmd() { return qHead != qTail && cmdQueue[qHead].pending; }

void cmdDone(bool ok) {
  if (ok) {
    cmdQueue[qHead].pending = false;
    qHead = (qHead+1) % QUEUE_SIZE;
    cmdCount++;
  } else {
    cmdQueue[qHead].retries++;
    if (cmdQueue[qHead].retries > 3) {
      cmdQueue[qHead].pending = false;
      qHead = (qHead+1) % QUEUE_SIZE;
    }
  }
}

// --- ESP-NOW CALLBACKS ---
void onRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len != sizeof(RoverPacket)) return;
  memcpy(&lastPkt, data, sizeof(lastPkt));
  pktCount++;
  blink(1);
  
  // Send JSON to Pi
  StaticJsonDocument<400> doc;
  doc["id"] = lastPkt.id;
  doc["temp"] = lastPkt.temp;
  doc["bat"] = lastPkt.bat;
  doc["lat"] = lastPkt.lat;
  doc["lng"] = lastPkt.lng;
  doc["hazard"] = lastPkt.hazard;
  doc["status"] = lastPkt.status;
  doc["distance"] = lastPkt.distance;
  doc["accelX"] = lastPkt.accelX;
  doc["accelY"] = lastPkt.accelY;
  serializeJson(doc, Serial1);
  Serial1.println();
}

void onSent(const uint8_t *mac, esp_now_send_status_t status) {
  cmdDone(status == ESP_NOW_SEND_SUCCESS);
  if (status == ESP_NOW_SEND_SUCCESS) blink(2);
}

// --- INIT ESP-NOW ---
bool initEN() {
  WiFi.disconnect();
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(100);
  
  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_recv_cb(onRecv);
  esp_now_register_send_cb(onSent);
  
  memcpy(roverPeer.peer_addr, roverMac, 6);
  roverPeer.channel = 0;
  roverPeer.encrypt = false;
  return esp_now_add_peer(&roverPeer) == ESP_OK;
}

// --- SEND QUEUED COMMANDS ---
void sendCmds() {
  if (!espNowOk || !hasCmd()) return;
  esp_now_send(roverMac, (uint8_t*)&cmdQueue[qHead].cmd, sizeof(CommandPacket));
}

// --- PROCESS PI COMMANDS ---
void procPi() {
  if (!Serial1.available()) return;
  String line = Serial1.readStringUntil('\n');
  line.trim();
  if (line.length() == 0 || line.length() >= 16) return;
  
  CommandPacket c;
  memset(&c, 0, sizeof(c));
  line.toCharArray(c.cmd, 16);
  queueCmd(c);
  blink(1);
}

// --- HEARTBEAT ---
void heartbeat() {
  StaticJsonDocument<128> doc;
  doc["type"] = "heartbeat";
  doc["packets"] = pktCount;
  doc["uptime"] = millis()/1000;
  serializeJson(doc, Serial1);
  Serial1.println();
}

// --- SETUP ---
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, PIN_RX, PIN_TX);
  
  pinMode(LED_PIN, OUTPUT);
  blink(3);
  
  espNowOk = initEN();
  
  if (espNowOk) {
    server.on("/", handleRoot);
    server.on("/json", handleJSON);
    server.begin();
    
    Serial.println("\n========================");
    Serial.println("STASIS Base Station Bridge");
    Serial.println("========================");
    Serial.printf("MAC: %s\n", WiFi.softAPmacAddress().c_str());
    Serial.printf("SSID: %s\n", AP_SSID);
    Serial.printf("IP: %s\n", WiFi.softAPIP().toString().c_str());
    Serial.println("========================");
    
    digitalWrite(LED_PIN, HIGH); delay(500); digitalWrite(LED_PIN, LOW);
  } else {
    while(1) { digitalWrite(LED_PIN, HIGH); delay(100); digitalWrite(LED_PIN, LOW); delay(100); }
  }
  
  memset(&lastPkt, 0, sizeof(lastPkt));
}

// --- LOOP ---
void loop() {
  server.handleClient();
  
  // Process commands from Pi
  procPi();
  
  // Send queued commands
  sendCmds();
  
  // Heartbeat
  if (millis() - lastBeat > HEARTBEAT_MS) {
    lastBeat = millis();
    heartbeat();
  }
  
  delay(10);
}

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
