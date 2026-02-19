/**
 * STASIS - ESP32-CAM VISION MODULE
 * 
 * Features:
 * - Fire detection (color analysis)
 * - Motion detection (frame differencing)
 * - Human detection (skin-tone patterns)
 * - Frame streaming on command
 * 
 * Commands from ESP32-S3:
 * - 0x01: CAPTURE - Take image and return detection
 * - 0x02: STREAM_ON - Start continuous streaming
 * - 0x03: STREAM_OFF - Stop streaming
 * - 0x04: DETECT - Run detection only
 * 
 * Output: "HAZARD:FIRE", "HAZARD:MOTION", "HAZARD:HUMAN", or "CLEAR"
 */

#include <Arduino.h>
#include <esp_camera.h>

// --- CAMERA PINS (AI Thinker ESP32-CAM) ---
#define PWDN_GPIO_NUM  32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  0
#define SIOD_GPIO_NUM  26
#define SIOC_GPIO_NUM  27
#define Y9_GPIO_NUM    35
#define Y8_GPIO_NUM    34
#define Y7_GPIO_NUM    39
#define Y6_GPIO_NUM    36
#define Y5_GPIO_NUM    21
#define Y4_GPIO_NUM    19
#define Y3_GPIO_NUM    18
#define Y2_GPIO_NUM    5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM  23
#define PCLK_GPIO_NUM  22
#define LED_PIN        4

// --- COMMANDS ---
enum { CMD_CAPTURE=1, CMD_STREAM_ON=2, CMD_STREAM_OFF=3, CMD_DETECT=4 };

// --- DETECTION RESULT ---
typedef struct __attribute__((packed)) {
  bool  fire;
  bool  motion;
  bool  human;
  float confidence;
  char  desc[24];
} DetectionResult;

// --- THRESHOLDS ---
#define FIRE_THRESH    15
#define MOTION_THRESH  30
#define MOTION_SENS    4000

// --- GLOBALS ---
static uint16_t prevFrame[320*240/50];
static bool streaming = false;
static bool camReady = false;

// --- CAMERA INIT ---
bool initCam() {
  camera_config_t c;
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM;  c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM;  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;  c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM;  c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM;  c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM; c.pin_href = HREF_GPIO_NUM;
  c.pin_sscb_sda = SIOD_GPIO_NUM; c.pin_sscb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM; c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_RGB565;
  c.frame_size = FRAMESIZE_QVGA;
  c.jpeg_quality = 12;
  c.fb_count = 2;
  
  for (int i=0; i<3; i++) {
    if (esp_camera_init(&c) == ESP_OK) { camReady = true; return true; }
    delay(500);
  }
  return false;
}

// --- ANALYZE FRAME ---
DetectionResult analyze(camera_fb_t *fb) {
  DetectionResult r = {false, false, false, 0.0, "CLEAR"};
  if (!fb || !fb->buf) { strcpy(r.desc, "ERROR"); return r; }
  
  int firePx=0, motionPx=0, humanPx=0;
  int step = 50, samples = (fb->width * fb->height) / step;
  
  for (int i=0; i < fb->width * fb->height; i += step) {
    uint16_t px = ((uint16_t*)fb->buf)[i];
    uint8_t red = (px >> 11) & 0x1F;
    uint8_t grn = (px >> 5) & 0x3F;
    uint8_t blu = px & 0x1F;
    
    // Fire: high red, medium green, low blue
    if (red > 25 && grn > 10 && blu < 8) firePx++;
    
    // Motion: compare with previous
    int idx = i/step;
    if (idx < sizeof(prevFrame)/2) {
      if (abs((int)px - prevFrame[idx]) > MOTION_SENS) motionPx++;
      prevFrame[idx] = px;
    }
    
    // Human: skin tone approximation
    if (red > 15 && grn > 8 && grn < 20 && blu > 5 && blu < 15) humanPx++;
  }
  
  // Set results
  if (firePx > FIRE_THRESH) { r.fire = true; strcpy(r.desc, "FIRE DETECTED"); }
  if (motionPx > MOTION_THRESH) { 
    r.motion = true; 
    if (r.fire) strcat(r.desc, "+MOTION"); else strcpy(r.desc, "MOTION DETECTED");
  }
  if (humanPx > FIRE_THRESH*2) { 
    r.human = true; 
    if (r.fire || r.motion) strcat(r.desc, "+HUMAN"); else strcpy(r.desc, "HUMAN DETECTED");
  }
  
  r.confidence = (float)(firePx + motionPx + humanPx) / samples * 100;
  return r;
}

// --- CAPTURE & SEND ---
void capture() {
  if (!camReady) { Serial.println("ERROR:CAM"); return; }
  
  digitalWrite(LED_PIN, LOW);
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) { Serial.println("ERROR:CAP"); digitalWrite(LED_PIN, HIGH); return; }
  
  DetectionResult r = analyze(fb);
  esp_camera_fb_return(fb);
  
  Serial.write((uint8_t*)&r, sizeof(r));
  Serial.println();
  
  if (r.fire) Serial.println("HAZARD:FIRE");
  else if (r.motion) Serial.println("HAZARD:MOTION");
  else if (r.human) Serial.println("HAZARD:HUMAN");
  else Serial.println("CLEAR");
  
  digitalWrite(LED_PIN, HIGH);
}

// --- STREAM FRAME ---
void streamFrame() {
  if (!camReady || !streaming) return;
  
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;
  
  Serial.printf("FRAME:%dx%d:%d:", fb->width, fb->height, fb->len);
  Serial.write(fb->buf, fb->len);
  Serial.println();
  
  esp_camera_fb_return(fb);
}

// --- COMMAND HANDLER ---
void handleCmd(uint8_t cmd) {
  switch (cmd) {
    case CMD_CAPTURE:    capture(); break;
    case CMD_STREAM_ON:  streaming = true; Serial.println("STREAM:ON"); break;
    case CMD_STREAM_OFF: streaming = false; Serial.println("STREAM:OFF"); break;
    case CMD_DETECT:     capture(); break;
    default: Serial.printf("ERR:CMD:%02X\n", cmd);
  }
}

// --- SETUP ---
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
  // Startup blinks
  for (int i=0; i<3; i++) { digitalWrite(LED_PIN, LOW); delay(100); digitalWrite(LED_PIN, HIGH); delay(100); }
  
  if (initCam()) {
    Serial.println("CAM:READY");
    digitalWrite(LED_PIN, LOW); delay(500); digitalWrite(LED_PIN, HIGH);
  } else {
    Serial.println("CAM:ERROR");
    while (1) { digitalWrite(LED_PIN, LOW); delay(200); digitalWrite(LED_PIN, HIGH); delay(200); }
  }
  
  memset(prevFrame, 0, sizeof(prevFrame));
}

// --- LOOP ---
void loop() {
  if (Serial.available()) handleCmd(Serial.read());
  
  if (streaming) { streamFrame(); delay(50); }
  
  delay(10);
}
  int firePx=0, motionPx=0, humanPx=0;
  int step = 50, samples = (fb->width * fb->height) / step;
  
  for (int i=0; i < fb->width * fb->height; i += step) {
    uint16_t px = ((uint16_t*)fb->buf)[i];
    uint8_t red = (px >> 11) & 0x1F;
    uint8_t grn = (px >> 5) & 0x3F;
    uint8_t blu = px & 0x1F;
    
    // Fire: high red, medium green, low blue
    if (red > 25 && grn > 10 && blu < 8) firePx++;
    
    // Motion: compare with previous
    int idx = i/step;
    if (idx < sizeof(prevFrame)/2) {
      if (abs((int)px - prevFrame[idx]) > MOTION_SENS) motionPx++;
      prevFrame[idx] = px;
    }
    
    // Human: skin tone approximation
    if (red > 15 && grn > 8 && grn < 20 && blu > 5 && blu < 15) humanPx++;
  }
  
  // Set results
  if (firePx > FIRE_THRESH) { r.fire = true; strcpy(r.desc, "FIRE DETECTED"); }
  if (motionPx > MOTION_THRESH) { 
    r.motion = true; 
    if (r.fire) strcat(r.desc, "+MOTION"); else strcpy(r.desc, "MOTION DETECTED");
  }
  if (humanPx > FIRE_THRESH*2) { 
    r.human = true; 
    if (r.fire || r.motion) strcat(r.desc, "+HUMAN"); else strcpy(r.desc, "HUMAN DETECTED");
  }
  
  r.confidence = (float)(firePx + motionPx + humanPx) / samples * 100;
  return r;
}

// --- CAPTURE & SEND ---
void capture() {
  if (!camReady) { Serial.println("ERROR:CAM"); return; }
  
  digitalWrite(LED_PIN, LOW);
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) { Serial.println("ERROR:CAP"); digitalWrite(LED_PIN, HIGH); return; }
  
  DetectionResult r = analyze(fb);
  esp_camera_fb_return(fb);
  
  Serial.write((uint8_t*)&r, sizeof(r));
  Serial.println();
  
  if (r.fire) Serial.println("HAZARD:FIRE");
  else if (r.motion) Serial.println("HAZARD:MOTION");
  else if (r.human) Serial.println("HAZARD:HUMAN");
  else Serial.println("CLEAR");
  
  digitalWrite(LED_PIN, HIGH);
}

// --- STREAM FRAME ---
void streamFrame() {
  if (!camReady || !streaming) return;
  
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;
  
  Serial.printf("FRAME:%dx%d:%d:", fb->width, fb->height, fb->len);
  Serial.write(fb->buf, fb->len);
  Serial.println();
  
  esp_camera_fb_return(fb);
}

// --- COMMAND HANDLER ---
void handleCmd(uint8_t cmd) {
  switch (cmd) {
    case CMD_CAPTURE:    capture(); break;
    case CMD_STREAM_ON:  streaming = true; Serial.println("STREAM:ON"); break;
    case CMD_STREAM_OFF: streaming = false; Serial.println("STREAM:OFF"); break;
    case CMD_DETECT:     capture(); break;
    default: Serial.printf("ERR:CMD:%02X\n", cmd);
  }
}

// --- SETUP ---
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
  // Startup blinks
  for (int i=0; i<3; i++) { digitalWrite(LED_PIN, LOW); delay(100); digitalWrite(LED_PIN, HIGH); delay(100); }
  
  if (initCam()) {
    Serial.println("CAM:READY");
    digitalWrite(LED_PIN, LOW); delay(500); digitalWrite(LED_PIN, HIGH);
  } else {
    Serial.println("CAM:ERROR");
    while (1) { digitalWrite(LED_PIN, LOW); delay(200); digitalWrite(LED_PIN, HIGH); delay(200); }
  }
  
  memset(prevFrame, 0, sizeof(prevFrame));
}

// --- LOOP ---
void loop() {
  if (Serial.available()) handleCmd(Serial.read());
  
  if (streaming) { streamFrame(); delay(50); }
  
  delay(10);
}

  if (firePixels > FIRE_PIXEL_THRESHOLD) {
    result.fire = true;
    result.confidence = fireRatio * 100.0;
    strcpy(result.description, "FIRE DETECTED");
  }
  
  if (motionPixels > MOTION_PIXEL_THRESHOLD) {
    result.motion = true;
    float motionConf = motionRatio * 100.0;
    if (result.confidence < motionConf) {
      result.confidence = motionConf;
    }
    if (result.fire) {
      strcat(result.description, " + MOTION");
    } else {
      strcpy(result.description, "MOTION DETECTED");
    }
  }
  
  // Human detection is less reliable, require higher threshold
  if (humanPixels > FIRE_PIXEL_THRESHOLD * 2) {
    result.human = true;
    if (result.fire || result.motion) {
      strcat(result.description, " + HUMAN");
    } else {
      strcpy(result.description, "HUMAN DETECTED");
    }
  }
  
  return result;
}

// ============================================================================
// CAPTURE AND SEND RESULT
// ============================================================================
void captureAndSend() {
  if (!cameraReady) {
    Serial.println("ERROR:CAMERA_NOT_READY");
    return;
  }
  
  // Blink LED to indicate capture
  digitalWrite(LED_PIN, LOW);  // LED on (active low)
  
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("ERROR:CAPTURE_FAILED");
    digitalWrite(LED_PIN, HIGH);
    return;
  }
  
  // Analyze frame
  DetectionResult result = analyzeFrame(fb);
  
  // Return frame buffer
  esp_camera_fb_return(fb);
  
  // Send result as binary struct
  Serial.write((uint8_t*)&result, sizeof(result));
  Serial.println();  // End marker
  
  // Also send human-readable text
  if (result.fire) {
    Serial.println("HAZARD:FIRE");
  } else if (result.motion) {
    Serial.println("HAZARD:MOTION");
  } else if (result.human) {
    Serial.println("HAZARD:HUMAN");
  } else {
    Serial.println("CLEAR");
  }
  
  digitalWrite(LED_PIN, HIGH);  // LED off
}

// ============================================================================
// STREAM FRAMES
// ============================================================================
void streamFrame() {
  if (!cameraReady || !streamingEnabled) return;
  
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;
  
  // Send frame header
  Serial.print("FRAME:");
  Serial.print(fb->width);
  Serial.print("x");
  Serial.print(fb->height);
  Serial.print(":");
  Serial.print(fb->len);
  Serial.print(":");
  
  // Send frame data in chunks
  const int chunkSize = 512;
  for (size_t i = 0; i < fb->len; i += chunkSize) {
    size_t sendLen = min((size_t)chunkSize, fb->len - i);
    Serial.write(fb->buf + i, sendLen);
  }
  Serial.println();
  
  esp_camera_fb_return(fb);
}

// ============================================================================
// DETECTION ONLY (No image capture)
// ============================================================================
void detectOnly() {
  if (!cameraReady) {
    Serial.println("ERROR:CAMERA_NOT_READY");
    return;
  }
  
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("ERROR:CAPTURE_FAILED");
    return;
  }
  
  DetectionResult result = analyzeFrame(fb);
  esp_camera_fb_return(fb);
  
  // Send binary result
  Serial.write((uint8_t*)&result, sizeof(result));
  Serial.println();
}

// ============================================================================
// COMMAND HANDLER
// ============================================================================
void handleCommand(uint8_t cmd) {
  switch (cmd) {
    case CMD_CAPTURE:
      captureAndSend();
      break;
      
    case CMD_STREAM_ON:
      streamingEnabled = true;
      Serial.println("STREAM:ON");
      break;
      
    case CMD_STREAM_OFF:
      streamingEnabled = false;
      Serial.println("STREAM:OFF");
      break;
      
    case CMD_DETECT:
      detectOnly();
      break;
      
    default:
      Serial.print("ERROR:UNKNOWN_CMD:");
      Serial.println(cmd, HEX);
      break;
  }
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  
  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // LED off (active low)
  
  // Blink LED to indicate startup
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, LOW);
    delay(100);
    digitalWrite(LED_PIN, HIGH);
    delay(100);
  }
  
  // Initialize camera
  if (initCamera()) {
    Serial.println("CAM:READY");
    // Success blink
    digitalWrite(LED_PIN, LOW);
    delay(500);
    digitalWrite(LED_PIN, HIGH);
  } else {
    Serial.println("CAM:ERROR");
    // Error blink continuously
    while (1) {
      digitalWrite(LED_PIN, LOW);
      delay(200);
      digitalWrite(LED_PIN, HIGH);
      delay(200);
    }
  }
  
  // Clear previous frame buffer
  memset(prevFrameBuf, 0, sizeof(prevFrameBuf));
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  // Check for commands from ESP32-S3
  if (Serial.available()) {
    uint8_t cmd = Serial.read();
    handleCommand(cmd);
  }
  
  // Stream frames if enabled
  if (streamingEnabled) {
    streamFrame();
    delay(50);  // ~20 FPS streaming
  }
  
  // Small delay to prevent tight loop
  delay(10);
}

// ============================================================================
// CAMERA INITIALIZATION
// ============================================================================
bool initCamera() {
  camera_config_t config;
  
  config.ledc_channel   = LEDC_CHANNEL_0;
  config.ledc_timer     = LEDC_TIMER_0;
  config.pin_d0         = Y2_GPIO_NUM;
  config.pin_d1         = Y3_GPIO_NUM;
  config.pin_d2         = Y4_GPIO_NUM;
  config.pin_d3         = Y5_GPIO_NUM;
  config.pin_d4         = Y6_GPIO_NUM;
  config.pin_d5         = Y7_GPIO_NUM;
  config.pin_d6         = Y8_GPIO_NUM;
  config.pin_d7         = Y9_GPIO_NUM;
  config.pin_xclk       = XCLK_GPIO_NUM;
  config.pin_pclk       = PCLK_GPIO_NUM;
  config.pin_vsync      = VSYNC_GPIO_NUM;
  config.pin_href       = HREF_GPIO_NUM;
  config.pin_sscb_sda   = SIOD_GPIO_NUM;
  config.pin_sscb_scl   = SIOC_GPIO_NUM;
  config.pin_pwdn       = PWDN_GPIO_NUM;
  config.pin_reset      = RESET_GPIO_NUM;
  config.xclk_freq_hz   = 20000000;
  config.pixel_format   = PIXFORMAT_RGB565;
  config.frame_size     = FRAMESIZE_QVGA;  // 320x240 for faster processing
  config.jpeg_quality   = 12;
  config.fb_count       = 2;  // Double buffer for streaming

  // Try to initialize camera with retry
  for (int attempt = 0; attempt < 3; attempt++) {
    if (esp_camera_init(&config) == ESP_OK) {
      cameraReady = true;
      return true;
    }
    delay(500);
  }
  
  return false;
}

// ============================================================================
// DETECTION ALGORITHMS
// ============================================================================

/**
 * Analyze frame for fire, motion, and human detection
 * Uses RGB565 format for efficient processing
 */
DetectionResult analyzeFrame(camera_fb_t *fb) {
  DetectionResult result;
  result.fire = false;
  result.motion = false;
  result.human = false;
  result.confidence = 0.0;
  strcpy(result.description, "CLEAR");

  if (!fb || !fb->buf) {
    strcpy(result.description, "ERROR: No frame");
    return result;
  }

  int width = fb->width;
  int height = fb->height;
  
  firePixels = 0;
  motionPixels = 0;
  humanPixels = 0;
  
  // Sample every 50th pixel for performance
  int sampleStep = 50;
  int totalSamples = (width * height) / sampleStep;
  
  for (int i = 0; i < width * height; i += sampleStep) {
    uint16_t pixel = ((uint16_t*)fb->buf)[i];
    
    // Extract RGB components from RGB565
    uint8_t r = (pixel >> 11) & 0x1F;  // 5 bits
    uint8_t g = (pixel >> 5) & 0x3F;   // 6 bits
    uint8_t b = pixel & 0x1F;          // 5 bits
    
    // --- FIRE DETECTION ---
    // Fire typically has high red, medium green, low blue
    if (r > FIRE_R_MIN && g > FIRE_G_MIN && b < FIRE_B_MAX) {
      firePixels++;
    }
    
    // --- MOTION DETECTION ---
    // Compare with previous frame
    int bufIndex = i / sampleStep;
    if (bufIndex < sizeof(prevFrameBuf) / sizeof(prevFrameBuf[0])) {
      int pixelDiff = abs((int)pixel - (int)prevFrameBuf[bufIndex]);
      if (pixelDiff > MOTION_SENSITIVITY) {
        motionPixels++;
      }
      prevFrameBuf[bufIndex] = pixel;
    }
    
    // --- HUMAN DETECTION (Simplified) ---
    // Look for skin-tone colors (simplified heuristic)
    // Skin typically has R > 15, G between 8-20, B between 5-15
    if (r > 15 && g > 8 && g < 20 && b > 5 && b < 15) {
      humanPixels++;
    }
  }
  
  // Calculate confidence based on pixel counts
  float fireRatio = (float)firePixels / totalSamples;
  float motionRatio = (float)motionPixels / totalSamples;
  float humanRatio = (float)humanPixels / totalSamples;
  
  // Determine detections
  if (firePixels > FIRE_PIXEL_THRESHOLD) {
    result.fire = true;
    result.confidence = fireRatio * 100.0;
    strcpy(result.description, "FIRE DETECTED");
  }
  
  if (motionPixels > MOTION_PIXEL_THRESHOLD) {
    result.motion = true;
    float motionConf = motionRatio * 100.0;
    if (result.confidence < motionConf) {
      result.confidence = motionConf;
    }
    if (result.fire) {
      strcat(result.description, " + MOTION");
    } else {
      strcpy(result.description, "MOTION DETECTED");
    }
  }
  
  // Human detection is less reliable, require higher threshold
  if (humanPixels > FIRE_PIXEL_THRESHOLD * 2) {
    result.human = true;
    if (result.fire || result.motion) {
      strcat(result.description, " + HUMAN");
    } else {
      strcpy(result.description, "HUMAN DETECTED");
    }
  }
  
  return result;
}

// ============================================================================
// CAPTURE AND SEND RESULT
// ============================================================================
void captureAndSend() {
  if (!cameraReady) {
    Serial.println("ERROR:CAMERA_NOT_READY");
    return;
  }
  
  // Blink LED to indicate capture
  digitalWrite(LED_PIN, LOW);  // LED on (active low)
  
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("ERROR:CAPTURE_FAILED");
    digitalWrite(LED_PIN, HIGH);
    return;
  }
  
  // Analyze frame
  DetectionResult result = analyzeFrame(fb);
  
  // Return frame buffer
  esp_camera_fb_return(fb);
  
  // Send result as binary struct
  Serial.write((uint8_t*)&result, sizeof(result));
  Serial.println();  // End marker
  
  // Also send human-readable text
  if (result.fire) {
    Serial.println("HAZARD:FIRE");
  } else if (result.motion) {
    Serial.println("HAZARD:MOTION");
  } else if (result.human) {
    Serial.println("HAZARD:HUMAN");
  } else {
    Serial.println("CLEAR");
  }
  
  digitalWrite(LED_PIN, HIGH);  // LED off
}

// ============================================================================
// STREAM FRAMES
// ============================================================================
void streamFrame() {
  if (!cameraReady || !streamingEnabled) return;
  
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;
  
  // Send frame header
  Serial.print("FRAME:");
  Serial.print(fb->width);
  Serial.print("x");
  Serial.print(fb->height);
  Serial.print(":");
  Serial.print(fb->len);
  Serial.print(":");
  
  // Send frame data in chunks
  const int chunkSize = 512;
  for (size_t i = 0; i < fb->len; i += chunkSize) {
    size_t sendLen = min((size_t)chunkSize, fb->len - i);
    Serial.write(fb->buf + i, sendLen);
  }
  Serial.println();
  
  esp_camera_fb_return(fb);
}

// ============================================================================
// DETECTION ONLY (No image capture)
// ============================================================================
void detectOnly() {
  if (!cameraReady) {
    Serial.println("ERROR:CAMERA_NOT_READY");
    return;
  }
  
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("ERROR:CAPTURE_FAILED");
    return;
  }
  
  DetectionResult result = analyzeFrame(fb);
  esp_camera_fb_return(fb);
  
  // Send binary result
  Serial.write((uint8_t*)&result, sizeof(result));
  Serial.println();
}

// ============================================================================
// COMMAND HANDLER
// ============================================================================
void handleCommand(uint8_t cmd) {
  switch (cmd) {
    case CMD_CAPTURE:
      captureAndSend();
      break;
      
    case CMD_STREAM_ON:
      streamingEnabled = true;
      Serial.println("STREAM:ON");
      break;
      
    case CMD_STREAM_OFF:
      streamingEnabled = false;
      Serial.println("STREAM:OFF");
      break;
      
    case CMD_DETECT:
      detectOnly();
      break;
      
    default:
      Serial.print("ERROR:UNKNOWN_CMD:");
      Serial.println(cmd, HEX);
      break;
  }
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  
  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // LED off (active low)
  
  // Blink LED to indicate startup
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, LOW);
    delay(100);
    digitalWrite(LED_PIN, HIGH);
    delay(100);
  }
  
  // Initialize camera
  if (initCamera()) {
    Serial.println("CAM:READY");
    // Success blink
    digitalWrite(LED_PIN, LOW);
    delay(500);
    digitalWrite(LED_PIN, HIGH);
  } else {
    Serial.println("CAM:ERROR");
    // Error blink continuously
    while (1) {
      digitalWrite(LED_PIN, LOW);
      delay(200);
      digitalWrite(LED_PIN, HIGH);
      delay(200);
    }
  }
  
  // Clear previous frame buffer
  memset(prevFrameBuf, 0, sizeof(prevFrameBuf));
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  // Check for commands from ESP32-S3
  if (Serial.available()) {
    uint8_t cmd = Serial.read();
    handleCommand(cmd);
  }
  
  // Stream frames if enabled
  if (streamingEnabled) {
    streamFrame();
    delay(50);  // ~20 FPS streaming
  }
  
  // Small delay to prevent tight loop
  delay(10);
}


// ============================================================================
// CAMERA INITIALIZATION
// ============================================================================
bool initCamera() {
  camera_config_t config;
  
  config.ledc_channel   = LEDC_CHANNEL_0;
  config.ledc_timer     = LEDC_TIMER_0;
  config.pin_d0         = Y2_GPIO_NUM;
  config.pin_d1         = Y3_GPIO_NUM;
  config.pin_d2         = Y4_GPIO_NUM;
  config.pin_d3         = Y5_GPIO_NUM;
  config.pin_d4         = Y6_GPIO_NUM;
  config.pin_d5         = Y7_GPIO_NUM;
  config.pin_d6         = Y8_GPIO_NUM;
  config.pin_d7         = Y9_GPIO_NUM;
  config.pin_xclk       = XCLK_GPIO_NUM;
  config.pin_pclk       = PCLK_GPIO_NUM;
  config.pin_vsync      = VSYNC_GPIO_NUM;
  config.pin_href       = HREF_GPIO_NUM;
  config.pin_sscb_sda   = SIOD_GPIO_NUM;
  config.pin_sscb_scl   = SIOC_GPIO_NUM;
  config.pin_pwdn       = PWDN_GPIO_NUM;
  config.pin_reset      = RESET_GPIO_NUM;
  config.xclk_freq_hz   = 20000000;
  config.pixel_format   = PIXFORMAT_RGB565;
  config.frame_size     = FRAMESIZE_QVGA;  // 320x240 for faster processing
  config.jpeg_quality   = 12;
  config.fb_count       = 2;  // Double buffer for streaming

  // Try to initialize camera with retry
  for (int attempt = 0; attempt < 3; attempt++) {
    if (esp_camera_init(&config) == ESP_OK) {
      cameraReady = true;
      return true;
    }
    delay(500);
  }
  
  return false;
}

// ============================================================================
// DETECTION ALGORITHMS
// ============================================================================

/**
 * Analyze frame for fire, motion, and human detection
 * Uses RGB565 format for efficient processing
 */
DetectionResult analyzeFrame(camera_fb_t *fb) {
  DetectionResult result;
  result.fire = false;
  result.motion = false;
  result.human = false;
  result.confidence = 0.0;
  strcpy(result.description, "CLEAR");

  if (!fb || !fb->buf) {
    strcpy(result.description, "ERROR: No frame");
    return result;
  }

  int width = fb->width;
  int height = fb->height;
  
  firePixels = 0;
  motionPixels = 0;
  humanPixels = 0;
  
  // Sample every 50th pixel for performance
  int sampleStep = 50;
  int totalSamples = (width * height) / sampleStep;
  
  for (int i = 0; i < width * height; i += sampleStep) {
    uint16_t pixel = ((uint16_t*)fb->buf)[i];
    
    // Extract RGB components from RGB565
    uint8_t r = (pixel >> 11) & 0x1F;  // 5 bits
    uint8_t g = (pixel >> 5) & 0x3F;   // 6 bits
    uint8_t b = pixel & 0x1F;          // 5 bits
    
    // --- FIRE DETECTION ---
    // Fire typically has high red, medium green, low blue
    if (r > FIRE_R_MIN && g > FIRE_G_MIN && b < FIRE_B_MAX) {
      firePixels++;
    }
    
    // --- MOTION DETECTION ---
    // Compare with previous frame
    int bufIndex = i / sampleStep;
    if (bufIndex < sizeof(prevFrameBuf) / sizeof(prevFrameBuf[0])) {
      int pixelDiff = abs((int)pixel - (int)prevFrameBuf[bufIndex]);
      if (pixelDiff > MOTION_SENSITIVITY) {
        motionPixels++;
      }
      prevFrameBuf[bufIndex] = pixel;
    }
    
    // --- HUMAN DETECTION (Simplified) ---
    // Look for skin-tone colors (simplified heuristic)
    // Skin typically has R > 15, G between 8-20, B between 5-15
    if (r > 15 && g > 8 && g < 20 && b > 5 && b < 15) {
      humanPixels++;
    }
  }
  
  // Calculate confidence based on pixel counts
  float fireRatio = (float)firePixels / totalSamples;
  float motionRatio = (float)motionPixels / totalSamples;
  float humanRatio = (float)humanPixels / totalSamples;
  
  // Determine detections
  if (firePixels > FIRE_PIXEL_THRESHOLD) {
    result.fire = true;
    result.confidence = fireRatio * 100.0;
    strcpy(result.description, "FIRE DETECTED");
  }
  
  if (motionPixels > MOTION_PIXEL_THRESHOLD) {
    result.motion = true;
    float motionConf = motionRatio * 100.0;
    if (result.confidence < motionConf) {
      result.confidence = motionConf;
    }
    if (result.fire) {
      strcat(result.description, " + MOTION");
    } else {
      strcpy(result.description, "MOTION DETECTED");
    }
  }
  
  // Human detection is less reliable, require higher threshold
  if (humanPixels > FIRE_PIXEL_THRESHOLD * 2) {
    result.human = true;
    if (result.fire || result.motion) {
      strcat(result.description, " + HUMAN");
    } else {
      strcpy(result.description, "HUMAN DETECTED");
    }
  }
  
  return result;
}

// ============================================================================
// CAPTURE AND SEND RESULT
// ============================================================================
void captureAndSend() {
  if (!cameraReady) {
    Serial.println("ERROR:CAMERA_NOT_READY");
    return;
  }
  
  // Blink LED to indicate capture
  digitalWrite(LED_PIN, LOW);  // LED on (active low)
  
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("ERROR:CAPTURE_FAILED");
    digitalWrite(LED_PIN, HIGH);
    return;
  }
  
  // Analyze frame
  DetectionResult result = analyzeFrame(fb);
  
  // Return frame buffer
  esp_camera_fb_return(fb);
  
  // Send result as binary struct
  Serial.write((uint8_t*)&result, sizeof(result));
  Serial.println();  // End marker
  
  // Also send human-readable text
  if (result.fire) {
    Serial.println("HAZARD:FIRE");
  } else if (result.motion) {
    Serial.println("HAZARD:MOTION");
  } else if (result.human) {
    Serial.println("HAZARD:HUMAN");
  } else {
    Serial.println("CLEAR");
  }
  
  digitalWrite(LED_PIN, HIGH);  // LED off
}

// ============================================================================
// STREAM FRAMES
// ============================================================================
void streamFrame() {
  if (!cameraReady || !streamingEnabled) return;
  
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;
  
  // Send frame header
  Serial.print("FRAME:");
  Serial.print(fb->width);
  Serial.print("x");
  Serial.print(fb->height);
  Serial.print(":");
  Serial.print(fb->len);
  Serial.print(":");
  
  // Send frame data in chunks
  const int chunkSize = 512;
  for (size_t i = 0; i < fb->len; i += chunkSize) {
    size_t sendLen = min((size_t)chunkSize, fb->len - i);
    Serial.write(fb->buf + i, sendLen);
  }
  Serial.println();
  
  esp_camera_fb_return(fb);
}

// ============================================================================
// DETECTION ONLY (No image capture)
// ============================================================================
void detectOnly() {
  if (!cameraReady) {
    Serial.println("ERROR:CAMERA_NOT_READY");
    return;
  }
  
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("ERROR:CAPTURE_FAILED");
    return;
  }
  
  DetectionResult result = analyzeFrame(fb);
  esp_camera_fb_return(fb);
  
  // Send binary result
  Serial.write((uint8_t*)&result, sizeof(result));
  Serial.println();
}

// ============================================================================
// COMMAND HANDLER
// ============================================================================
void handleCommand(uint8_t cmd) {
  switch (cmd) {
    case CMD_CAPTURE:
      captureAndSend();
      break;
      
    case CMD_STREAM_ON:
      streamingEnabled = true;
      Serial.println("STREAM:ON");
      break;
      
    case CMD_STREAM_OFF:
      streamingEnabled = false;
      Serial.println("STREAM:OFF");
      break;
      
    case CMD_DETECT:
      detectOnly();
      break;
      
    default:
      Serial.print("ERROR:UNKNOWN_CMD:");
      Serial.println(cmd, HEX);
      break;
  }
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  
  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // LED off (active low)
  
  // Blink LED to indicate startup
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, LOW);
    delay(100);
    digitalWrite(LED_PIN, HIGH);
    delay(100);
  }
  
  // Initialize camera
  if (initCamera()) {
    Serial.println("CAM:READY");
    // Success blink
    digitalWrite(LED_PIN, LOW);
    delay(500);
    digitalWrite(LED_PIN, HIGH);
  } else {
    Serial.println("CAM:ERROR");
    // Error blink continuously
    while (1) {
      digitalWrite(LED_PIN, LOW);
      delay(200);
      digitalWrite(LED_PIN, HIGH);
      delay(200);
    }
  }
  
  // Clear previous frame buffer
  memset(prevFrameBuf, 0, sizeof(prevFrameBuf));
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  // Check for commands from ESP32-S3
  if (Serial.available()) {
    uint8_t cmd = Serial.read();
    handleCommand(cmd);
  }
  
  // Stream frames if enabled
  if (streamingEnabled) {
    streamFrame();
    delay(50);  // ~20 FPS streaming
  }
  
  // Small delay to prevent tight loop
  delay(10);
}

// ============================================================================
// CAMERA INITIALIZATION
// ============================================================================
bool initCamera() {
  camera_config_t config;
  
  config.ledc_channel   = LEDC_CHANNEL_0;
  config.ledc_timer     = LEDC_TIMER_0;
  config.pin_d0         = Y2_GPIO_NUM;
  config.pin_d1         = Y3_GPIO_NUM;
  config.pin_d2         = Y4_GPIO_NUM;
  config.pin_d3         = Y5_GPIO_NUM;
  config.pin_d4         = Y6_GPIO_NUM;
  config.pin_d5         = Y7_GPIO_NUM;
  config.pin_d6         = Y8_GPIO_NUM;
  config.pin_d7         = Y9_GPIO_NUM;
  config.pin_xclk       = XCLK_GPIO_NUM;
  config.pin_pclk       = PCLK_GPIO_NUM;
  config.pin_vsync      = VSYNC_GPIO_NUM;
  config.pin_href       = HREF_GPIO_NUM;
  config.pin_sscb_sda   = SIOD_GPIO_NUM;
  config.pin_sscb_scl   = SIOC_GPIO_NUM;
  config.pin_pwdn       = PWDN_GPIO_NUM;
  config.pin_reset      = RESET_GPIO_NUM;
  config.xclk_freq_hz   = 20000000;
  config.pixel_format   = PIXFORMAT_RGB565;
  config.frame_size     = FRAMESIZE_QVGA;  // 320x240 for faster processing
  config.jpeg_quality   = 12;
  config.fb_count       = 2;  // Double buffer for streaming

  // Try to initialize camera with retry
  for (int attempt = 0; attempt < 3; attempt++) {
    if (esp_camera_init(&config) == ESP_OK) {
      cameraReady = true;
      return true;
    }
    delay(500);
  }
  
  return false;
}

// ============================================================================
// DETECTION ALGORITHMS
// ============================================================================

/**
 * Analyze frame for fire, motion, and human detection
 * Uses RGB565 format for efficient processing
 */
DetectionResult analyzeFrame(camera_fb_t *fb) {
  DetectionResult result;
  result.fire = false;
  result.motion = false;
  result.human = false;
  result.confidence = 0.0;
  strcpy(result.description, "CLEAR");

  if (!fb || !fb->buf) {
    strcpy(result.description, "ERROR: No frame");
    return result;
  }

  int width = fb->width;
  int height = fb->height;
  
  firePixels = 0;
  motionPixels = 0;
  humanPixels = 0;
  
  // Sample every 50th pixel for performance
  int sampleStep = 50;
  int totalSamples = (width * height) / sampleStep;
  
  for (int i = 0; i < width * height; i += sampleStep) {
    uint16_t pixel = ((uint16_t*)fb->buf)[i];
    
    // Extract RGB components from RGB565
    uint8_t r = (pixel >> 11) & 0x1F;  // 5 bits
    uint8_t g = (pixel >> 5) & 0x3F;   // 6 bits
    uint8_t b = pixel & 0x1F;          // 5 bits
    
    // --- FIRE DETECTION ---
    // Fire typically has high red, medium green, low blue
    if (r > FIRE_R_MIN && g > FIRE_G_MIN && b < FIRE_B_MAX) {
      firePixels++;
    }
    
    // --- MOTION DETECTION ---
    // Compare with previous frame
    int bufIndex = i / sampleStep;
    if (bufIndex < sizeof(prevFrameBuf) / sizeof(prevFrameBuf[0])) {
      int pixelDiff = abs((int)pixel - (int)prevFrameBuf[bufIndex]);
      if (pixelDiff > MOTION_SENSITIVITY) {
        motionPixels++;
      }
      prevFrameBuf[bufIndex] = pixel;
    }
    
    // --- HUMAN DETECTION (Simplified) ---
    // Look for skin-tone colors (simplified heuristic)
    // Skin typically has R > 15, G between 8-20, B between 5-15
    if (r > 15 && g > 8 && g < 20 && b > 5 && b < 15) {
      humanPixels++;
    }
  }
  
  // Calculate confidence based on pixel counts
  float fireRatio = (float)firePixels / totalSamples;
  float motionRatio = (float)motionPixels / totalSamples;
  float humanRatio = (float)humanPixels / totalSamples;
  
  // Determine detections
  if (firePixels > FIRE_PIXEL_THRESHOLD) {
    result.fire = true;
    result.confidence = fireRatio * 100.0;
    strcpy(result.description, "FIRE DETECTED");
  }
  
  if (motionPixels > MOTION_PIXEL_THRESHOLD) {
    result.motion = true;
    float motionConf = motionRatio * 100.0;
    if (result.confidence < motionConf) {
      result.confidence = motionConf;
    }
    if (result.fire) {
      strcat(result.description, " + MOTION");
    } else {
      strcpy(result.description, "MOTION DETECTED");
    }
  }
  
  // Human detection is less reliable, require higher threshold
  if (humanPixels > FIRE_PIXEL_THRESHOLD * 2) {
    result.human = true;
    if (result.fire || result.motion) {
      strcat(result.description, " + HUMAN");
    } else {
      strcpy(result.description, "HUMAN DETECTED");
    }
  }
  
  return result;
}

// ============================================================================
// CAPTURE AND SEND RESULT
// ============================================================================
void captureAndSend() {
  if (!cameraReady) {
    Serial.println("ERROR:CAMERA_NOT_READY");
    return;
  }
  
  // Blink LED to indicate capture
  digitalWrite(LED_PIN, LOW);  // LED on (active low)
  
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("ERROR:CAPTURE_FAILED");
    digitalWrite(LED_PIN, HIGH);
    return;
  }
  
  // Analyze frame
  DetectionResult result = analyzeFrame(fb);
  
  // Return frame buffer
  esp_camera_fb_return(fb);
  
  // Send result as binary struct
  Serial.write((uint8_t*)&result, sizeof(result));
  Serial.println();  // End marker
  
  // Also send human-readable text
  if (result.fire) {
    Serial.println("HAZARD:FIRE");
  } else if (result.motion) {
    Serial.println("HAZARD:MOTION");
  } else if (result.human) {
    Serial.println("HAZARD:HUMAN");
  } else {
    Serial.println("CLEAR");
  }
  
  digitalWrite(LED_PIN, HIGH);  // LED off
}

// ============================================================================
// STREAM FRAMES
// ============================================================================
void streamFrame() {
  if (!cameraReady || !streamingEnabled) return;
  
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;
  
  // Send frame header
  Serial.print("FRAME:");
  Serial.print(fb->width);
  Serial.print("x");
  Serial.print(fb->height);
  Serial.print(":");
  Serial.print(fb->len);
  Serial.print(":");
  
  // Send frame data in chunks
  const int chunkSize = 512;
  for (size_t i = 0; i < fb->len; i += chunkSize) {
    size_t sendLen = min((size_t)chunkSize, fb->len - i);
    Serial.write(fb->buf + i, sendLen);
  }
  Serial.println();
  
  esp_camera_fb_return(fb);
}

// ============================================================================
// DETECTION ONLY (No image capture)
// ============================================================================
void detectOnly() {
  if (!cameraReady) {
    Serial.println("ERROR:CAMERA_NOT_READY");
    return;
  }
  
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("ERROR:CAPTURE_FAILED");
    return;
  }
  
  DetectionResult result = analyzeFrame(fb);
  esp_camera_fb_return(fb);
  
  // Send binary result
  Serial.write((uint8_t*)&result, sizeof(result));
  Serial.println();
}

// ============================================================================
// COMMAND HANDLER
// ============================================================================
void handleCommand(uint8_t cmd) {
  switch (cmd) {
    case CMD_CAPTURE:
      captureAndSend();
      break;
      
    case CMD_STREAM_ON:
      streamingEnabled = true;
      Serial.println("STREAM:ON");
      break;
      
    case CMD_STREAM_OFF:
      streamingEnabled = false;
      Serial.println("STREAM:OFF");
      break;
      
    case CMD_DETECT:
      detectOnly();
      break;
      
    default:
      Serial.print("ERROR:UNKNOWN_CMD:");
      Serial.println(cmd, HEX);
      break;
  }
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  
  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // LED off (active low)
  
  // Blink LED to indicate startup
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, LOW);
    delay(100);
    digitalWrite(LED_PIN, HIGH);
    delay(100);
  }
  
  // Initialize camera
  if (initCamera()) {
    Serial.println("CAM:READY");
    // Success blink
    digitalWrite(LED_PIN, LOW);
    delay(500);
    digitalWrite(LED_PIN, HIGH);
  } else {
    Serial.println("CAM:ERROR");
    // Error blink continuously
    while (1) {
      digitalWrite(LED_PIN, LOW);
      delay(200);
      digitalWrite(LED_PIN, HIGH);
      delay(200);
    }
  }
  
  // Clear previous frame buffer
  memset(prevFrameBuf, 0, sizeof(prevFrameBuf));
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  // Check for commands from ESP32-S3
  if (Serial.available()) {
    uint8_t cmd = Serial.read();
    handleCommand(cmd);
  }
  
  // Stream frames if enabled
  if (streamingEnabled) {
    streamFrame();
    delay(50);  // ~20 FPS streaming
  }
  
  // Small delay to prevent tight loop
  delay(10);
}


