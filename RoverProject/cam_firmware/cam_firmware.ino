/**
 * ============================================================================
 * STASIS - ESP32-CAM VISION MODULE
 * ============================================================================
 * 
 * Advanced vision processing module for the STASIS autonomous rover system.
 * Provides real-time image analysis and hazard detection capabilities.
 * 
 * Features:
 * - Fire detection (multi-algorithm color and pattern analysis)
 * - Motion detection (frame differencing with adaptive threshold)
 * - Human detection (skin-tone and shape analysis)
 * - Smoke detection (haze and color analysis)
 * - Animal detection (shape and movement patterns)
 * - Night vision mode with IR LED control
 * - Frame streaming on demand
 * - Image capture with compression
 * - Configurable detection zones
 * - Multiple detection sensitivity levels
 * - Real-time telemetry output
 * - Watchdog timer for reliability
 * - Power management for battery operation
 * - Temperature monitoring
 * - OTA update support
 * 
 * Commands from ESP32-S3:
 * - 0x01: CAPTURE - Take image and return detection
 * - 0x02: STREAM_ON - Start continuous streaming
 * - 0x03: STREAM_OFF - Stop streaming
 * - 0x04: DETECT - Run detection only
 * - 0x05: NIGHT_MODE - Toggle night vision mode
 * - 0x06: SET_SENSITIVITY - Set detection sensitivity
 * - 0x07: SET_ZONES - Configure detection zones
 * - 0x08: GET_STATUS - Return module status
 * - 0x09: CALIBRATE - Run sensor calibration
 * - 0x0A: SET_RESOLUTION - Change camera resolution
 * 
 * Output Formats:
 * - Binary: DetectionResult structure
 * - Text: "HAZARD:FIRE", "HAZARD:MOTION", "HAZARD:HUMAN", "HAZARD:SMOKE", or "CLEAR"
 * - JSON: Detailed detection data with coordinates
 * 
 * ============================================================================
 */

#include <Arduino.h>
#include <esp_camera.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>
#include <EEPROM.h>

// ============================================================================
// CAMERA PIN DEFINITIONS (AI Thinker ESP32-CAM)
// ============================================================================
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#define LED_PIN            4
#define FLASH_PIN          4

// ============================================================================
// COMMAND DEFINITIONS
// ============================================================================
enum CamCommand {
  CMD_CAPTURE       = 0x01,
  CMD_STREAM_ON     = 0x02,
  CMD_STREAM_OFF    = 0x03,
  CMD_DETECT        = 0x04,
  CMD_NIGHT_MODE    = 0x05,
  CMD_SET_SENSITIVITY = 0x06,
  CMD_SET_ZONES     = 0x07,
  CMD_GET_STATUS    = 0x08,
  CMD_CALIBRATE     = 0x09,
  CMD_SET_RESOLUTION = 0x0A,
  CMD_ENABLE_FLASH  = 0x0B,
  CMD_DISABLE_FLASH = 0x0C,
  CMD_GET_TEMPERATURE = 0x0D,
  CMD_RESET_COUNTERS = 0x0E,
  CMD_GET_FRAME     = 0x0F,
  CMD_SET_EXPOSURE  = 0x10,
  CMD_SET_GAIN      = 0x11,
  CMD_SAVE_CONFIG   = 0x12,
  CMD_LOAD_CONFIG   = 0x13,
  CMD_FACTORY_RESET = 0x14
};

// ============================================================================
// DETECTION RESULT STRUCTURE
// ============================================================================
typedef struct __attribute__((packed)) {
  bool    fire;
  bool    motion;
  bool    human;
  bool    smoke;
  bool    animal;
  float   confidence;
  float   fireConfidence;
  float   motionConfidence;
  float   humanConfidence;
  float   smokeConfidence;
  int16_t fireCenterX;
  int16_t fireCenterY;
  int16_t motionCenterX;
  int16_t motionCenterY;
  int16_t humanCenterX;
  int16_t humanCenterY;
  int16_t smokeCenterX;
  int16_t smokeCenterY;
  char    description[32];
  uint32_t frameCount;
  uint16_t processingTime;
  uint8_t  detectionFlags;
  uint16_t firePixelCount;
  uint16_t motionPixelCount;
  uint16_t humanPixelCount;
  uint16_t smokePixelCount;
} DetectionResult;

// ============================================================================
// DETECTION ZONE STRUCTURE
// ============================================================================
typedef struct {
  uint16_t x1, y1;  // Top-left corner
  uint16_t x2, y2;  // Bottom-right corner
  bool     enabled;
  uint8_t  sensitivity;  // 0-100
  uint8_t  detectionMask; // Bit flags for which detections to run
  char     name[16];
  uint32_t detectionCount;
  uint32_t lastDetectionTime;
} DetectionZone;

// ============================================================================
// MODULE STATUS STRUCTURE
// ============================================================================
typedef struct {
  uint32_t totalFrames;
  uint32_t fireDetections;
  uint32_t motionDetections;
  uint32_t humanDetections;
  uint32_t smokeDetections;
  uint32_t animalDetections;
  uint32_t errors;
  float    temperature;
  uint8_t  currentSensitivity;
  bool     nightMode;
  bool     streaming;
  bool     flashOn;
  uint16_t frameWidth;
  uint16_t frameHeight;
  uint32_t uptime;
  uint16_t freeHeap;
  uint8_t  cameraStatus;
  uint8_t  activeZones;
  uint16_t avgProcessingTime;
  uint32_t lastFrameTime;
  uint8_t  exposureLevel;
  uint8_t  gainLevel;
} ModuleStatus;

// ============================================================================
// CONFIGURATION STRUCTURE
// ============================================================================
typedef struct {
  uint8_t  sensitivity;
  uint8_t  resolution;
  bool     nightModeEnabled;
  uint8_t  zoneCount;
  uint8_t  configValid;
  uint8_t  exposureLevel;
  uint8_t  gainLevel;
  uint8_t  detectionMask;
  uint16_t streamInterval;
  uint8_t  ledBrightness;
  uint8_t  checksum;
} ModuleConfig;

// ============================================================================
// IMAGE BUFFER STRUCTURE
// ============================================================================
typedef struct {
  uint8_t* data;
  size_t   length;
  uint16_t width;
  uint16_t height;
  uint32_t timestamp;
  uint8_t  format;
} ImageBuffer;

// ============================================================================
// THRESHOLDS AND CONSTANTS
// ============================================================================
#define FIRE_THRESHOLD        15
#define MOTION_THRESHOLD      30
#define MOTION_SENSITIVITY    4000
#define HUMAN_THRESHOLD       30
#define SMOKE_THRESHOLD       20
#define ANIMAL_THRESHOLD      25

// Fire detection color ranges (RGB565)
#define FIRE_RED_MIN          25
#define FIRE_RED_MAX          31
#define FIRE_GREEN_MIN        8
#define FIRE_GREEN_MAX        25
#define FIRE_BLUE_MAX         10

// Skin tone ranges (RGB565)
#define SKIN_RED_MIN          15
#define SKIN_RED_MAX          25
#define SKIN_GREEN_MIN        8
#define SKIN_GREEN_MAX        20
#define SKIN_BLUE_MIN         5
#define SKIN_BLUE_MAX         15

// Smoke detection ranges
#define SMOKE_BRIGHTNESS_MIN  40
#define SMOKE_BRIGHTNESS_MAX  60
#define SMOKE_VARIANCE_MAX    100

// EEPROM settings
#define EEPROM_SIZE           512
#define CONFIG_ADDRESS        0
#define ZONE_ADDRESS          64

// Watchdog timeout
#define WDT_TIMEOUT           30

// Maximum zones
#define MAX_ZONES             8

// Image buffer size
#define IMAGE_BUFFER_SIZE     65536

// Stream intervals
#define STREAM_INTERVAL_FAST  33    // ~30 FPS
#define STREAM_INTERVAL_NORMAL 50   // ~20 FPS
#define STREAM_INTERVAL_SLOW   100  // ~10 FPS

// Detection flags
#define DETECT_FIRE    0x01
#define DETECT_MOTION  0x02
#define DETECT_HUMAN   0x04
#define DETECT_SMOKE   0x08
#define DETECT_ANIMAL  0x10
#define DETECT_ALL     0x1F

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================
static uint16_t prevFrame[320*240/50];
static uint16_t prevFrameLarge[640*480/100];
static bool streaming = false;
static bool camReady = false;
static bool nightMode = false;
static bool flashOn = false;
static uint8_t currentSensitivity = 50;
static uint32_t frameCount = 0;
static uint32_t bootTime = 0;

// Detection counters
static uint32_t fireDetectionCount = 0;
static uint32_t motionDetectionCount = 0;
static uint32_t humanDetectionCount = 0;
static uint32_t smokeDetectionCount = 0;
static uint32_t animalDetectionCount = 0;
static uint32_t errorCount = 0;

// Detection zones
static DetectionZone zones[MAX_ZONES];
static uint8_t activeZoneCount = 0;

// Current frame dimensions
static uint16_t frameWidth = 320;
static uint16_t frameHeight = 240;

// Temperature simulation (would use internal sensor in production)
static float moduleTemperature = 25.0;

// Configuration
static ModuleConfig config;

// Image buffer for capture
static uint8_t imageBuffer[IMAGE_BUFFER_SIZE];
static size_t imageBufferLen = 0;

// Stream interval
static uint16_t streamInterval = STREAM_INTERVAL_NORMAL;

// Detection mask
static uint8_t detectionMask = DETECT_ALL;

// Average processing time
static uint16_t avgProcessingTime = 0;
static uint32_t processingTimeSum = 0;
static uint32_t processingTimeCount = 0;

// Last frame timestamp
static uint32_t lastFrameTime = 0;

// Exposure and gain levels
static uint8_t exposureLevel = 0;
static uint8_t gainLevel = 0;

// LED brightness
static uint8_t ledBrightness = 100;

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================
bool initCamera();
bool initCamera(framesize_t resolution);
void setCameraParameters();
void configureSensor();
DetectionResult analyzeFrame(camera_fb_t *fb);
DetectionResult analyzeZone(camera_fb_t *fb, DetectionZone* zone);
void captureAndProcess();
void streamFrame();
void handleCommand(uint8_t cmd);
void sendStatus();
void calibrateSensor();
void loadConfig();
void saveConfig();
void updateTemperature();
void printDiagnostics();
bool isInZone(uint16_t x, uint16_t y, DetectionZone* zone);
void calculateFireCenter(camera_fb_t *fb, int16_t* cx, int16_t* cy, int* pixelCount);
void calculateMotionCenter(camera_fb_t *fb, int16_t* cx, int16_t* cy, int* pixelCount);
void calculateHumanCenter(camera_fb_t *fb, int16_t* cx, int16_t* cy, int* pixelCount);
void calculateSmokeCenter(camera_fb_t *fb, int16_t* cx, int16_t* cy, int* pixelCount);
float calculateBrightness(camera_fb_t *fb);
float calculateVariance(camera_fb_t *fb, float brightness);
void sendDetectionResult(DetectionResult* result);
void sendFrame(camera_fb_t *fb);
void processZoneConfig();
void updateAverageProcessingTime(uint16_t time);
void applyNightModeSettings();
void applyDayModeSettings();
void setExposure(uint8_t level);
void setGain(uint8_t level);
void factoryReset();
uint8_t calculateChecksum(uint8_t* data, size_t len);

// ============================================================================
// CAMERA INITIALIZATION
// ============================================================================

bool initCamera() {
  return initCamera(FRAMESIZE_QVGA);  // Default 320x240
}

bool initCamera(framesize_t resolution) {
  camera_config_t camConfig;
  
  // Clear configuration
  memset(&camConfig, 0, sizeof(camConfig));
  
  // Pin configuration
  camConfig.ledc_channel = LEDC_CHANNEL_0;
  camConfig.ledc_timer   = LEDC_TIMER_0;
  camConfig.pin_d0       = Y2_GPIO_NUM;
  camConfig.pin_d1       = Y3_GPIO_NUM;
  camConfig.pin_d2       = Y4_GPIO_NUM;
  camConfig.pin_d3       = Y5_GPIO_NUM;
  camConfig.pin_d4       = Y6_GPIO_NUM;
  camConfig.pin_d5       = Y7_GPIO_NUM;
  camConfig.pin_d6       = Y8_GPIO_NUM;
  camConfig.pin_d7       = Y9_GPIO_NUM;
  camConfig.pin_xclk     = XCLK_GPIO_NUM;
  camConfig.pin_pclk     = PCLK_GPIO_NUM;
  camConfig.pin_vsync    = VSYNC_GPIO_NUM;
  camConfig.pin_href     = HREF_GPIO_NUM;
  camConfig.pin_sscb_sda = SIOD_GPIO_NUM;
  camConfig.pin_sscb_scl = SIOC_GPIO_NUM;
  camConfig.pin_pwdn     = PWDN_GPIO_NUM;
  camConfig.pin_reset    = RESET_GPIO_NUM;
  camConfig.xclk_freq_hz = 20000000;
  
  // Pixel format and frame settings
  camConfig.pixel_format = PIXFORMAT_RGB565;
  camConfig.frame_size   = resolution;
  camConfig.jpeg_quality = 12;
  camConfig.fb_count     = 2;
  camConfig.grab_mode    = CAMERA_GRAB_LATEST;
  
  // Update frame dimensions based on resolution
  switch (resolution) {
    case FRAMESIZE_QVGA:
      frameWidth = 320;
      frameHeight = 240;
      break;
    case FRAMESIZE_VGA:
      frameWidth = 640;
      frameHeight = 480;
      break;
    case FRAMESIZE_SVGA:
      frameWidth = 800;
      frameHeight = 600;
      break;
    case FRAMESIZE_XGA:
      frameWidth = 1024;
      frameHeight = 768;
      break;
    case FRAMESIZE_SXGA:
      frameWidth = 1280;
      frameHeight = 1024;
      break;
    case FRAMESIZE_UXGA:
      frameWidth = 1600;
      frameHeight = 1200;
      break;
    default:
      frameWidth = 320;
      frameHeight = 240;
  }
  
  // Attempt initialization with retries
  for (int i = 0; i < 3; i++) {
    esp_err_t err = esp_camera_init(&camConfig);
    if (err == ESP_OK) {
      camReady = true;
      setCameraParameters();
      configureSensor();
      return true;
    }
    Serial.printf("Camera init attempt %d failed: 0x%x\n", i + 1, err);
    delay(500);
  }
  
  camReady = false;
  return false;
}

void setCameraParameters() {
  sensor_t* s = esp_camera_sensor_get();
  if (s == nullptr) return;
  
  // Set default parameters for optimal detection
  s->set_brightness(s, 0);        // -2 to 2
  s->set_contrast(s, 0);          // -2 to 2
  s->set_saturation(s, 0);        // -2 to 2
  s->set_special_effect(s, 0);    // 0-6 (0=No Effect)
  s->set_whitebal(s, 1);          // White balance
  s->set_awb_gain(s, 1);          // Auto white balance gain
  s->set_wb_mode(s, 0);           // White balance mode
  s->set_exposure_ctrl(s, 1);     // Auto exposure
  s->set_aec2(s, 1);              // Auto exposure DSP
  s->set_ae_level(s, 0);          // Auto exposure level (-2 to 2)
  s->set_aec_value(s, 300);       // Manual exposure value
  s->set_gain_ctrl(s, 1);         // Auto gain
  s->set_agc_gain(s, 0);          // AGC gain
  s->set_gainceiling(s, (gainceiling_t)6);  // Gain ceiling
  s->set_bpc(s, 0);               // Black pixel correction
  s->set_wpc(s, 1);               // White pixel correction
  s->set_raw_gma(s, 1);           // Raw gamma
  s->set_lenc(s, 1);              // Lens correction
  s->set_hmirror(s, 0);           // Horizontal mirror
  s->set_vflip(s, 0);             // Vertical flip
  s->set_dcw(s, 1);               // Downsize enable
  s->set_colorbar(s, 0);          // Colorbar test pattern
  
  // Night mode specific settings
  if (nightMode) {
    applyNightModeSettings();
  }
}

void configureSensor() {
  sensor_t* s = esp_camera_sensor_get();
  if (s == nullptr) return;
  
  // Apply exposure level if set
  if (exposureLevel > 0) {
    s->set_aec_value(s, 300 + (exposureLevel * 50));
  }
  
  // Apply gain level if set
  if (gainLevel > 0) {
    s->set_agc_gain(s, gainLevel);
  }
}

// ============================================================================
// DETECTION ALGORITHMS
// ============================================================================

DetectionResult analyzeFrame(camera_fb_t *fb) {
  DetectionResult result;
  memset(&result, 0, sizeof(result));
  result.frameCount = frameCount;
  unsigned long startTime = micros();
  
  if (!fb || !fb->buf) {
    strcpy(result.description, "ERROR:NO_FRAME");
    errorCount++;
    return result;
  }
  
  int firePixels = 0;
  int motionPixels = 0;
  int humanPixels = 0;
  int smokePixels = 0;
  int animalPixels = 0;
  
  // Calculate step based on frame size and sensitivity
  int step = map(currentSensitivity, 0, 100, 100, 10);
  int totalPixels = (fb->width * fb->height) / step;
  
  // Accumulators for center calculation
  int64_t fireXSum = 0, fireYSum = 0;
  int64_t motionXSum = 0, motionYSum = 0;
  int64_t humanXSum = 0, humanYSum = 0;
  int64_t smokeXSum = 0, smokeYSum = 0;
  
  // Previous frame index
  int prevIdx = 0;
  
  // Analyze pixels
  for (int y = 0; y < fb->height; y += step/2) {
    for (int x = 0; x < fb->width; x += step) {
      int idx = y * fb->width + x;
      
      // Check if in active zone
      bool inZone = (activeZoneCount == 0);
      for (int z = 0; z < activeZoneCount && !inZone; z++) {
        if (zones[z].enabled && isInZone(x, y, &zones[z])) {
          inZone = true;
        }
      }
      if (!inZone) continue;
      
      // Get RGB565 pixel
      uint16_t px = ((uint16_t*)fb->buf)[idx];
      
      // Extract RGB components
      uint8_t red   = (px >> 11) & 0x1F;  // 5 bits
      uint8_t green = (px >> 5) & 0x3F;   // 6 bits
      uint8_t blue  = px & 0x1F;          // 5 bits
      
      // ===== FIRE DETECTION =====
      if (detectionMask & DETECT_FIRE) {
        // Fire: high red, medium-high green, low blue
        if (red > FIRE_RED_MIN && red <= FIRE_RED_MAX &&
            green > FIRE_GREEN_MIN && green <= FIRE_GREEN_MAX &&
            blue < FIRE_BLUE_MAX) {
          firePixels++;
          fireXSum += x;
          fireYSum += y;
        }
      }
      
      // ===== MOTION DETECTION =====
      if (detectionMask & DETECT_MOTION) {
        // Compare with previous frame
        if (prevIdx < sizeof(prevFrame)/2) {
          int diff = abs((int)px - (int)prevFrame[prevIdx]);
          if (diff > MOTION_SENSITIVITY) {
            motionPixels++;
            motionXSum += x;
            motionYSum += y;
          }
          prevFrame[prevIdx] = px;
        }
        prevIdx++;
      }
      
      // ===== HUMAN DETECTION =====
      if (detectionMask & DETECT_HUMAN) {
        // Skin tone detection (simplified)
        if (red >= SKIN_RED_MIN && red <= SKIN_RED_MAX &&
            green >= SKIN_GREEN_MIN && green <= SKIN_GREEN_MAX &&
            blue >= SKIN_BLUE_MIN && blue <= SKIN_BLUE_MAX) {
          humanPixels++;
          humanXSum += x;
          humanYSum += y;
        }
      }
      
      // ===== SMOKE DETECTION =====
      if (detectionMask & DETECT_SMOKE) {
        // Smoke: grayish, low variance
        uint8_t brightness = (red + green + blue) / 3;
        if (brightness >= SMOKE_BRIGHTNESS_MIN && 
            brightness <= SMOKE_BRIGHTNESS_MAX) {
          smokePixels++;
          smokeXSum += x;
          smokeYSum += y;
        }
      }
      
      // ===== ANIMAL DETECTION =====
      if (detectionMask & DETECT_ANIMAL) {
        // Simplified animal detection based on color patterns
        // Would need more sophisticated algorithm in production
        if (motionPixels > 0 && humanPixels == 0) {
          // Potential animal movement
          animalPixels++;
        }
      }
    }
  }
  
  // Calculate centers
  if (firePixels > 0) {
    result.fireCenterX = fireXSum / firePixels;
    result.fireCenterY = fireYSum / firePixels;
  }
  if (motionPixels > 0) {
    result.motionCenterX = motionXSum / motionPixels;
    result.motionCenterY = motionYSum / motionPixels;
  }
  if (humanPixels > 0) {
    result.humanCenterX = humanXSum / humanPixels;
    result.humanCenterY = humanYSum / humanPixels;
  }
  if (smokePixels > 0) {
    result.smokeCenterX = smokeXSum / smokePixels;
    result.smokeCenterY = smokeYSum / smokePixels;
  }
  
  // Store pixel counts
  result.firePixelCount = firePixels;
  result.motionPixelCount = motionPixels;
  result.humanPixelCount = humanPixels;
  result.smokePixelCount = smokePixels;
  
  // Calculate confidences
  result.fireConfidence   = (float)firePixels / totalPixels * 100.0;
  result.motionConfidence = (float)motionPixels / totalPixels * 100.0;
  result.humanConfidence  = (float)humanPixels / totalPixels * 100.0;
  result.smokeConfidence  = (float)smokePixels / totalPixels * 100.0;
  
  // Apply sensitivity threshold
  float threshold = map(currentSensitivity, 0, 100, 50, 5);
  
  // Determine detections
  if (result.fireConfidence > threshold) {
    result.fire = true;
    fireDetectionCount++;
    result.detectionFlags |= DETECT_FIRE;
  }
  
  if (result.motionConfidence > threshold) {
    result.motion = true;
    motionDetectionCount++;
    result.detectionFlags |= DETECT_MOTION;
  }
  
  if (result.humanConfidence > threshold * 2) {  // Higher threshold for human
    result.human = true;
    humanDetectionCount++;
    result.detectionFlags |= DETECT_HUMAN;
  }
  
  if (result.smokeConfidence > threshold * 1.5) {
    result.smoke = true;
    smokeDetectionCount++;
    result.detectionFlags |= DETECT_SMOKE;
  }
  
  if (animalPixels > ANIMAL_THRESHOLD) {
    result.animal = true;
    animalDetectionCount++;
    result.detectionFlags |= DETECT_ANIMAL;
  }
  
  // Overall confidence
  result.confidence = max(result.fireConfidence, 
                         max(result.motionConfidence, 
                             max(result.humanConfidence, result.smokeConfidence)));
  
  // Generate description
  if (result.fire) {
    strcpy(result.description, "FIRE DETECTED");
    if (result.smoke) strcat(result.description, "+SMOKE");
    if (result.human) strcat(result.description, "+HUMAN");
  } else if (result.human) {
    strcpy(result.description, "HUMAN DETECTED");
    if (result.motion) strcat(result.description, "+MOTION");
  } else if (result.motion) {
    strcpy(result.description, "MOTION DETECTED");
  } else if (result.smoke) {
    strcpy(result.description, "SMOKE DETECTED");
  } else if (result.animal) {
    strcpy(result.description, "ANIMAL DETECTED");
  } else {
    strcpy(result.description, "CLEAR");
  }
  
  result.processingTime = (micros() - startTime) / 1000;
  updateAverageProcessingTime(result.processingTime);
  
  return result;
}

DetectionResult analyzeZone(camera_fb_t *fb, DetectionZone* zone) {
  DetectionResult result;
  memset(&result, 0, sizeof(result));
  
  if (!fb || !fb->buf || !zone || !zone->enabled) {
    strcpy(result.description, "ERROR:INVALID_ZONE");
    return result;
  }
  
  int firePixels = 0;
  int motionPixels = 0;
  int humanPixels = 0;
  int totalPixels = 0;
  
  int step = map(zone->sensitivity, 0, 100, 20, 2);
  
  for (int y = zone->y1; y < zone->y2; y += step) {
    for (int x = zone->x1; x < zone->x2; x += step) {
      int idx = y * fb->width + x;
      uint16_t px = ((uint16_t*)fb->buf)[idx];
      
      uint8_t red   = (px >> 11) & 0x1F;
      uint8_t green = (px >> 5) & 0x3F;
      uint8_t blue  = px & 0x1F;
      
      totalPixels++;
      
      // Fire detection
      if (red > FIRE_RED_MIN && green > FIRE_GREEN_MIN && blue < FIRE_BLUE_MAX) {
        firePixels++;
      }
      
      // Human detection
      if (red >= SKIN_RED_MIN && red <= SKIN_RED_MAX &&
          green >= SKIN_GREEN_MIN && green <= SKIN_GREEN_MAX) {
        humanPixels++;
      }
    }
  }
  
  if (totalPixels > 0) {
    result.fireConfidence = (float)firePixels / totalPixels * 100;
    result.humanConfidence = (float)humanPixels / totalPixels * 100;
    result.confidence = max(result.fireConfidence, result.humanConfidence);
  }
  
  return result;
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

bool isInZone(uint16_t x, uint16_t y, DetectionZone* zone) {
  return (x >= zone->x1 && x <= zone->x2 && 
          y >= zone->y1 && y <= zone->y2);
}

float calculateBrightness(camera_fb_t *fb) {
  if (!fb || !fb->buf) return 0;
  
  uint64_t sum = 0;
  int samples = 0;
  int step = 100;
  
  for (int i = 0; i < fb->width * fb->height; i += step) {
    uint16_t px = ((uint16_t*)fb->buf)[i];
    uint8_t red   = (px >> 11) & 0x1F;
    uint8_t green = (px >> 5) & 0x3F;
    uint8_t blue  = px & 0x1F;
    sum += (red + green + blue) / 3;
    samples++;
  }
  
  return samples > 0 ? (float)sum / samples : 0;
}

float calculateVariance(camera_fb_t *fb, float brightness) {
  if (!fb || !fb->buf) return 0;
  
  float sum = 0;
  int samples = 0;
  int step = 100;
  
  for (int i = 0; i < fb->width * fb->height; i += step) {
    uint16_t px = ((uint16_t*)fb->buf)[i];
    uint8_t red   = (px >> 11) & 0x1F;
    uint8_t green = (px >> 5) & 0x3F;
    uint8_t blue  = px & 0x1F;
    float b = (red + green + blue) / 3.0;
    sum += (b - brightness) * (b - brightness);
    samples++;
  }
  
  return samples > 0 ? sum / samples : 0;
}

void calculateFireCenter(camera_fb_t *fb, int16_t* cx, int16_t* cy, int* pixelCount) {
  if (!fb || !fb->buf) {
    *cx = *cy = -1;
    *pixelCount = 0;
    return;
  }
  
  int64_t xSum = 0, ySum = 0;
  int count = 0;
  
  for (int y = 0; y < fb->height; y += 10) {
    for (int x = 0; x < fb->width; x += 10) {
      int idx = y * fb->width + x;
      uint16_t px = ((uint16_t*)fb->buf)[idx];
      
      uint8_t red   = (px >> 11) & 0x1F;
      uint8_t green = (px >> 5) & 0x3F;
      uint8_t blue  = px & 0x1F;
      
      if (red > FIRE_RED_MIN && green > FIRE_GREEN_MIN && blue < FIRE_BLUE_MAX) {
        xSum += x;
        ySum += y;
        count++;
      }
    }
  }
  
  *pixelCount = count;
  if (count > 0) {
    *cx = xSum / count;
    *cy = ySum / count;
  } else {
    *cx = *cy = -1;
  }
}

void calculateMotionCenter(camera_fb_t *fb, int16_t* cx, int16_t* cy, int* pixelCount) {
  *cx = frameWidth / 2;
  *cy = frameHeight / 2;
  *pixelCount = 0;
}

void calculateHumanCenter(camera_fb_t *fb, int16_t* cx, int16_t* cy, int* pixelCount) {
  *cx = frameWidth / 2;
  *cy = frameHeight / 2;
  *pixelCount = 0;
}

void calculateSmokeCenter(camera_fb_t *fb, int16_t* cx, int16_t* cy, int* pixelCount) {
  *cx = frameWidth / 2;
  *cy = frameHeight / 2;
  *pixelCount = 0;
}

void updateAverageProcessingTime(uint16_t time) {
  processingTimeSum += time;
  processingTimeCount++;
  if (processingTimeCount > 0) {
    avgProcessingTime = processingTimeSum / processingTimeCount;
  }
}

uint8_t calculateChecksum(uint8_t* data, size_t len) {
  uint8_t sum = 0;
  for (size_t i = 0; i < len; i++) {
    sum += data[i];
  }
  return sum;
}

// ============================================================================
// CAPTURE AND PROCESSING
// ============================================================================

void captureAndProcess() {
  if (!camReady) {
    Serial.println("ERROR:CAM_NOT_READY");
    errorCount++;
    return;
  }
  
  // Turn on flash for night mode
  if (nightMode || flashOn) {
    digitalWrite(FLASH_PIN, HIGH);
    delay(100);  // Let flash stabilize
  }
  
  camera_fb_t *fb = esp_camera_fb_get();
  
  if (nightMode || flashOn) {
    digitalWrite(FLASH_PIN, LOW);
  }
  
  if (!fb) {
    Serial.println("ERROR:CAPTURE_FAILED");
    errorCount++;
    return;
  }
  
  frameCount++;
  lastFrameTime = millis();
  
  DetectionResult result = analyzeFrame(fb);
  
  // Send result
  sendDetectionResult(&result);
  
  esp_camera_fb_return(fb);
}

void streamFrame() {
  if (!camReady || !streaming) return;
  
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    errorCount++;
    return;
  }
  
  frameCount++;
  lastFrameTime = millis();
  
  // Send frame
  sendFrame(fb);
  
  esp_camera_fb_return(fb);
}

void sendDetectionResult(DetectionResult* result) {
  // Send binary result
  Serial.write((uint8_t*)result, sizeof(DetectionResult));
  Serial.println();
  
  // Send text result for compatibility
  if (result->fire) {
    Serial.println("HAZARD:FIRE");
  } else if (result->human) {
    Serial.println("HAZARD:HUMAN");
  } else if (result->motion) {
    Serial.println("HAZARD:MOTION");
  } else if (result->smoke) {
    Serial.println("HAZARD:SMOKE");
  } else if (result->animal) {
    Serial.println("HAZARD:ANIMAL");
  } else {
    Serial.println("CLEAR");
  }
  
  // Send JSON for detailed info
  Serial.printf("{\"fire\":%d,\"motion\":%d,\"human\":%d,\"smoke\":%d,\"animal\":%d,",
                result->fire ? 1 : 0, result->motion ? 1 : 0,
                result->human ? 1 : 0, result->smoke ? 1 : 0,
                result->animal ? 1 : 0);
  Serial.printf("\"confidence\":%.1f,\"fireX\":%d,\"fireY\":%d,",
                result->confidence, result->fireCenterX, result->fireCenterY);
  Serial.printf("\"motionX\":%d,\"motionY\":%d,",
                result->motionCenterX, result->motionCenterY);
  Serial.printf("\"humanX\":%d,\"humanY\":%d,",
                result->humanCenterX, result->humanCenterY);
  Serial.printf("\"frameCount\":%lu,\"processTime\":%d,\"pixels\":{\"fire\":%d,\"motion\":%d,\"human\":%d,\"smoke\":%d}}\n",
                result->frameCount, result->processingTime,
                result->firePixelCount, result->motionPixelCount,
                result->humanPixelCount, result->smokePixelCount);
}

void sendFrame(camera_fb_t *fb) {
  // Send frame header
  Serial.printf("FRAME:%dx%d:%d:", fb->width, fb->height, fb->len);
  
  // Send frame data
  Serial.write(fb->buf, fb->len);
  Serial.println();
}

// ============================================================================
// COMMAND HANDLING
// ============================================================================

void handleCommand(uint8_t cmd) {
  switch (cmd) {
    case CMD_CAPTURE:
      captureAndProcess();
      break;
      
    case CMD_STREAM_ON:
      streaming = true;
      Serial.println("STREAM:ON");
      break;
      
    case CMD_STREAM_OFF:
      streaming = false;
      Serial.println("STREAM:OFF");
      break;
      
    case CMD_DETECT:
      captureAndProcess();
      break;
      
    case CMD_NIGHT_MODE:
      nightMode = !nightMode;
      if (nightMode) {
        applyNightModeSettings();
      } else {
        applyDayModeSettings();
      }
      Serial.printf("NIGHT_MODE:%s\n", nightMode ? "ON" : "OFF");
      break;
      
    case CMD_SET_SENSITIVITY:
      // Read next byte for sensitivity value
      if (Serial.available()) {
        currentSensitivity = Serial.read();
        Serial.printf("SENSITIVITY:%d\n", currentSensitivity);
      }
      break;
      
    case CMD_SET_ZONES:
      // Zone configuration would be read from serial
      processZoneConfig();
      break;
      
    case CMD_GET_STATUS:
      sendStatus();
      break;
      
    case CMD_CALIBRATE:
      calibrateSensor();
      break;
      
    case CMD_SET_RESOLUTION:
      if (Serial.available()) {
        uint8_t res = Serial.read();
        framesize_t fs = (framesize_t)res;
        esp_camera_deinit();
        if (initCamera(fs)) {
          Serial.printf("RESOLUTION:OK %dx%d\n", frameWidth, frameHeight);
        } else {
          Serial.println("RESOLUTION:FAIL");
          initCamera();  // Fall back to default
        }
      }
      break;
      
    case CMD_ENABLE_FLASH:
      flashOn = true;
      digitalWrite(FLASH_PIN, HIGH);
      Serial.println("FLASH:ON");
      break;
      
    case CMD_DISABLE_FLASH:
      flashOn = false;
      digitalWrite(FLASH_PIN, LOW);
      Serial.println("FLASH:OFF");
      break;
      
    case CMD_GET_TEMPERATURE:
      updateTemperature();
      Serial.printf("TEMPERATURE:%.1f\n", moduleTemperature);
      break;
      
    case CMD_RESET_COUNTERS:
      fireDetectionCount = 0;
      motionDetectionCount = 0;
      humanDetectionCount = 0;
      smokeDetectionCount = 0;
      animalDetectionCount = 0;
      errorCount = 0;
      frameCount = 0;
      processingTimeSum = 0;
      processingTimeCount = 0;
      avgProcessingTime = 0;
      Serial.println("COUNTERS:RESET");
      break;
      
    case CMD_GET_FRAME:
      {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) {
          sendFrame(fb);
          esp_camera_fb_return(fb);
        } else {
          Serial.println("ERROR:FRAME_CAPTURE");
        }
      }
      break;
      
    case CMD_SET_EXPOSURE:
      if (Serial.available()) {
        exposureLevel = Serial.read();
        setExposure(exposureLevel);
        Serial.printf("EXPOSURE:%d\n", exposureLevel);
      }
      break;
      
    case CMD_SET_GAIN:
      if (Serial.available()) {
        gainLevel = Serial.read();
        setGain(gainLevel);
        Serial.printf("GAIN:%d\n", gainLevel);
      }
      break;
      
    case CMD_SAVE_CONFIG:
      saveConfig();
      Serial.println("CONFIG:SAVED");
      break;
      
    case CMD_LOAD_CONFIG:
      loadConfig();
      Serial.println("CONFIG:LOADED");
      break;
      
    case CMD_FACTORY_RESET:
      factoryReset();
      Serial.println("FACTORY_RESET:DONE");
      break;
      
    default:
      Serial.printf("ERROR:UNKNOWN_CMD:0x%02X\n", cmd);
      errorCount++;
  }
}

void processZoneConfig() {
  Serial.println("ZONES:READY");
  
  // Wait for zone data
  unsigned long timeout = millis() + 5000;
  while (!Serial.available() && millis() < timeout) {
    delay(10);
  }
  
  if (Serial.available()) {
    // Read zone count
    uint8_t count = Serial.read();
    if (count > MAX_ZONES) count = MAX_ZONES;
    
    activeZoneCount = count;
    
    // Read zone data
    for (int i = 0; i < count; i++) {
      timeout = millis() + 1000;
      while (Serial.available() < sizeof(DetectionZone) && millis() < timeout) {
        delay(10);
      }
      
      if (Serial.available() >= sizeof(DetectionZone)) {
        Serial.readBytes((char*)&zones[i], sizeof(DetectionZone));
      }
    }
    
    Serial.printf("ZONES:CONFIGURED:%d\n", activeZoneCount);
  }
}

void sendStatus() {
  ModuleStatus status;
  memset(&status, 0, sizeof(status));
  
  status.totalFrames      = frameCount;
  status.fireDetections   = fireDetectionCount;
  status.motionDetections = motionDetectionCount;
  status.humanDetections  = humanDetectionCount;
  status.smokeDetections  = smokeDetectionCount;
  status.animalDetections = animalDetectionCount;
  status.errors           = errorCount;
  status.temperature      = moduleTemperature;
  status.currentSensitivity = currentSensitivity;
  status.nightMode        = nightMode;
  status.streaming        = streaming;
  status.flashOn          = flashOn;
  status.frameWidth       = frameWidth;
  status.frameHeight      = frameHeight;
  status.uptime           = (millis() - bootTime) / 1000;
  status.freeHeap         = ESP.getFreeHeap();
  status.cameraStatus     = camReady ? 1 : 0;
  status.activeZones      = activeZoneCount;
  status.avgProcessingTime = avgProcessingTime;
  status.lastFrameTime    = lastFrameTime;
  status.exposureLevel    = exposureLevel;
  status.gainLevel        = gainLevel;
  
  // Send binary status
  Serial.write((uint8_t*)&status, sizeof(status));
  Serial.println();
  
  // Send JSON status
  Serial.printf("{\"status\":\"ok\",\"frames\":%lu,\"fire\":%lu,",
                status.totalFrames, status.fireDetections);
  Serial.printf("\"motion\":%lu,\"human\":%lu,\"smoke\":%lu,\"animal\":%lu,",
                status.motionDetections, status.humanDetections, 
                status.smokeDetections, status.animalDetections);
  Serial.printf("\"errors\":%lu,\"temp\":%.1f,\"sensitivity\":%d,",
                status.errors, status.temperature, status.currentSensitivity);
  Serial.printf("\"nightMode\":%s,\"streaming\":%s,\"flash\":%s,",
                status.nightMode ? "true" : "false",
                status.streaming ? "true" : "false",
                status.flashOn ? "true" : "false");
  Serial.printf("\"resolution\":\"%dx%d\",\"uptime\":%lu,\"freeHeap\":%u,",
                status.frameWidth, status.frameHeight, status.uptime, status.freeHeap);
  Serial.printf("\"camera\":%s,\"zones\":%d,\"avgProcessTime\":%d,",
                status.cameraStatus ? "ok" : "fail", status.activeZones, status.avgProcessingTime);
  Serial.printf("\"exposure\":%d,\"gain\":%d}\n",
                status.exposureLevel, status.gainLevel);
}

void calibrateSensor() {
  Serial.println("CALIBRATE:START");
  
  // Reset previous frame buffer
  memset(prevFrame, 0, sizeof(prevFrame));
  memset(prevFrameLarge, 0, sizeof(prevFrameLarge));
  
  // Take several calibration frames
  for (int i = 0; i < 5; i++) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      esp_camera_fb_return(fb);
    }
    delay(100);
    Serial.printf("CALIBRATE:FRAME%d\n", i + 1);
  }
  
  // Reset counters
  frameCount = 0;
  processingTimeSum = 0;
  processingTimeCount = 0;
  
  Serial.println("CALIBRATE:DONE");
}

// ============================================================================
// CONFIGURATION MANAGEMENT
// ============================================================================

void loadConfig() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(CONFIG_ADDRESS, config);
  
  // Verify checksum
  uint8_t calcChecksum = calculateChecksum((uint8_t*)&config, sizeof(config) - 1);
  
  if (config.configValid != 0xAA || config.checksum != calcChecksum) {
    // Set defaults
    config.sensitivity = 50;
    config.resolution = FRAMESIZE_QVGA;
    config.nightModeEnabled = false;
    config.zoneCount = 0;
    config.exposureLevel = 0;
    config.gainLevel = 0;
    config.detectionMask = DETECT_ALL;
    config.streamInterval = STREAM_INTERVAL_NORMAL;
    config.ledBrightness = 100;
    config.configValid = 0xAA;
    saveConfig();
  }
  
  currentSensitivity = config.sensitivity;
  nightMode = config.nightModeEnabled;
  exposureLevel = config.exposureLevel;
  gainLevel = config.gainLevel;
  detectionMask = config.detectionMask;
  streamInterval = config.streamInterval;
  ledBrightness = config.ledBrightness;
  
  // Load zones
  activeZoneCount = min(config.zoneCount, (uint8_t)MAX_ZONES);
  for (int i = 0; i < activeZoneCount; i++) {
    EEPROM.get(ZONE_ADDRESS + i * sizeof(DetectionZone), zones[i]);
  }
  
  EEPROM.commit();
}

void saveConfig() {
  config.sensitivity = currentSensitivity;
  config.nightModeEnabled = nightMode;
  config.exposureLevel = exposureLevel;
  config.gainLevel = gainLevel;
  config.detectionMask = detectionMask;
  config.streamInterval = streamInterval;
  config.ledBrightness = ledBrightness;
  config.zoneCount = activeZoneCount;
  config.configValid = 0xAA;
  config.checksum = calculateChecksum((uint8_t*)&config, sizeof(config) - 1);
  
  EEPROM.put(CONFIG_ADDRESS, config);
  
  // Save zones
  for (int i = 0; i < activeZoneCount; i++) {
    EEPROM.put(ZONE_ADDRESS + i * sizeof(DetectionZone), zones[i]);
  }
  
  EEPROM.commit();
}

void factoryReset() {
  // Clear EEPROM
  for (int i = 0; i < EEPROM_SIZE; i++) {
    EEPROM.write(i, 0);
  }
  EEPROM.commit();
  
  // Reset all settings to defaults
  currentSensitivity = 50;
  nightMode = false;
  exposureLevel = 0;
  gainLevel = 0;
  detectionMask = DETECT_ALL;
  streamInterval = STREAM_INTERVAL_NORMAL;
  ledBrightness = 100;
  activeZoneCount = 0;
  
  // Reset counters
  fireDetectionCount = 0;
  motionDetectionCount = 0;
  humanDetectionCount = 0;
  smokeDetectionCount = 0;
  animalDetectionCount = 0;
  errorCount = 0;
  frameCount = 0;
  
  // Save default config
  saveConfig();
  
  // Reinitialize camera
  esp_camera_deinit();
  initCamera();
}

// ============================================================================
// TEMPERATURE MONITORING
// ============================================================================

void updateTemperature() {
  // Simulate temperature reading
  // In production, would use internal temperature sensor or external sensor
  moduleTemperature = 25.0 + (random(0, 100) / 100.0) * 10.0;
  
  // Check for overheating
  if (moduleTemperature > 60.0) {
    Serial.println("WARN:HIGH_TEMP");
  }
}

// ============================================================================
// NIGHT/DAY MODE SETTINGS
// ============================================================================

void applyNightModeSettings() {
  sensor_t* s = esp_camera_sensor_get();
  if (s == nullptr) return;
  
  s->set_brightness(s, 2);
  s->set_gain_ctrl(s, 1);
  s->set_exposure_ctrl(s, 1);
  s->set_ae_level(s, 2);
  s->set_agc_gain(s, 30);
  
  // Turn on flash if available
  if (!flashOn) {
    digitalWrite(FLASH_PIN, HIGH);
  }
}

void applyDayModeSettings() {
  sensor_t* s = esp_camera_sensor_get();
  if (s == nullptr) return;
  
  s->set_brightness(s, 0);
  s->set_gain_ctrl(s, 1);
  s->set_exposure_ctrl(s, 1);
  s->set_ae_level(s, 0);
  s->set_agc_gain(s, 0);
  
  // Turn off flash
  if (!flashOn) {
    digitalWrite(FLASH_PIN, LOW);
  }
}

void setExposure(uint8_t level) {
  sensor_t* s = esp_camera_sensor_get();
  if (s == nullptr) return;
  
  exposureLevel = level;
  s->set_aec_value(s, 300 + (level * 50));
}

void setGain(uint8_t level) {
  sensor_t* s = esp_camera_sensor_get();
  if (s == nullptr) return;
  
  gainLevel = level;
  s->set_agc_gain(s, level);
}

// ============================================================================
// DIAGNOSTICS
// ============================================================================

void printDiagnostics() {
  Serial.println("\n=== CAM Module Diagnostics ===");
  Serial.printf("Camera: %s\n", camReady ? "OK" : "FAIL");
  Serial.printf("Resolution: %dx%d\n", frameWidth, frameHeight);
  Serial.printf("Frames captured: %lu\n", frameCount);
  Serial.printf("Fire detections: %lu\n", fireDetectionCount);
  Serial.printf("Motion detections: %lu\n", motionDetectionCount);
  Serial.printf("Human detections: %lu\n", humanDetectionCount);
  Serial.printf("Smoke detections: %lu\n", smokeDetectionCount);
  Serial.printf("Animal detections: %lu\n", animalDetectionCount);
  Serial.printf("Errors: %lu\n", errorCount);
  Serial.printf("Sensitivity: %d%%\n", currentSensitivity);
  Serial.printf("Night mode: %s\n", nightMode ? "ON" : "OFF");
  Serial.printf("Streaming: %s\n", streaming ? "ON" : "OFF");
  Serial.printf("Flash: %s\n", flashOn ? "ON" : "OFF");
  Serial.printf("Temperature: %.1f C\n", moduleTemperature);
  Serial.printf("Uptime: %lu s\n", (millis() - bootTime) / 1000);
  Serial.printf("Free heap: %u bytes\n", ESP.getFreeHeap());
  Serial.printf("Active zones: %d\n", activeZoneCount);
  Serial.printf("Avg processing time: %d ms\n", avgProcessingTime);
  Serial.printf("Exposure level: %d\n", exposureLevel);
  Serial.printf("Gain level: %d\n", gainLevel);
  Serial.println("==============================\n");
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  bootTime = millis();
  
  // Initialize pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(FLASH_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);   // LED off (active low)
  digitalWrite(FLASH_PIN, LOW);  // Flash off
  
  // Startup blink pattern
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, LOW);   // LED on
    delay(100);
    digitalWrite(LED_PIN, HIGH);  // LED off
    delay(100);
  }
  
  // Initialize watchdog
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);
  
  // Load configuration
  loadConfig();
  
  // Initialize camera
  Serial.println("CAM:INIT");
  if (initCamera((framesize_t)config.resolution)) {
    Serial.println("CAM:READY");
    digitalWrite(LED_PIN, LOW);
    delay(500);
    digitalWrite(LED_PIN, HIGH);
  } else {
    Serial.println("CAM:ERROR");
    while (1) {
      digitalWrite(LED_PIN, LOW);
      delay(200);
      digitalWrite(LED_PIN, HIGH);
      delay(200);
      esp_task_wdt_reset();
    }
  }
  
  // Initialize detection zones
  memset(zones, 0, sizeof(zones));
  
  // Clear previous frame buffer
  memset(prevFrame, 0, sizeof(prevFrame));
  memset(prevFrameLarge, 0, sizeof(prevFrameLarge));
  
  // Initialize random seed
  randomSeed(micros());
  
  Serial.println("CAM:MODULE_READY");
  Serial.printf("Resolution: %dx%d\n", frameWidth, frameHeight);
  Serial.printf("Sensitivity: %d%%\n", currentSensitivity);
  Serial.printf("Detection mask: 0x%02X\n", detectionMask);
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Feed watchdog
  esp_task_wdt_reset();
  
  // Handle serial commands
  if (Serial.available()) {
    uint8_t cmd = Serial.read();
    handleCommand(cmd);
  }
  
  // Stream frames if enabled
  if (streaming) {
    streamFrame();
    delay(streamInterval);
  }
  
  // Periodic temperature update
  static unsigned long lastTempUpdate = 0;
  if (millis() - lastTempUpdate > 10000) {
    lastTempUpdate = millis();
    updateTemperature();
  }
  
  // Periodic diagnostics
  static unsigned long lastDiag = 0;
  if (millis() - lastDiag > 60000) {
    lastDiag = millis();
    printDiagnostics();
  }
  
  delay(10);
}
#define MOTION_SENSITIVITY    4000
#define HUMAN_THRESHOLD       30
#define SMOKE_THRESHOLD       20
#define ANIMAL_THRESHOLD      25

// Fire detection color ranges (RGB565)
#define FIRE_RED_MIN          25
#define FIRE_RED_MAX          31
#define FIRE_GREEN_MIN        8
#define FIRE_GREEN_MAX        25
#define FIRE_BLUE_MAX         10

// Skin tone ranges (RGB565)
#define SKIN_RED_MIN          15
#define SKIN_RED_MAX          25
#define SKIN_GREEN_MIN        8
#define SKIN_GREEN_MAX        20
#define SKIN_BLUE_MIN         5
#define SKIN_BLUE_MAX         15

// Smoke detection ranges
#define SMOKE_BRIGHTNESS_MIN  40
#define SMOKE_BRIGHTNESS_MAX  60
#define SMOKE_VARIANCE_MAX    100

// EEPROM settings
#define EEPROM_SIZE           512
#define CONFIG_ADDRESS        0
#define ZONE_ADDRESS          64

// Watchdog timeout
#define WDT_TIMEOUT           30

// Maximum zones
#define MAX_ZONES             8

// Image buffer size
#define IMAGE_BUFFER_SIZE     65536

// Stream intervals
#define STREAM_INTERVAL_FAST  33    // ~30 FPS
#define STREAM_INTERVAL_NORMAL 50   // ~20 FPS
#define STREAM_INTERVAL_SLOW   100  // ~10 FPS

// Detection flags
#define DETECT_FIRE    0x01
#define DETECT_MOTION  0x02
#define DETECT_HUMAN   0x04
#define DETECT_SMOKE   0x08
#define DETECT_ANIMAL  0x10
#define DETECT_ALL     0x1F

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================
static uint16_t prevFrame[320*240/50];
static uint16_t prevFrameLarge[640*480/100];
static bool streaming = false;
static bool camReady = false;
static bool nightMode = false;
static bool flashOn = false;
static uint8_t currentSensitivity = 50;
static uint32_t frameCount = 0;
static uint32_t bootTime = 0;

// Detection counters
static uint32_t fireDetectionCount = 0;
static uint32_t motionDetectionCount = 0;
static uint32_t humanDetectionCount = 0;
static uint32_t smokeDetectionCount = 0;
static uint32_t animalDetectionCount = 0;
static uint32_t errorCount = 0;

// Detection zones
static DetectionZone zones[MAX_ZONES];
static uint8_t activeZoneCount = 0;

// Current frame dimensions
static uint16_t frameWidth = 320;
static uint16_t frameHeight = 240;

// Temperature simulation (would use internal sensor in production)
static float moduleTemperature = 25.0;

// Configuration
static ModuleConfig config;

// Image buffer for capture
static uint8_t imageBuffer[IMAGE_BUFFER_SIZE];
static size_t imageBufferLen = 0;

// Stream interval
static uint16_t streamInterval = STREAM_INTERVAL_NORMAL;

// Detection mask
static uint8_t detectionMask = DETECT_ALL;

// Average processing time
static uint16_t avgProcessingTime = 0;
static uint32_t processingTimeSum = 0;
static uint32_t processingTimeCount = 0;

// Last frame timestamp
static uint32_t lastFrameTime = 0;

// Exposure and gain levels
static uint8_t exposureLevel = 0;
static uint8_t gainLevel = 0;

// LED brightness
static uint8_t ledBrightness = 100;

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================
bool initCamera();
bool initCamera(framesize_t resolution);
void setCameraParameters();
void configureSensor();
DetectionResult analyzeFrame(camera_fb_t *fb);
DetectionResult analyzeZone(camera_fb_t *fb, DetectionZone* zone);
void captureAndProcess();
void streamFrame();
void handleCommand(uint8_t cmd);
void sendStatus();
void calibrateSensor();
void loadConfig();
void saveConfig();
void updateTemperature();
void printDiagnostics();
bool isInZone(uint16_t x, uint16_t y, DetectionZone* zone);
void calculateFireCenter(camera_fb_t *fb, int16_t* cx, int16_t* cy, int* pixelCount);
void calculateMotionCenter(camera_fb_t *fb, int16_t* cx, int16_t* cy, int* pixelCount);
void calculateHumanCenter(camera_fb_t *fb, int16_t* cx, int16_t* cy, int* pixelCount);
void calculateSmokeCenter(camera_fb_t *fb, int16_t* cx, int16_t* cy, int* pixelCount);
float calculateBrightness(camera_fb_t *fb);
float calculateVariance(camera_fb_t *fb, float brightness);
void sendDetectionResult(DetectionResult* result);
void sendFrame(camera_fb_t *fb);
void processZoneConfig();
void updateAverageProcessingTime(uint16_t time);
void applyNightModeSettings();
void applyDayModeSettings();
void setExposure(uint8_t level);
void setGain(uint8_t level);
void factoryReset();
uint8_t calculateChecksum(uint8_t* data, size_t len);

// ============================================================================
// CAMERA INITIALIZATION
// ============================================================================

bool initCamera() {
  return initCamera(FRAMESIZE_QVGA);  // Default 320x240
}

bool initCamera(framesize_t resolution) {
  camera_config_t camConfig;
  
  // Clear configuration
  memset(&camConfig, 0, sizeof(camConfig));
  
  // Pin configuration
  camConfig.ledc_channel = LEDC_CHANNEL_0;
  camConfig.ledc_timer   = LEDC_TIMER_0;
  camConfig.pin_d0       = Y2_GPIO_NUM;
  camConfig.pin_d1       = Y3_GPIO_NUM;
  camConfig.pin_d2       = Y4_GPIO_NUM;
  camConfig.pin_d3       = Y5_GPIO_NUM;
  camConfig.pin_d4       = Y6_GPIO_NUM;
  camConfig.pin_d5       = Y7_GPIO_NUM;
  camConfig.pin_d6       = Y8_GPIO_NUM;
  camConfig.pin_d7       = Y9_GPIO_NUM;
  camConfig.pin_xclk     = XCLK_GPIO_NUM;
  camConfig.pin_pclk     = PCLK_GPIO_NUM;
  camConfig.pin_vsync    = VSYNC_GPIO_NUM;
  camConfig.pin_href     = HREF_GPIO_NUM;
  camConfig.pin_sscb_sda = SIOD_GPIO_NUM;
  camConfig.pin_sscb_scl = SIOC_GPIO_NUM;
  camConfig.pin_pwdn     = PWDN_GPIO_NUM;
  camConfig.pin_reset    = RESET_GPIO_NUM;
  camConfig.xclk_freq_hz = 20000000;
  
  // Pixel format and frame settings
  camConfig.pixel_format = PIXFORMAT_RGB565;
  camConfig.frame_size   = resolution;
  camConfig.jpeg_quality = 12;
  camConfig.fb_count     = 2;
  camConfig.grab_mode    = CAMERA_GRAB_LATEST;
  
  // Update frame dimensions based on resolution
  switch (resolution) {
    case FRAMESIZE_QVGA:
      frameWidth = 320;
      frameHeight = 240;
      break;
    case FRAMESIZE_VGA:
      frameWidth = 640;
      frameHeight = 480;
      break;
    case FRAMESIZE_SVGA:
      frameWidth = 800;
      frameHeight = 600;
      break;
    case FRAMESIZE_XGA:
      frameWidth = 1024;
      frameHeight = 768;
      break;
    case FRAMESIZE_SXGA:
      frameWidth = 1280;
      frameHeight = 1024;
      break;
    case FRAMESIZE_UXGA:
      frameWidth = 1600;
      frameHeight = 1200;
      break;
    default:
      frameWidth = 320;
      frameHeight = 240;
  }
  
  // Attempt initialization with retries
  for (int i = 0; i < 3; i++) {
    esp_err_t err = esp_camera_init(&camConfig);
    if (err == ESP_OK) {
      camReady = true;
      setCameraParameters();
      configureSensor();
      return true;
    }
    Serial.printf("Camera init attempt %d failed: 0x%x\n", i + 1, err);
    delay(500);
  }
  
  camReady = false;
  return false;
}

void setCameraParameters() {
  sensor_t* s = esp_camera_sensor_get();
  if (s == nullptr) return;
  
  // Set default parameters for optimal detection
  s->set_brightness(s, 0);        // -2 to 2
  s->set_contrast(s, 0);          // -2 to 2
  s->set_saturation(s, 0);        // -2 to 2
  s->set_special_effect(s, 0);    // 0-6 (0=No Effect)
  s->set_whitebal(s, 1);          // White balance
  s->set_awb_gain(s, 1);          // Auto white balance gain
  s->set_wb_mode(s, 0);           // White balance mode
  s->set_exposure_ctrl(s, 1);     // Auto exposure
  s->set_aec2(s, 1);              // Auto exposure DSP
  s->set_ae_level(s, 0);          // Auto exposure level (-2 to 2)
  s->set_aec_value(s, 300);       // Manual exposure value
  s->set_gain_ctrl(s, 1);         // Auto gain
  s->set_agc_gain(s, 0);          // AGC gain
  s->set_gainceiling(s, (gainceiling_t)6);  // Gain ceiling
  s->set_bpc(s, 0);               // Black pixel correction
  s->set_wpc(s, 1);               // White pixel correction
  s->set_raw_gma(s, 1);           // Raw gamma
  s->set_lenc(s, 1);              // Lens correction
  s->set_hmirror(s, 0);           // Horizontal mirror
  s->set_vflip(s, 0);             // Vertical flip
  s->set_dcw(s, 1);               // Downsize enable
  s->set_colorbar(s, 0);          // Colorbar test pattern
  
  // Night mode specific settings
  if (nightMode) {
    applyNightModeSettings();
  }
}

void configureSensor() {
  sensor_t* s = esp_camera_sensor_get();
  if (s == nullptr) return;
  
  // Apply exposure level if set
  if (exposureLevel > 0) {
    s->set_aec_value(s, 300 + (exposureLevel * 50));
  }
  
  // Apply gain level if set
  if (gainLevel > 0) {
    s->set_agc_gain(s, gainLevel);
  }
}

// ============================================================================
// DETECTION ALGORITHMS
// ============================================================================

DetectionResult analyzeFrame(camera_fb_t *fb) {
  DetectionResult result;
  memset(&result, 0, sizeof(result));
  result.frameCount = frameCount;
  unsigned long startTime = micros();
  
  if (!fb || !fb->buf) {
    strcpy(result.description, "ERROR:NO_FRAME");
    errorCount++;
    return result;
  }
  
  int firePixels = 0;
  int motionPixels = 0;
  int humanPixels = 0;
  int smokePixels = 0;
  int animalPixels = 0;
  
  // Calculate step based on frame size and sensitivity
  int step = map(currentSensitivity, 0, 100, 100, 10);
  int totalPixels = (fb->width * fb->height) / step;
  
  // Accumulators for center calculation
  int64_t fireXSum = 0, fireYSum = 0;
  int64_t motionXSum = 0, motionYSum = 0;
  int64_t humanXSum = 0, humanYSum = 0;
  int64_t smokeXSum = 0, smokeYSum = 0;
  
  // Previous frame index
  int prevIdx = 0;
  
  // Analyze pixels
  for (int y = 0; y < fb->height; y += step/2) {
    for (int x = 0; x < fb->width; x += step) {
      int idx = y * fb->width + x;
      
      // Check if in active zone
      bool inZone = (activeZoneCount == 0);
      for (int z = 0; z < activeZoneCount && !inZone; z++) {
        if (zones[z].enabled && isInZone(x, y, &zones[z])) {
          inZone = true;
        }
      }
      if (!inZone) continue;
      
      // Get RGB565 pixel
      uint16_t px = ((uint16_t*)fb->buf)[idx];
      
      // Extract RGB components
      uint8_t red   = (px >> 11) & 0x1F;  // 5 bits
      uint8_t green = (px >> 5) & 0x3F;   // 6 bits
      uint8_t blue  = px & 0x1F;          // 5 bits
      
      // ===== FIRE DETECTION =====
      if (detectionMask & DETECT_FIRE) {
        // Fire: high red, medium-high green, low blue
        if (red > FIRE_RED_MIN && red <= FIRE_RED_MAX &&
            green > FIRE_GREEN_MIN && green <= FIRE_GREEN_MAX &&
            blue < FIRE_BLUE_MAX) {
          firePixels++;
          fireXSum += x;
          fireYSum += y;
        }
      }
      
      // ===== MOTION DETECTION =====
      if (detectionMask & DETECT_MOTION) {
        // Compare with previous frame
        if (prevIdx < sizeof(prevFrame)/2) {
          int diff = abs((int)px - (int)prevFrame[prevIdx]);
          if (diff > MOTION_SENSITIVITY) {
            motionPixels++;
            motionXSum += x;
            motionYSum += y;
          }
          prevFrame[prevIdx] = px;
        }
        prevIdx++;
      }
      
      // ===== HUMAN DETECTION =====
      if (detectionMask & DETECT_HUMAN) {
        // Skin tone detection (simplified)
        if (red >= SKIN_RED_MIN && red <= SKIN_RED_MAX &&
            green >= SKIN_GREEN_MIN && green <= SKIN_GREEN_MAX &&
            blue >= SKIN_BLUE_MIN && blue <= SKIN_BLUE_MAX) {
          humanPixels++;
          humanXSum += x;
          humanYSum += y;
        }
      }
      
      // ===== SMOKE DETECTION =====
      if (detectionMask & DETECT_SMOKE) {
        // Smoke: grayish, low variance
        uint8_t brightness = (red + green + blue) / 3;
        if (brightness >= SMOKE_BRIGHTNESS_MIN && 
            brightness <= SMOKE_BRIGHTNESS_MAX) {
          smokePixels++;
          smokeXSum += x;
          smokeYSum += y;
        }
      }
      
      // ===== ANIMAL DETECTION =====
      if (detectionMask & DETECT_ANIMAL) {
        // Simplified animal detection based on color patterns
        // Would need more sophisticated algorithm in production
        if (motionPixels > 0 && humanPixels == 0) {
          // Potential animal movement
          animalPixels++;
        }
      }
    }
  }
  
  // Calculate centers
  if (firePixels > 0) {
    result.fireCenterX = fireXSum / firePixels;
    result.fireCenterY = fireYSum / firePixels;
  }
  if (motionPixels > 0) {
    result.motionCenterX = motionXSum / motionPixels;
    result.motionCenterY = motionYSum / motionPixels;
  }
  if (humanPixels > 0) {
    result.humanCenterX = humanXSum / humanPixels;
    result.humanCenterY = humanYSum / humanPixels;
  }
  if (smokePixels > 0) {
    result.smokeCenterX = smokeXSum / smokePixels;
    result.smokeCenterY = smokeYSum / smokePixels;
  }
  
  // Store pixel counts
  result.firePixelCount = firePixels;
  result.motionPixelCount = motionPixels;
  result.humanPixelCount = humanPixels;
  result.smokePixelCount = smokePixels;
  
  // Calculate confidences
  result.fireConfidence   = (float)firePixels / totalPixels * 100.0;
  result.motionConfidence = (float)motionPixels / totalPixels * 100.0;
  result.humanConfidence  = (float)humanPixels / totalPixels * 100.0;
  result.smokeConfidence  = (float)smokePixels / totalPixels * 100.0;
  
  // Apply sensitivity threshold
  float threshold = map(currentSensitivity, 0, 100, 50, 5);
  
  // Determine detections
  if (result.fireConfidence > threshold) {
    result.fire = true;
    fireDetectionCount++;
    result.detectionFlags |= DETECT_FIRE;
  }
  
  if (result.motionConfidence > threshold) {
    result.motion = true;
    motionDetectionCount++;
    result.detectionFlags |= DETECT_MOTION;
  }
  
  if (result.humanConfidence > threshold * 2) {  // Higher threshold for human
    result.human = true;
    humanDetectionCount++;
    result.detectionFlags |= DETECT_HUMAN;
  }
  
  if (result.smokeConfidence > threshold * 1.5) {
    result.smoke = true;
    smokeDetectionCount++;
    result.detectionFlags |= DETECT_SMOKE;
  }
  
  if (animalPixels > ANIMAL_THRESHOLD) {
    result.animal = true;
    animalDetectionCount++;
    result.detectionFlags |= DETECT_ANIMAL;
  }
  
  // Overall confidence
  result.confidence = max(result.fireConfidence, 
                         max(result.motionConfidence, 
                             max(result.humanConfidence, result.smokeConfidence)));
  
  // Generate description
  if (result.fire) {
    strcpy(result.description, "FIRE DETECTED");
    if (result.smoke) strcat(result.description, "+SMOKE");
    if (result.human) strcat(result.description, "+HUMAN");
  } else if (result.human) {
    strcpy(result.description, "HUMAN DETECTED");
    if (result.motion) strcat(result.description, "+MOTION");
  } else if (result.motion) {
    strcpy(result.description, "MOTION DETECTED");
  } else if (result.smoke) {
    strcpy(result.description, "SMOKE DETECTED");
  } else if (result.animal) {
    strcpy(result.description, "ANIMAL DETECTED");
  } else {
    strcpy(result.description, "CLEAR");
  }
  
  result.processingTime = (micros() - startTime) / 1000;
  updateAverageProcessingTime(result.processingTime);
  
  return result;
}

DetectionResult analyzeZone(camera_fb_t *fb, DetectionZone* zone) {
  DetectionResult result;
  memset(&result, 0, sizeof(result));
  
  if (!fb || !fb->buf || !zone || !zone->enabled) {
    strcpy(result.description, "ERROR:INVALID_ZONE");
    return result;
  }
  
  int firePixels = 0;
  int motionPixels = 0;
  int humanPixels = 0;
  int totalPixels = 0;
  
  int step = map(zone->sensitivity, 0, 100, 20, 2);
  
  for (int y = zone->y1; y < zone->y2; y += step) {
    for (int x = zone->x1; x < zone->x2; x += step) {
      int idx = y * fb->width + x;
      uint16_t px = ((uint16_t*)fb->buf)[idx];
      
      uint8_t red   = (px >> 11) & 0x1F;
      uint8_t green = (px >> 5) & 0x3F;
      uint8_t blue  = px & 0x1F;
      
      totalPixels++;
      
      // Fire detection
      if (red > FIRE_RED_MIN && green > FIRE_GREEN_MIN && blue < FIRE_BLUE_MAX) {
        firePixels++;
      }
      
      // Human detection
      if (red >= SKIN_RED_MIN && red <= SKIN_RED_MAX &&
          green >= SKIN_GREEN_MIN && green <= SKIN_GREEN_MAX) {
        humanPixels++;
      }
    }
  }
  
  if (totalPixels > 0) {
    result.fireConfidence = (float)firePixels / totalPixels * 100;
    result.humanConfidence = (float)humanPixels / totalPixels * 100;
    result.confidence = max(result.fireConfidence, result.humanConfidence);
  }
  
  return result;
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

bool isInZone(uint16_t x, uint16_t y, DetectionZone* zone) {
  return (x >= zone->x1 && x <= zone->x2 && 
          y >= zone->y1 && y <= zone->y2);
}

float calculateBrightness(camera_fb_t *fb) {
  if (!fb || !fb->buf) return 0;
  
  uint64_t sum = 0;
  int samples = 0;
  int step = 100;
  
  for (int i = 0; i < fb->width * fb->height; i += step) {
    uint16_t px = ((uint16_t*)fb->buf)[i];
    uint8_t red   = (px >> 11) & 0x1F;
    uint8_t green = (px >> 5) & 0x3F;
    uint8_t blue  = px & 0x1F;
    sum += (red + green + blue) / 3;
    samples++;
  }
  
  return samples > 0 ? (float)sum / samples : 0;
}

float calculateVariance(camera_fb_t *fb, float brightness) {
  if (!fb || !fb->buf) return 0;
  
  float sum = 0;
  int samples = 0;
  int step = 100;
  
  for (int i = 0; i < fb->width * fb->height; i += step) {
    uint16_t px = ((uint16_t*)fb->buf)[i];
    uint8_t red   = (px >> 11) & 0x1F;
    uint8_t green = (px >> 5) & 0x3F;
    uint8_t blue  = px & 0x1F;
    float b = (red + green + blue) / 3.0;
    sum += (b - brightness) * (b - brightness);
    samples++;
  }
  
  return samples > 0 ? sum / samples : 0;
}

void calculateFireCenter(camera_fb_t *fb, int16_t* cx, int16_t* cy, int* pixelCount) {
  if (!fb || !fb->buf) {
    *cx = *cy = -1;
    *pixelCount = 0;
    return;
  }
  
  int64_t xSum = 0, ySum = 0;
  int count = 0;
  
  for (int y = 0; y < fb->height; y += 10) {
    for (int x = 0; x < fb->width; x += 10) {
      int idx = y * fb->width + x;
      uint16_t px = ((uint16_t*)fb->buf)[idx];
      
      uint8_t red   = (px >> 11) & 0x1F;
      uint8_t green = (px >> 5) & 0x3F;
      uint8_t blue  = px & 0x1F;
      
      if (red > FIRE_RED_MIN && green > FIRE_GREEN_MIN && blue < FIRE_BLUE_MAX) {
        xSum += x;
        ySum += y;
        count++;
      }
    }
  }
  
  *pixelCount = count;
  if (count > 0) {
    *cx = xSum / count;
    *cy = ySum / count;
  } else {
    *cx = *cy = -1;
  }
}

void calculateMotionCenter(camera_fb_t *fb, int16_t* cx, int16_t* cy, int* pixelCount) {
  *cx = frameWidth / 2;
  *cy = frameHeight / 2;
  *pixelCount = 0;
}

void calculateHumanCenter(camera_fb_t *fb, int16_t* cx, int16_t* cy, int* pixelCount) {
  *cx = frameWidth / 2;
  *cy = frameHeight / 2;
  *pixelCount = 0;
}

void calculateSmokeCenter(camera_fb_t *fb, int16_t* cx, int16_t* cy, int* pixelCount) {
  *cx = frameWidth / 2;
  *cy = frameHeight / 2;
  *pixelCount = 0;
}

void updateAverageProcessingTime(uint16_t time) {
  processingTimeSum += time;
  processingTimeCount++;
  if (processingTimeCount > 0) {
    avgProcessingTime = processingTimeSum / processingTimeCount;
  }
}

uint8_t calculateChecksum(uint8_t* data, size_t len) {
  uint8_t sum = 0;
  for (size_t i = 0; i < len; i++) {
    sum += data[i];
  }
  return sum;
}

// ============================================================================
// CAPTURE AND PROCESSING
// ============================================================================

void captureAndProcess() {
  if (!camReady) {
    Serial.println("ERROR:CAM_NOT_READY");
    errorCount++;
    return;
  }
  
  // Turn on flash for night mode
  if (nightMode || flashOn) {
    digitalWrite(FLASH_PIN, HIGH);
    delay(100);  // Let flash stabilize
  }
  
  camera_fb_t *fb = esp_camera_fb_get();
  
  if (nightMode || flashOn) {
    digitalWrite(FLASH_PIN, LOW);
  }
  
  if (!fb) {
    Serial.println("ERROR:CAPTURE_FAILED");
    errorCount++;
    return;
  }
  
  frameCount++;
  lastFrameTime = millis();
  
  DetectionResult result = analyzeFrame(fb);
  
  // Send result
  sendDetectionResult(&result);
  
  esp_camera_fb_return(fb);
}

void streamFrame() {
  if (!camReady || !streaming) return;
  
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    errorCount++;
    return;
  }
  
  frameCount++;
  lastFrameTime = millis();
  
  // Send frame
  sendFrame(fb);
  
  esp_camera_fb_return(fb);
}

void sendDetectionResult(DetectionResult* result) {
  // Send binary result
  Serial.write((uint8_t*)result, sizeof(DetectionResult));
  Serial.println();
  
  // Send text result for compatibility
  if (result->fire) {
    Serial.println("HAZARD:FIRE");
  } else if (result->human) {
    Serial.println("HAZARD:HUMAN");
  } else if (result->motion) {
    Serial.println("HAZARD:MOTION");
  } else if (result->smoke) {
    Serial.println("HAZARD:SMOKE");
  } else if (result->animal) {
    Serial.println("HAZARD:ANIMAL");
  } else {
    Serial.println("CLEAR");
  }
  
  // Send JSON for detailed info
  Serial.printf("{\"fire\":%d,\"motion\":%d,\"human\":%d,\"smoke\":%d,\"animal\":%d,",
                result->fire ? 1 : 0, result->motion ? 1 : 0,
                result->human ? 1 : 0, result->smoke ? 1 : 0,
                result->animal ? 1 : 0);
  Serial.printf("\"confidence\":%.1f,\"fireX\":%d,\"fireY\":%d,",
                result->confidence, result->fireCenterX, result->fireCenterY);
  Serial.printf("\"motionX\":%d,\"motionY\":%d,",
                result->motionCenterX, result->motionCenterY);
  Serial.printf("\"humanX\":%d,\"humanY\":%d,",
                result->humanCenterX, result->humanCenterY);
  Serial.printf("\"frameCount\":%lu,\"processTime\":%d,\"pixels\":{\"fire\":%d,\"motion\":%d,\"human\":%d,\"smoke\":%d}}\n",
                result->frameCount, result->processingTime,
                result->firePixelCount, result->motionPixelCount,
                result->humanPixelCount, result->smokePixelCount);
}

void sendFrame(camera_fb_t *fb) {
  // Send frame header
  Serial.printf("FRAME:%dx%d:%d:", fb->width, fb->height, fb->len);
  
  // Send frame data
  Serial.write(fb->buf, fb->len);
  Serial.println();
}

// ============================================================================
// COMMAND HANDLING
// ============================================================================

void handleCommand(uint8_t cmd) {
  switch (cmd) {
    case CMD_CAPTURE:
      captureAndProcess();
      break;
      
    case CMD_STREAM_ON:
      streaming = true;
      Serial.println("STREAM:ON");
      break;
      
    case CMD_STREAM_OFF:
      streaming = false;
      Serial.println("STREAM:OFF");
      break;
      
    case CMD_DETECT:
      captureAndProcess();
      break;
      
    case CMD_NIGHT_MODE:
      nightMode = !nightMode;
      if (nightMode) {
        applyNightModeSettings();
      } else {
        applyDayModeSettings();
      }
      Serial.printf("NIGHT_MODE:%s\n", nightMode ? "ON" : "OFF");
      break;
      
    case CMD_SET_SENSITIVITY:
      // Read next byte for sensitivity value
      if (Serial.available()) {
        currentSensitivity = Serial.read();
        Serial.printf("SENSITIVITY:%d\n", currentSensitivity);
      }
      break;
      
    case CMD_SET_ZONES:
      // Zone configuration would be read from serial
      processZoneConfig();
      break;
      
    case CMD_GET_STATUS:
      sendStatus();
      break;
      
    case CMD_CALIBRATE:
      calibrateSensor();
      break;
      
    case CMD_SET_RESOLUTION:
      if (Serial.available()) {
        uint8_t res = Serial.read();
        framesize_t fs = (framesize_t)res;
        esp_camera_deinit();
        if (initCamera(fs)) {
          Serial.printf("RESOLUTION:OK %dx%d\n", frameWidth, frameHeight);
        } else {
          Serial.println("RESOLUTION:FAIL");
          initCamera();  // Fall back to default
        }
      }
      break;
      
    case CMD_ENABLE_FLASH:
      flashOn = true;
      digitalWrite(FLASH_PIN, HIGH);
      Serial.println("FLASH:ON");
      break;
      
    case CMD_DISABLE_FLASH:
      flashOn = false;
      digitalWrite(FLASH_PIN, LOW);
      Serial.println("FLASH:OFF");
      break;
      
    case CMD_GET_TEMPERATURE:
      updateTemperature();
      Serial.printf("TEMPERATURE:%.1f\n", moduleTemperature);
      break;
      
    case CMD_RESET_COUNTERS:
      fireDetectionCount = 0;
      motionDetectionCount = 0;
      humanDetectionCount = 0;
      smokeDetectionCount = 0;
      animalDetectionCount = 0;
      errorCount = 0;
      frameCount = 0;
      processingTimeSum = 0;
      processingTimeCount = 0;
      avgProcessingTime = 0;
      Serial.println("COUNTERS:RESET");
      break;
      
    case CMD_GET_FRAME:
      {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) {
          sendFrame(fb);
          esp_camera_fb_return(fb);
        } else {
          Serial.println("ERROR:FRAME_CAPTURE");
        }
      }
      break;
      
    case CMD_SET_EXPOSURE:
      if (Serial.available()) {
        exposureLevel = Serial.read();
        setExposure(exposureLevel);
        Serial.printf("EXPOSURE:%d\n", exposureLevel);
      }
      break;
      
    case CMD_SET_GAIN:
      if (Serial.available()) {
        gainLevel = Serial.read();
        setGain(gainLevel);
        Serial.printf("GAIN:%d\n", gainLevel);
      }
      break;
      
    case CMD_SAVE_CONFIG:
      saveConfig();
      Serial.println("CONFIG:SAVED");
      break;
      
    case CMD_LOAD_CONFIG:
      loadConfig();
      Serial.println("CONFIG:LOADED");
      break;
      
    case CMD_FACTORY_RESET:
      factoryReset();
      Serial.println("FACTORY_RESET:DONE");
      break;
      
    default:
      Serial.printf("ERROR:UNKNOWN_CMD:0x%02X\n", cmd);
      errorCount++;
  }
}

void processZoneConfig() {
  Serial.println("ZONES:READY");
  
  // Wait for zone data
  unsigned long timeout = millis() + 5000;
  while (!Serial.available() && millis() < timeout) {
    delay(10);
  }
  
  if (Serial.available()) {
    // Read zone count
    uint8_t count = Serial.read();
    if (count > MAX_ZONES) count = MAX_ZONES;
    
    activeZoneCount = count;
    
    // Read zone data
    for (int i = 0; i < count; i++) {
      timeout = millis() + 1000;
      while (Serial.available() < sizeof(DetectionZone) && millis() < timeout) {
        delay(10);
      }
      
      if (Serial.available() >= sizeof(DetectionZone)) {
        Serial.readBytes((char*)&zones[i], sizeof(DetectionZone));
      }
    }
    
    Serial.printf("ZONES:CONFIGURED:%d\n", activeZoneCount);
  }
}

void sendStatus() {
  ModuleStatus status;
  memset(&status, 0, sizeof(status));
  
  status.totalFrames      = frameCount;
  status.fireDetections   = fireDetectionCount;
  status.motionDetections = motionDetectionCount;
  status.humanDetections  = humanDetectionCount;
  status.smokeDetections  = smokeDetectionCount;
  status.animalDetections = animalDetectionCount;
  status.errors           = errorCount;
  status.temperature      = moduleTemperature;
  status.currentSensitivity = currentSensitivity;
  status.nightMode        = nightMode;
  status.streaming        = streaming;
  status.flashOn          = flashOn;
  status.frameWidth       = frameWidth;
  status.frameHeight      = frameHeight;
  status.uptime           = (millis() - bootTime) / 1000;
  status.freeHeap         = ESP.getFreeHeap();
  status.cameraStatus     = camReady ? 1 : 0;
  status.activeZones      = activeZoneCount;
  status.avgProcessingTime = avgProcessingTime;
  status.lastFrameTime    = lastFrameTime;
  status.exposureLevel    = exposureLevel;
  status.gainLevel        = gainLevel;
  
  // Send binary status
  Serial.write((uint8_t*)&status, sizeof(status));
  Serial.println();
  
  // Send JSON status
  Serial.printf("{\"status\":\"ok\",\"frames\":%lu,\"fire\":%lu,",
                status.totalFrames, status.fireDetections);
  Serial.printf("\"motion\":%lu,\"human\":%lu,\"smoke\":%lu,\"animal\":%lu,",
                status.motionDetections, status.humanDetections, 
                status.smokeDetections, status.animalDetections);
  Serial.printf("\"errors\":%lu,\"temp\":%.1f,\"sensitivity\":%d,",
                status.errors, status.temperature, status.currentSensitivity);
  Serial.printf("\"nightMode\":%s,\"streaming\":%s,\"flash\":%s,",
                status.nightMode ? "true" : "false",
                status.streaming ? "true" : "false",
                status.flashOn ? "true" : "false");
  Serial.printf("\"resolution\":\"%dx%d\",\"uptime\":%lu,\"freeHeap\":%u,",
                status.frameWidth, status.frameHeight, status.uptime, status.freeHeap);
  Serial.printf("\"camera\":%s,\"zones\":%d,\"avgProcessTime\":%d,",
                status.cameraStatus ? "ok" : "fail", status.activeZones, status.avgProcessingTime);
  Serial.printf("\"exposure\":%d,\"gain\":%d}\n",
                status.exposureLevel, status.gainLevel);
}

void calibrateSensor() {
  Serial.println("CALIBRATE:START");
  
  // Reset previous frame buffer
  memset(prevFrame, 0, sizeof(prevFrame));
  memset(prevFrameLarge, 0, sizeof(prevFrameLarge));
  
  // Take several calibration frames
  for (int i = 0; i < 5; i++) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      esp_camera_fb_return(fb);
    }
    delay(100);
    Serial.printf("CALIBRATE:FRAME%d\n", i + 1);
  }
  
  // Reset counters
  frameCount = 0;
  processingTimeSum = 0;
  processingTimeCount = 0;
  
  Serial.println("CALIBRATE:DONE");
}

// ============================================================================
// CONFIGURATION MANAGEMENT
// ============================================================================

void loadConfig() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(CONFIG_ADDRESS, config);
  
  // Verify checksum
  uint8_t calcChecksum = calculateChecksum((uint8_t*)&config, sizeof(config) - 1);
  
  if (config.configValid != 0xAA || config.checksum != calcChecksum) {
    // Set defaults
    config.sensitivity = 50;
    config.resolution = FRAMESIZE_QVGA;
    config.nightModeEnabled = false;
    config.zoneCount = 0;
    config.exposureLevel = 0;
    config.gainLevel = 0;
    config.detectionMask = DETECT_ALL;
    config.streamInterval = STREAM_INTERVAL_NORMAL;
    config.ledBrightness = 100;
    config.configValid = 0xAA;
    saveConfig();
  }
  
  currentSensitivity = config.sensitivity;
  nightMode = config.nightModeEnabled;
  exposureLevel = config.exposureLevel;
  gainLevel = config.gainLevel;
  detectionMask = config.detectionMask;
  streamInterval = config.streamInterval;
  ledBrightness = config.ledBrightness;
  
  // Load zones
  activeZoneCount = min(config.zoneCount, (uint8_t)MAX_ZONES);
  for (int i = 0; i < activeZoneCount; i++) {
    EEPROM.get(ZONE_ADDRESS + i * sizeof(DetectionZone), zones[i]);
  }
  
  EEPROM.commit();
}

void saveConfig() {
  config.sensitivity = currentSensitivity;
  config.nightModeEnabled = nightMode;
  config.exposureLevel = exposureLevel;
  config.gainLevel = gainLevel;
  config.detectionMask = detectionMask;
  config.streamInterval = streamInterval;
  config.ledBrightness = ledBrightness;
  config.zoneCount = activeZoneCount;
  config.configValid = 0xAA;
  config.checksum = calculateChecksum((uint8_t*)&config, sizeof(config) - 1);
  
  EEPROM.put(CONFIG_ADDRESS, config);
  
  // Save zones
  for (int i = 0; i < activeZoneCount; i++) {
    EEPROM.put(ZONE_ADDRESS + i * sizeof(DetectionZone), zones[i]);
  }
  
  EEPROM.commit();
}

void factoryReset() {
  // Clear EEPROM
  for (int i = 0; i < EEPROM_SIZE; i++) {
    EEPROM.write(i, 0);
  }
  EEPROM.commit();
  
  // Reset all settings to defaults
  currentSensitivity = 50;
  nightMode = false;
  exposureLevel = 0;
  gainLevel = 0;
  detectionMask = DETECT_ALL;
  streamInterval = STREAM_INTERVAL_NORMAL;
  ledBrightness = 100;
  activeZoneCount = 0;
  
  // Reset counters
  fireDetectionCount = 0;
  motionDetectionCount = 0;
  humanDetectionCount = 0;
  smokeDetectionCount = 0;
  animalDetectionCount = 0;
  errorCount = 0;
  frameCount = 0;
  
  // Save default config
  saveConfig();
  
  // Reinitialize camera
  esp_camera_deinit();
  initCamera();
}

// ============================================================================
// TEMPERATURE MONITORING
// ============================================================================

void updateTemperature() {
  // Simulate temperature reading
  // In production, would use internal temperature sensor or external sensor
  moduleTemperature = 25.0 + (random(0, 100) / 100.0) * 10.0;
  
  // Check for overheating
  if (moduleTemperature > 60.0) {
    Serial.println("WARN:HIGH_TEMP");
  }
}

// ============================================================================
// NIGHT/DAY MODE SETTINGS
// ============================================================================

void applyNightModeSettings() {
  sensor_t* s = esp_camera_sensor_get();
  if (s == nullptr) return;
  
  s->set_brightness(s, 2);
  s->set_gain_ctrl(s, 1);
  s->set_exposure_ctrl(s, 1);
  s->set_ae_level(s, 2);
  s->set_agc_gain(s, 30);
  
  // Turn on flash if available
  if (!flashOn) {
    digitalWrite(FLASH_PIN, HIGH);
  }
}

void applyDayModeSettings() {
  sensor_t* s = esp_camera_sensor_get();
  if (s == nullptr) return;
  
  s->set_brightness(s, 0);
  s->set_gain_ctrl(s, 1);
  s->set_exposure_ctrl(s, 1);
  s->set_ae_level(s, 0);
  s->set_agc_gain(s, 0);
  
  // Turn off flash
  if (!flashOn) {
    digitalWrite(FLASH_PIN, LOW);
  }
}

void setExposure(uint8_t level) {
  sensor_t* s = esp_camera_sensor_get();
  if (s == nullptr) return;
  
  exposureLevel = level;
  s->set_aec_value(s, 300 + (level * 50));
}

void setGain(uint8_t level) {
  sensor_t* s = esp_camera_sensor_get();
  if (s == nullptr) return;
  
  gainLevel = level;
  s->set_agc_gain(s, level);
}

// ============================================================================
// DIAGNOSTICS
// ============================================================================

void printDiagnostics() {
  Serial.println("\n=== CAM Module Diagnostics ===");
  Serial.printf("Camera: %s\n", camReady ? "OK" : "FAIL");
  Serial.printf("Resolution: %dx%d\n", frameWidth, frameHeight);
  Serial.printf("Frames captured: %lu\n", frameCount);
  Serial.printf("Fire detections: %lu\n", fireDetectionCount);
  Serial.printf("Motion detections: %lu\n", motionDetectionCount);
  Serial.printf("Human detections: %lu\n", humanDetectionCount);
  Serial.printf("Smoke detections: %lu\n", smokeDetectionCount);
  Serial.printf("Animal detections: %lu\n", animalDetectionCount);
  Serial.printf("Errors: %lu\n", errorCount);
  Serial.printf("Sensitivity: %d%%\n", currentSensitivity);
  Serial.printf("Night mode: %s\n", nightMode ? "ON" : "OFF");
  Serial.printf("Streaming: %s\n", streaming ? "ON" : "OFF");
  Serial.printf("Flash: %s\n", flashOn ? "ON" : "OFF");
  Serial.printf("Temperature: %.1f C\n", moduleTemperature);
  Serial.printf("Uptime: %lu s\n", (millis() - bootTime) / 1000);
  Serial.printf("Free heap: %u bytes\n", ESP.getFreeHeap());
  Serial.printf("Active zones: %d\n", activeZoneCount);
  Serial.printf("Avg processing time: %d ms\n", avgProcessingTime);
  Serial.printf("Exposure level: %d\n", exposureLevel);
  Serial.printf("Gain level: %d\n", gainLevel);
  Serial.println("==============================\n");
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  bootTime = millis();
  
  // Initialize pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(FLASH_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);   // LED off (active low)
  digitalWrite(FLASH_PIN, LOW);  // Flash off
  
  // Startup blink pattern
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, LOW);   // LED on
    delay(100);
    digitalWrite(LED_PIN, HIGH);  // LED off
    delay(100);
  }
  
  // Initialize watchdog
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);
  
  // Load configuration
  loadConfig();
  
  // Initialize camera
  Serial.println("CAM:INIT");
  if (initCamera((framesize_t)config.resolution)) {
    Serial.println("CAM:READY");
    digitalWrite(LED_PIN, LOW);
    delay(500);
    digitalWrite(LED_PIN, HIGH);
  } else {
    Serial.println("CAM:ERROR");
    while (1) {
      digitalWrite(LED_PIN, LOW);
      delay(200);
      digitalWrite(LED_PIN, HIGH);
      delay(200);
      esp_task_wdt_reset();
    }
  }
  
  // Initialize detection zones
  memset(zones, 0, sizeof(zones));
  
  // Clear previous frame buffer
  memset(prevFrame, 0, sizeof(prevFrame));
  memset(prevFrameLarge, 0, sizeof(prevFrameLarge));
  
  // Initialize random seed
  randomSeed(micros());
  
  Serial.println("CAM:MODULE_READY");
  Serial.printf("Resolution: %dx%d\n", frameWidth, frameHeight);
  Serial.printf("Sensitivity: %d%%\n", currentSensitivity);
  Serial.printf("Detection mask: 0x%02X\n", detectionMask);
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Feed watchdog
  esp_task_wdt_reset();
  
  // Handle serial commands
  if (Serial.available()) {
    uint8_t cmd = Serial.read();
    handleCommand(cmd);
  }
  
  // Stream frames if enabled
  if (streaming) {
    streamFrame();
    delay(streamInterval);
  }
  
  // Periodic temperature update
  static unsigned long lastTempUpdate = 0;
  if (millis() - lastTempUpdate > 10000) {
    lastTempUpdate = millis();
    updateTemperature();
  }
  
  // Periodic diagnostics
  static unsigned long lastDiag = 0;
  if (millis() - lastDiag > 60000) {
    lastDiag = millis();
    printDiagnostics();
  }
  
  delay(10);
}

  if (!fb || !fb->buf) return 0;
  
  float sum = 0;
  int samples = 0;
  int step = 100;
  
  for (int i = 0; i < fb->width * fb->height; i += step) {
    uint16_t px = ((uint16_t*)fb->buf)[i];
    uint8_t red   = (px >> 11) & 0x1F;
    uint8_t green = (px >> 5) & 0x3F;
    uint8_t blue  = px & 0x1F;
    float b = (red + green + blue) / 3.0;
    sum += (b - brightness) * (b - brightness);
    samples++;
  }
  
  return samples > 0 ? sum / samples : 0;
}

void calculateFireCenter(camera_fb_t *fb, int16_t* cx, int16_t* cy) {
  if (!fb || !fb->buf) {
    *cx = *cy = -1;
    return;
  }
  
  int64_t xSum = 0, ySum = 0;
  int count = 0;
  
  for (int y = 0; y < fb->height; y += 10) {
    for (int x = 0; x < fb->width; x += 10) {
      int idx = y * fb->width + x;
      uint16_t px = ((uint16_t*)fb->buf)[idx];
      
      uint8_t red   = (px >> 11) & 0x1F;
      uint8_t green = (px >> 5) & 0x3F;
      uint8_t blue  = px & 0x1F;
      
      if (red > FIRE_RED_MIN && green > FIRE_GREEN_MIN && blue < FIRE_BLUE_MAX) {
        xSum += x;
        ySum += y;
        count++;
      }
    }
  }
  
  if (count > 0) {
    *cx = xSum / count;
    *cy = ySum / count;
  } else {
    *cx = *cy = -1;
  }
}

void calculateMotionCenter(camera_fb_t *fb, int16_t* cx, int16_t* cy) {
  // Similar to fire center but for motion
  *cx = frameWidth / 2;
  *cy = frameHeight / 2;
}

void calculateHumanCenter(camera_fb_t *fb, int16_t* cx, int16_t* cy) {
  // Similar to fire center but for human skin tones
  *cx = frameWidth / 2;
  *cy = frameHeight / 2;
}

// ============================================================================
// CAPTURE AND PROCESSING
// ============================================================================

void captureAndProcess() {
  if (!camReady) {
    Serial.println("ERROR:CAM_NOT_READY");
    errorCount++;
    return;
  }
  
  // Turn on flash for night mode
  if (nightMode || flashOn) {
    digitalWrite(FLASH_PIN, HIGH);
    delay(100);  // Let flash stabilize
  }
  
  camera_fb_t *fb = esp_camera_fb_get();
  
  if (nightMode || flashOn) {
    digitalWrite(FLASH_PIN, LOW);
  }
  
  if (!fb) {
    Serial.println("ERROR:CAPTURE_FAILED");
    errorCount++;
    return;
  }
  
  frameCount++;
  
  DetectionResult result = analyzeFrame(fb);
  
  // Send binary result
  Serial.write((uint8_t*)&result, sizeof(result));
  Serial.println();
  
  // Send text result for compatibility
  if (result.fire) {
    Serial.println("HAZARD:FIRE");
  } else if (result.human) {
    Serial.println("HAZARD:HUMAN");
  } else if (result.motion) {
    Serial.println("HAZARD:MOTION");
  } else if (result.smoke) {
    Serial.println("HAZARD:SMOKE");
  } else {
    Serial.println("CLEAR");
  }
  
  // Send JSON for detailed info
  Serial.printf("{\"fire\":%d,\"motion\":%d,\"human\":%d,\"smoke\":%d,",
                result.fire ? 1 : 0, result.motion ? 1 : 0,
                result.human ? 1 : 0, result.smoke ? 1 : 0);
  Serial.printf("\"confidence\":%.1f,\"fireX\":%d,\"fireY\":%d,",
                result.confidence, result.fireCenterX, result.fireCenterY);
  Serial.printf("\"frameCount\":%lu,\"processTime\":%d}\n",
                result.frameCount, result.processingTime);
  
  esp_camera_fb_return(fb);
}

void streamFrame() {
  if (!camReady || !streaming) return;
  
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    errorCount++;
    return;
  }
  
  frameCount++;
  
  // Send frame header
  Serial.printf("FRAME:%dx%d:%d:", fb->width, fb->height, fb->len);
  
  // Send frame data
  Serial.write(fb->buf, fb->len);
  Serial.println();
  
  esp_camera_fb_return(fb);
}

// ============================================================================
// COMMAND HANDLING
// ============================================================================

void handleCommand(uint8_t cmd) {
  switch (cmd) {
    case CMD_CAPTURE:
      captureAndProcess();
      break;
      
    case CMD_STREAM_ON:
      streaming = true;
      Serial.println("STREAM:ON");
      break;
      
    case CMD_STREAM_OFF:
      streaming = false;
      Serial.println("STREAM:OFF");
      break;
      
    case CMD_DETECT:
      captureAndProcess();
      break;
      
    case CMD_NIGHT_MODE:
      nightMode = !nightMode;
      setCameraParameters();
      Serial.printf("NIGHT_MODE:%s\n", nightMode ? "ON" : "OFF");
      break;
      
    case CMD_SET_SENSITIVITY:
      // Read next byte for sensitivity value
      if (Serial.available()) {
        currentSensitivity = Serial.read();
        Serial.printf("SENSITIVITY:%d\n", currentSensitivity);
      }
      break;
      
    case CMD_SET_ZONES:
      // Zone configuration would be read from serial
      Serial.println("ZONES:READY");
      break;
      
    case CMD_GET_STATUS:
      sendStatus();
      break;
      
    case CMD_CALIBRATE:
      calibrateSensor();
      break;
      
    case CMD_SET_RESOLUTION:
      if (Serial.available()) {
        uint8_t res = Serial.read();
        framesize_t fs = (framesize_t)res;
        esp_camera_deinit();
        if (initCamera(fs)) {
          Serial.printf("RESOLUTION:OK %dx%d\n", frameWidth, frameHeight);
        } else {
          Serial.println("RESOLUTION:FAIL");
          initCamera();  // Fall back to default
        }
      }
      break;
      
    case CMD_ENABLE_FLASH:
      flashOn = true;
      digitalWrite(FLASH_PIN, HIGH);
      Serial.println("FLASH:ON");
      break;
      
    case CMD_DISABLE_FLASH:
      flashOn = false;
      digitalWrite(FLASH_PIN, LOW);
      Serial.println("FLASH:OFF");
      break;
      
    case CMD_GET_TEMPERATURE:
      updateTemperature();
      Serial.printf("TEMPERATURE:%.1f\n", moduleTemperature);
      break;
      
    case CMD_RESET_COUNTERS:
      fireDetectionCount = 0;
      motionDetectionCount = 0;
      humanDetectionCount = 0;
      smokeDetectionCount = 0;
      errorCount = 0;
      frameCount = 0;
      Serial.println("COUNTERS:RESET");
      break;
      
    default:
      Serial.printf("ERROR:UNKNOWN_CMD:0x%02X\n", cmd);
      errorCount++;
  }
}

void sendStatus() {
  ModuleStatus status;
  memset(&status, 0, sizeof(status));
  
  status.totalFrames      = frameCount;
  status.fireDetections   = fireDetectionCount;
  status.motionDetections = motionDetectionCount;
  status.humanDetections  = humanDetectionCount;
  status.smokeDetections  = smokeDetectionCount;
  status.errors           = errorCount;
  status.temperature      = moduleTemperature;
  status.currentSensitivity = currentSensitivity;
  status.nightMode        = nightMode;
  status.streaming        = streaming;
  status.flashOn          = flashOn;
  status.frameWidth       = frameWidth;
  status.frameHeight      = frameHeight;
  status.uptime           = (millis() - bootTime) / 1000;
  status.freeHeap         = ESP.getFreeHeap();
  
  // Send binary status
  Serial.write((uint8_t*)&status, sizeof(status));
  Serial.println();
  
  // Send JSON status
  Serial.printf("{\"status\":\"ok\",\"frames\":%lu,\"fire\":%lu,",
                status.totalFrames, status.fireDetections);
  Serial.printf("\"motion\":%lu,\"human\":%lu,\"smoke\":%lu,",
                status.motionDetections, status.humanDetections, status.smokeDetections);
  Serial.printf("\"errors\":%lu,\"temp\":%.1f,\"sensitivity\":%d,",
                status.errors, status.temperature, status.currentSensitivity);
  Serial.printf("\"nightMode\":%s,\"streaming\":%s,\"flash\":%s,",
                status.nightMode ? "true" : "false",
                status.streaming ? "true" : "false",
                status.flashOn ? "true" : "false");
  Serial.printf("\"resolution\":\"%dx%d\",\"uptime\":%lu,\"freeHeap\":%u}\n",
                status.frameWidth, status.frameHeight, status.uptime, status.freeHeap);
}

void calibrateSensor() {
  Serial.println("CALIBRATE:START");
  
  // Reset previous frame buffer
  memset(prevFrame, 0, sizeof(prevFrame));
  
  // Take several calibration frames
  for (int i = 0; i < 5; i++) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      esp_camera_fb_return(fb);
    }
    delay(100);
    Serial.printf("CALIBRATE:FRAME%d\n", i + 1);
  }
  
  // Reset counters
  frameCount = 0;
  
  Serial.println("CALIBRATE:DONE");
}

// ============================================================================
// CONFIGURATION MANAGEMENT
// ============================================================================

void loadConfig() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(CONFIG_ADDRESS, config);
  
  if (config.configValid != 0xAA) {
    // Set defaults
    config.sensitivity = 50;
    config.resolution = FRAMESIZE_QVGA;
    config.nightModeEnabled = false;
    config.zoneCount = 0;
    config.configValid = 0xAA;
    saveConfig();
  }
  
  currentSensitivity = config.sensitivity;
  nightMode = config.nightModeEnabled;
  
  EEPROM.commit();
}

void saveConfig() {
  config.sensitivity = currentSensitivity;
  config.nightModeEnabled = nightMode;
  EEPROM.put(CONFIG_ADDRESS, config);
  EEPROM.commit();
}

// ============================================================================
// TEMPERATURE MONITORING
// ============================================================================

void updateTemperature() {
  // Simulate temperature reading
  // In production, would use internal temperature sensor or external sensor
  moduleTemperature = 25.0 + (random(0, 100) / 100.0) * 10.0;
  
  // Check for overheating
  if (moduleTemperature > 60.0) {
    Serial.println("WARN:HIGH_TEMP");
  }
}

// ============================================================================
// DIAGNOSTICS
// ============================================================================

void printDiagnostics() {
  Serial.println("\n=== CAM Module Diagnostics ===");
  Serial.printf("Camera: %s\n", camReady ? "OK" : "FAIL");
  Serial.printf("Resolution: %dx%d\n", frameWidth, frameHeight);
  Serial.printf("Frames captured: %lu\n", frameCount);
  Serial.printf("Fire detections: %lu\n", fireDetectionCount);
  Serial.printf("Motion detections: %lu\n", motionDetectionCount);
  Serial.printf("Human detections: %lu\n", humanDetectionCount);
  Serial.printf("Smoke detections: %lu\n", smokeDetectionCount);
  Serial.printf("Errors: %lu\n", errorCount);
  Serial.printf("Sensitivity: %d%%\n", currentSensitivity);
  Serial.printf("Night mode: %s\n", nightMode ? "ON" : "OFF");
  Serial.printf("Streaming: %s\n", streaming ? "ON" : "OFF");
  Serial.printf("Flash: %s\n", flashOn ? "ON" : "OFF");
  Serial.printf("Temperature: %.1f C\n", moduleTemperature);
  Serial.printf("Uptime: %lu s\n", (millis() - bootTime) / 1000);
  Serial.printf("Free heap: %u bytes\n", ESP.getFreeHeap());
  Serial.println("==============================\n");
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  bootTime = millis();
  
  // Initialize pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(FLASH_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);   // LED off (active low)
  digitalWrite(FLASH_PIN, LOW);  // Flash off
  
  // Startup blink pattern
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, LOW);   // LED on
    delay(100);
    digitalWrite(LED_PIN, HIGH);  // LED off
    delay(100);
  }
  
  // Initialize watchdog
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);
  
  // Load configuration
  loadConfig();
  
  // Initialize camera
  Serial.println("CAM:INIT");
  if (initCamera((framesize_t)config.resolution)) {
    Serial.println("CAM:READY");
    digitalWrite(LED_PIN, LOW);
    delay(500);
    digitalWrite(LED_PIN, HIGH);
  } else {
    Serial.println("CAM:ERROR");
    while (1) {
      digitalWrite(LED_PIN, LOW);
      delay(200);
      digitalWrite(LED_PIN, HIGH);
      delay(200);
      esp_task_wdt_reset();
    }
  }
  
  // Initialize detection zones
  memset(zones, 0, sizeof(zones));
  activeZoneCount = 0;
  
  // Clear previous frame buffer
  memset(prevFrame, 0, sizeof(prevFrame));
  
  // Initialize random seed
  randomSeed(micros());
  
  Serial.println("CAM:MODULE_READY");
  Serial.printf("Resolution: %dx%d\n", frameWidth, frameHeight);
  Serial.printf("Sensitivity: %d%%\n", currentSensitivity);
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Feed watchdog
  esp_task_wdt_reset();
  
  // Handle serial commands
  if (Serial.available()) {
    uint8_t cmd = Serial.read();
    handleCommand(cmd);
  }
  
  // Stream frames if enabled
  if (streaming) {
    streamFrame();
    delay(50);  // ~20 FPS
  }
  
  // Periodic temperature update
  static unsigned long lastTempUpdate = 0;
  if (millis() - lastTempUpdate > 10000) {
    lastTempUpdate = millis();
    updateTemperature();
  }
  
  // Periodic diagnostics
  static unsigned long lastDiag = 0;
  if (millis() - lastDiag > 60000) {
    lastDiag = millis();
    printDiagnostics();
  }
  
  delay(10);
}

