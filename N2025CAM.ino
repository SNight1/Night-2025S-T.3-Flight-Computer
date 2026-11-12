#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"
#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
WebServer server(80);

//AI-THINKER ESP32-CAM PINOUT 
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
#define FLASH_LED_PIN      4

//LOGGING & TIMING VARS (1Hz)
unsigned long previousCaptureMillis = 0;
const unsigned long captureInterval = 1000; // ถ่ายภาพทุก 1 วินาที (1Hz)
uint32_t frameCounter = 0;
String flightFolder = "";
bool sdCardReady = false;
bool isRecording = true;

//WEB SERVER HANDLER
void handleCapturePreview() {
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    server.send(500, "text/plain", "Camera Capture Failed");
    return;
  }
  server.sendHeader("Content-Disposition", "inline; filename=preview.jpg");
  server.send_P(200, "image/jpeg", (const char *)fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

void handleRoot() {
  String html = "<html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>Night 2025S,T.3 Ground Camera (1Hz)</title></head>";
  html += "<body style='background:#111;color:#fff;text-align:center;font-family:sans-serif;'>";
  html += "<h2> Night 2025S,T.3 — Payload Camera (1Hz)</h2>";
  html += "<img id='cam' src='/preview' style='width:90%;max-width:640px;border:2px solid #0f0;'/>";
  html += "<p>Frames Captured: <span id='cnt'>" + String(frameCounter) + "</span></p>";
  html += "<script>setInterval(()=>{document.getElementById('cam').src='/preview?t='+Date.now();}, 1000);</script>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

//CAMERA & SD INITIALIZATION
bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  // ปรับ XCLK 10MHz + GRAB_LATEST แก้ปัญหา FB-OVF
  config.xclk_freq_hz = 10000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_SVGA; // 800x600
    config.jpeg_quality = 12;
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    config.frame_size = FRAMESIZE_VGA;  // 640x480
    config.jpeg_quality = 15;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  return (esp_camera_init(&config) == ESP_OK);
}

bool initSDCard() {
  if (!SD_MMC.begin("/sdcard", true)) return false;
  if (SD_MMC.cardType() == CARD_NONE) return false;

  int flightNum = 1;
  while (true) {
    flightFolder = "/FLIGHT_" + String(flightNum);
    if (!SD_MMC.exists(flightFolder)) {
      SD_MMC.mkdir(flightFolder);
      break;
    }
    flightNum++;
  }
  return true;
}

//CAPTURE FRAME 1 Hz
void captureFrame(unsigned long timestampMs) {
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println(F("[ERR] Camera capture failed"));
    return;
  }

  frameCounter++;

  if (sdCardReady) {
    String path = flightFolder + "/IMG_" + String(timestampMs) + ".jpg";
    File file = SD_MMC.open(path.c_str(), FILE_WRITE);
    if (file) {
      file.write(fb->buf, fb->len);
      file.close();
      Serial.print(F("[SD] Saved: ")); Serial.println(path);
    } else {
      Serial.println(F("[ERR] Failed to write frame to SD"));
    }
  } else {
    // กรณียังไม่มี SD Card จะแสดงสถานะการจับภาพ 1Hz บน Serial Monitor แทน
    Serial.print(F("[1Hz] Frame #")); Serial.print(frameCounter);
    Serial.print(F(" | Size: ")); Serial.print(fb->len);
    Serial.print(F(" bytes | T+")); Serial.print(timestampMs); Serial.println(F(" ms"));
  }

  esp_camera_fb_return(fb); // คืนหน่วยความจำ Buffer ทันที
}

void setup() {
  Serial.begin(115200);
  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);

  Serial.println(F("\nBOOT Initializing ESP32-CAM (1Hz Mode)..."));

  if (!initCamera()) {
    Serial.println(F("Camera Init Failed!"));
    while (true) delay(1000);
  }
  Serial.println(F("Camera Ready"));

  if (initSDCard()) {
    sdCardReady = true;
    Serial.print(F("[OK] SD Card Ready. Logging to: "));
    Serial.println(flightFolder);
  } else {
    sdCardReady = false;
    Serial.println(F("No SD Card detected Running 1Hz Live Preview Mode"));
  }

  WiFi.softAP(ssid, password);
  Serial.print(F("Connect Wi-Fi & Open: http://"));
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/preview", handleCapturePreview);
  server.begin();
}

void loop() {
  server.handleClient();

  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 's' || cmd == 'S') {
      isRecording = true;
      Serial.println(F("1Hz Capture Started"));
    } else if (cmd == 'x' || cmd == 'X') {
      isRecording = false;
      Serial.println(F("1Hz Capture Stopped"));
    }
  }

  unsigned long currentMillis = millis();
  if (isRecording && (currentMillis - previousCaptureMillis >= captureInterval)) {
    previousCaptureMillis = currentMillis;
    captureFrame(currentMillis);
  }
}