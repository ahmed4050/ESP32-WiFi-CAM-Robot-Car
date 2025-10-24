/*
  Project: ESP32-CAM Robot Car
  Author: Keyestudio (Modified)
  Function: Control robot car via WiFi with camera streaming
  Speed levels: Low=85, Mid=170, High=255
*/

#include "esp_camera.h"
#include <WiFi.h>
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_http_server.h"
#include <WiFiManager.h>
#include "esp_system.h"
#include "esp_task_wdt.h"

// ======== WiFi Manager Configuration ========
WiFiManager wm;

// ======== Camera Pins (AI-THINKER) ========
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

// ======== Motor Control Pins ========
#define MOTOR_R_PIN_1 14
#define MOTOR_R_PIN_2 15
#define MOTOR_L_PIN_1 13
#define MOTOR_L_PIN_2 12

// ======== LED Pin ========
#define LED_GPIO_NUM 4

// ======== Motor Speed Variables ========
int MOTOR_R_Speed = 170;
int MOTOR_L_Speed = 170;

// ======== HTTP Server Handles ========
#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t camera_httpd = NULL;
httpd_handle_t stream_httpd = NULL;

void startCameraServer();

// ======== LED Flash Function ========
void blinkLED(int times = 3, int delayMs = 200) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_GPIO_NUM, HIGH);
    delay(delayMs);
    digitalWrite(LED_GPIO_NUM, LOW);
    delay(delayMs);
  }
}

// ======== WiFiManager Callbacks ========
void configModeCallback(WiFiManager *myWiFiManager) {
  Serial.println("\n========================================");
  Serial.println("      WiFi Configuration Mode");
  Serial.println("========================================");
  Serial.print("AP SSID: ");
  Serial.println(myWiFiManager->getConfigPortalSSID());
  Serial.println("AP Password: None (Open Network)");
  Serial.print("Config Portal IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.println("Captive Portal: ENABLED");
  Serial.println("Browser should open automatically!");
  Serial.println("========================================\n");
  
  // إشارة LED بسيطة - غير محجبة
  digitalWrite(LED_GPIO_NUM, HIGH);
}

void saveConfigCallback() {
  Serial.println("✅ WiFi Configuration Saved!");
  Serial.println("Connecting to network...");
  
  // إطفاء LED بعد الحفظ
  digitalWrite(LED_GPIO_NUM, LOW);
}

// ======== HTML Web Interface ========
static const char PROGMEM INDEX_HTML[] = R"rawliteral(
<html>
  <head>
    <title>ESP32-CAM Robot</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <meta charset="UTF-8"/>
    <link rel="icon" href="data:image/svg+xml,<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 100 100'><text y='0.9em' font-size='90'>🤖</text></svg>">
    <style>
      body {
        font-family: Arial;
        text-align: center;
        margin: 0 auto;
        padding-top: 20px;
      }
      .developer-info {
        background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
        color: white;
        padding: 15px;
        margin: -20px -8px 20px -8px;
        box-shadow: 0 2px 5px rgba(0,0,0,0.2);
      }
      .developer-info h2 {
        margin: 5px 0;
        font-size: 22px;
        text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
      }
      .developer-info p {
        margin: 3px 0;
        font-size: 14px;
        opacity: 0.95;
      }
      .developer-info .contact {
        font-size: 12px;
        margin-top: 8px;
      }
      .developer-info .icon {
        font-size: 40px;
        margin-bottom: 5px;
      }
      .button-container {
        display: grid;
        grid-template-areas:
          "keyes forward led"
          "left stop right"
          "plus backward minus";
        grid-gap: 10px;
        justify-content: center;
        align-content: center;
        margin-top: 20px;
      }
      .button {
        background-color: #2f4468;
        color: white;
        border: none;
        padding: 20px 0;
        text-align: center;
        font-size: 18px;
        cursor: pointer;
        width: 90px;
        height: 60px;
        border-radius: 15px;
      }
      .led-button {
        background-color: #777;
        color: white;
        border: none;
        padding: 20px 0;
        text-align: center;
        font-size: 18px;
        cursor: pointer;
        width: 90px;
        height: 60px;
        border-radius: 15px;
      }
      .led-on {
        background-color: #f0c40f;
        color: black;
      }
      .forward { grid-area: forward; }
      .led { grid-area: led; }
      .left { grid-area: left; }
      .stop { grid-area: stop; }
      .right { grid-area: right; }
      .backward { grid-area: backward; }
      .plus { grid-area: plus; }
      .minus { grid-area: minus; }
      .keyes { grid-area: keyes; }
      img {
        width: auto;
        max-width: 100%;
        height: auto;
        border: 2px solid #2f4468;
        border-radius: 10px;
        margin-top: 20px;
      }
      .settings-btn {
        background-color: #e74c3c;
        color: white;
        border: none;
        padding: 10px 20px;
        margin: 10px;
        font-size: 14px;
        cursor: pointer;
        border-radius: 5px;
      }
      .settings-btn:hover {
        background-color: #c0392b;
      }
      .info-box {
        background-color: #ecf0f1;
        padding: 10px;
        margin: 10px 0;
        border-radius: 5px;
        font-size: 14px;
      }
    </style>
  </head>
  <body>
    <div class="developer-info">
      <div class="icon">🤖</div>
      <h2>AHMED ALQASSABI</h2>
      <p class="contact">📱 +96899848382</p>
      <p class="contact">📧 ahmed4050@gmail.com</p>
    </div>
    <h1>ESP32-CAM Robot</h1>
    <div class="info-box">
      <strong>WiFi:</strong> <span id="wifiInfo">Loading...</span><br>
      <strong>IP:</strong> <span id="ipInfo">Loading...</span>
    </div>
    <button class="settings-btn" onclick="resetWiFi()">🔄 Reset WiFi</button>
    <button class="settings-btn" onclick="restartESP()">♻️ Restart ESP32</button>
    <img src="" id="photo">
    <div class="button-container">
      <button class="button forward" onmousedown="toggleCheckbox('forward');" ontouchstart="toggleCheckbox('forward');" onmouseup="toggleCheckbox('stop');" ontouchend="toggleCheckbox('stop');">↑</button>
      <button id="ledButton" class="led-button led" onclick="toggleLED()">OFF</button>
      <button class="button left" onmousedown="toggleCheckbox('left');" ontouchstart="toggleCheckbox('left');" onmouseup="toggleCheckbox('stop');" ontouchend="toggleCheckbox('stop');">←</button>
      <button class="button stop" onmousedown="toggleCheckbox('stop');">●</button>
      <button class="button right" onmousedown="toggleCheckbox('right');" ontouchstart="toggleCheckbox('right');" onmouseup="toggleCheckbox('stop');" ontouchend="toggleCheckbox('stop');">→</button>
      <button class="button backward" onmousedown="toggleCheckbox('backward');" ontouchstart="toggleCheckbox('backward');" onmouseup="toggleCheckbox('stop');" ontouchend="toggleCheckbox('stop');">↓</button>
      <button class="button plus" onmouseup="toggleCheckbox('plus');">+</button>
      <button class="button minus" onmouseup="toggleCheckbox('minus');">-</button>
      <button class="button keyes">Keyes</button>
    </div>
    <script>
      window.onload = function () {
        document.getElementById("photo").src = window.location.href.slice(0, -1) + ":81/stream";
        updateInfo();
      };
      function updateInfo() {
        fetch('/info')
          .then(response => response.json())
          .then(data => {
            document.getElementById("wifiInfo").textContent = data.ssid + " (" + data.rssi + " dBm)";
            document.getElementById("ipInfo").textContent = data.ip;
          })
          .catch(error => {
            document.getElementById("wifiInfo").textContent = "N/A";
            document.getElementById("ipInfo").textContent = "N/A";
          });
      }
      function toggleCheckbox(action) {
        var xhr = new XMLHttpRequest();
        xhr.open("GET", "/action?go=" + action, true);
        xhr.send();
      }
      function resetWiFi() {
        if (confirm("هل تريد إعادة ضبط إعدادات WiFi؟\n\nسيتم إعادة تشغيل ESP32 وفتح شبكة ESP32-CAM-Setup")) {
          fetch('/action?reset=wifi')
            .then(() => {
              alert("تم إعادة ضبط WiFi!\n\nابحث عن شبكة: ESP32-CAM-Setup");
            });
        }
      }
      function restartESP() {
        if (confirm("هل تريد إعادة تشغيل ESP32؟")) {
          fetch('/action?reset=restart')
            .then(() => {
              alert("جاري إعادة التشغيل...\n\nانتظر 10 ثواني ثم حدّث الصفحة");
            });
        }
      }
      let ledState = false;
      const ledButton = document.getElementById("ledButton");
      function toggleLED() {
        ledState = !ledState;
        if (ledState) {
          ledButton.classList.add("led-on");
          ledButton.textContent = "ON";
        } else {
          ledButton.classList.remove("led-on");
          ledButton.textContent = "OFF";
        }
        var xhr = new XMLHttpRequest();
        xhr.open("GET", "/action?led=" + (ledState ? "on" : "off"), true);
        xhr.send();
      }
    </script>
  </body>
</html>
)rawliteral";

// ======== HTTP Handlers ========
static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, (const char *)INDEX_HTML, strlen(INDEX_HTML));
}

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    } else {
      if (fb->width > 400) {
        if (fb->format != PIXFORMAT_JPEG) {
          bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
          esp_camera_fb_return(fb);
          fb = NULL;
          if (!jpeg_converted) {
            Serial.println("JPEG compression failed");
            res = ESP_FAIL;
          }
        } else {
          _jpg_buf_len = fb->len;
          _jpg_buf = fb->buf;
        }
      }
    }
    if (res == ESP_OK) {
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if (_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if (res != ESP_OK) {
      break;
    }
  }
  return res;
}

static esp_err_t action_handler(httpd_req_t *req) {
  char query[100];
  int len = httpd_req_get_url_query_len(req) + 1;
  if (len > sizeof(query)) {
    httpd_resp_send_404(req);
    return ESP_OK;
  }

  if (httpd_req_get_url_query_str(req, query, len) == ESP_OK) {
    // WiFi Reset Command
    if (strstr(query, "reset=wifi")) {
      Serial.println("🔄 WiFi Reset requested via Web!");
      httpd_resp_set_type(req, "text/plain");
      httpd_resp_send(req, "Resetting WiFi...", HTTPD_RESP_USE_STRLEN);
      delay(500);
      wm.resetSettings();
      delay(1000);
      ESP.restart();
      return ESP_OK;
    }
    // ESP32 Restart Command
    else if (strstr(query, "reset=restart")) {
      Serial.println("♻️ ESP32 Restart requested via Web!");
      httpd_resp_set_type(req, "text/plain");
      httpd_resp_send(req, "Restarting...", HTTPD_RESP_USE_STRLEN);
      delay(500);
      ESP.restart();
      return ESP_OK;
    }
    // Motor Controls
    else if (strstr(query, "go=forward")) {
      Serial.println("Forward");
      analogWrite(MOTOR_R_PIN_1, 0);
      analogWrite(MOTOR_R_PIN_2, MOTOR_R_Speed);
      analogWrite(MOTOR_L_PIN_1, MOTOR_L_Speed);
      analogWrite(MOTOR_L_PIN_2, 0);
    } else if (strstr(query, "go=backward")) {
      Serial.println("Backward");
      analogWrite(MOTOR_R_PIN_1, MOTOR_R_Speed);
      analogWrite(MOTOR_R_PIN_2, 0);
      analogWrite(MOTOR_L_PIN_1, 0);
      analogWrite(MOTOR_L_PIN_2, MOTOR_L_Speed);
    } else if (strstr(query, "go=left")) {
      Serial.println("Left");
      analogWrite(MOTOR_R_PIN_1, 0);
      analogWrite(MOTOR_R_PIN_2, MOTOR_R_Speed);
      analogWrite(MOTOR_L_PIN_1, 0);
      analogWrite(MOTOR_L_PIN_2, MOTOR_L_Speed);
    } else if (strstr(query, "go=right")) {
      Serial.println("Right");
      analogWrite(MOTOR_R_PIN_1, MOTOR_R_Speed);
      analogWrite(MOTOR_R_PIN_2, 0);
      analogWrite(MOTOR_L_PIN_1, MOTOR_L_Speed);
      analogWrite(MOTOR_L_PIN_2, 0);
    } else if (strstr(query, "go=stop")) {
      Serial.println("Stop");
      analogWrite(MOTOR_R_PIN_1, 0);
      analogWrite(MOTOR_R_PIN_2, 0);
      analogWrite(MOTOR_L_PIN_1, 0);
      analogWrite(MOTOR_L_PIN_2, 0);
    } else if (strstr(query, "led=on")) {
      Serial.println("LED ON");
      digitalWrite(LED_GPIO_NUM, HIGH);
    } else if (strstr(query, "led=off")) {
      Serial.println("LED OFF");
      digitalWrite(LED_GPIO_NUM, LOW);
    } else if (strstr(query, "go=plus")) {
      MOTOR_R_Speed = MOTOR_R_Speed + 85;
      MOTOR_L_Speed = MOTOR_L_Speed + 85;
      if (MOTOR_L_Speed >= 255) MOTOR_L_Speed = 255;
      if (MOTOR_R_Speed >= 255) MOTOR_R_Speed = 255;
      Serial.print("Speed UP - L:");
      Serial.print(MOTOR_L_Speed);
      Serial.print(" R:");
      Serial.println(MOTOR_R_Speed);
    } else if (strstr(query, "go=minus")) {
      MOTOR_R_Speed = MOTOR_R_Speed - 85;
      MOTOR_L_Speed = MOTOR_L_Speed - 85;
      if (MOTOR_L_Speed <= 85) MOTOR_L_Speed = 85;
      if (MOTOR_R_Speed <= 85) MOTOR_R_Speed = 85;
      Serial.print("Speed DOWN - L:");
      Serial.print(MOTOR_L_Speed);
      Serial.print(" R:");
      Serial.println(MOTOR_R_Speed);
    }
  }

  httpd_resp_send(req, "", HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

// Handler للحصول على معلومات النظام (JSON)
static esp_err_t info_handler(httpd_req_t *req) {
  char json[400];  // زيادة الحجم لتجنب buffer overflow
  snprintf(json, sizeof(json), 
    "{\"ssid\":\"%s\",\"ip\":\"%s\",\"rssi\":%d,\"heap\":%d}",
    WiFi.SSID().c_str(),
    WiFi.localIP().toString().c_str(),
    WiFi.RSSI(),
    ESP.getFreeHeap()
  );
  
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  
  httpd_uri_t index_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = index_handler,
    .user_ctx = NULL
  };

  httpd_uri_t cmd_uri = {
    .uri = "/action",
    .method = HTTP_GET,
    .handler = action_handler,
    .user_ctx = NULL
  };
  
  httpd_uri_t info_uri = {
    .uri = "/info",
    .method = HTTP_GET,
    .handler = info_handler,
    .user_ctx = NULL
  };
  
  httpd_uri_t stream_uri = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = stream_handler,
    .user_ctx = NULL
  };
  
  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &cmd_uri);
    httpd_register_uri_handler(camera_httpd, &info_uri);
  }
  
  config.server_port += 1;
  config.ctrl_port += 1;
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}

// ======== Setup Function ========
void setup() {
  // تعطيل Brown-out detector بشكل أقوى
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  
  // تأخير للاستقرار
  delay(1000);

  // Initialize Serial
  Serial.begin(115200);
  Serial.setDebugOutput(false);
  Serial.println("\n=== ESP32-CAM Robot Starting ===");
  
  // طباعة سبب إعادة التشغيل
  esp_reset_reason_t reset_reason = esp_reset_reason();
  Serial.print("Reset reason: ");
  switch(reset_reason) {
    case ESP_RST_POWERON: Serial.println("Power-on reset"); break;
    case ESP_RST_EXT: Serial.println("External reset"); break;
    case ESP_RST_SW: Serial.println("Software reset"); break;
    case ESP_RST_PANIC: Serial.println("Exception/panic reset"); break;
    case ESP_RST_INT_WDT: Serial.println("Interrupt watchdog reset"); break;
    case ESP_RST_TASK_WDT: Serial.println("Task watchdog reset"); break;
    case ESP_RST_WDT: Serial.println("Other watchdog reset"); break;
    case ESP_RST_DEEPSLEEP: Serial.println("Deep sleep reset"); break;
    case ESP_RST_BROWNOUT: Serial.println("Brownout reset"); break;
    case ESP_RST_SDIO: Serial.println("SDIO reset"); break;
    default: Serial.println("Unknown reset"); break;
  }

  // Initialize Motor Pins
  pinMode(MOTOR_R_PIN_1, OUTPUT);
  pinMode(MOTOR_R_PIN_2, OUTPUT);
  pinMode(MOTOR_L_PIN_1, OUTPUT);
  pinMode(MOTOR_L_PIN_2, OUTPUT);
  
  // Initialize LED Pin
  pinMode(LED_GPIO_NUM, OUTPUT);
  digitalWrite(LED_GPIO_NUM, LOW);  // LED OFF initially

  // Configure Camera
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
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
    Serial.println("PSRAM found");
  } else {
    config.frame_size = FRAMESIZE_HVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
    Serial.println("PSRAM not found");
  }

  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return;
  }
  Serial.println("Camera initialized successfully");

  // Wi-Fi connection using WiFiManager
  Serial.println("Starting WiFi Manager...");
  
  // إعداد WiFi Manager مع تحسينات محسّنة
  wm.setAPCallback(configModeCallback);
  wm.setSaveConfigCallback(saveConfigCallback);
  wm.setConfigPortalTimeout(180); // 3 دقائق timeout (معقول أكثر)
  wm.setConnectTimeout(30); // 30 ثانية للاتصال (أفضل للشبكات البطيئة)
  wm.setDebugOutput(true);
  
  // إعدادات Captive Portal - لفتح المتصفح تلقائياً
  wm.setAPStaticIPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
  wm.setCaptivePortalEnable(true); // تفعيل Captive Portal
  
  // إعدادات إضافية للاستقرار
  wm.setConnectRetries(3); // محاولة الاتصال 3 مرات
  wm.setCleanConnect(true); // قطع الاتصال قبل المحاولة الجديدة
  wm.setRemoveDuplicateAPs(true); // إزالة الشبكات المكررة من القائمة
  wm.setMinimumSignalQuality(10); // قبول الشبكات بإشارة ≥ 10%
  wm.setSaveConnectTimeout(10000); // 10 ثواني للحفظ والاتصال
  wm.setShowInfoUpdate(true); // إظهار معلومات التحديث
  wm.setShowInfoErase(false); // إخفاء زر المسح
  
  // تحسين صفحة البوابة
  wm.setTitle("ESP32-CAM Robot Setup"); // عنوان الصفحة
  wm.setHostname("esp32cam-robot"); // اسم الجهاز
  
  // محاولة الاتصال التلقائي، أو فتح Portal للإعداد
  Serial.println("Attempting WiFi connection...");
  if (!wm.autoConnect("ESP32-CAM-Setup")) {
    Serial.println("Failed to connect and hit timeout");
    
    // محاولة أخيرة قبل إعادة التشغيل
    Serial.println("Resetting WiFi settings and trying again...");
    wm.resetSettings();
    delay(2000);
    ESP.restart();
  }
  
  Serial.println("\n✅ WiFi connected successfully!");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.print("📡 SSID: ");
  Serial.println(WiFi.SSID());
  Serial.print("📶 Signal Strength: ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
  Serial.print("🌐 IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("🎥 Camera Stream: http://");
  Serial.println(WiFi.localIP());
  Serial.print("🎮 Control Panel: http://");
  Serial.println(WiFi.localIP());
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");

  // 📸 فلاش الكاميرا (3 مرات) للتنبيه من الاتصال الناجح بالشبكة
  Serial.println("📸 LED Flash - Connection Success Alert!");
  blinkLED(3, 200);  // 3 مرات وميض مع تأخير 200ms

  // إشارة نجاح - LED ثابت
  digitalWrite(LED_GPIO_NUM, LOW);

  // Start streaming web server
  startCameraServer();
  Serial.println("HTTP server started");
  
  // طباعة معلومات الذاكرة
  Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
  Serial.printf("Min free heap: %d bytes\n", ESP.getMinFreeHeap());
  Serial.printf("Max alloc heap: %d bytes\n", ESP.getMaxAllocHeap());
}

// ======== Loop Function ========
void loop() {
  // متغير لمراقبة الذاكرة
  static unsigned long lastMemCheck = 0;
  
  // فحص الذاكرة كل 10 ثواني
  if (millis() - lastMemCheck > 10000) {
    lastMemCheck = millis();
    uint32_t freeHeap = ESP.getFreeHeap();
    
    Serial.printf("Free heap: %d bytes\n", freeHeap);
    
    // تحذير إذا كانت الذاكرة قليلة
    if (freeHeap < 50000) {  // أقل من 50KB
      Serial.println("⚠️ WARNING: Low memory detected!");
    }
  }
  
  // Server runs in background - no blocking needed
  delay(100);
}