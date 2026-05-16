/*
  Project: ESP32-CAM Robot Car
  Author: Ahmed Alqassabi
  Features: Strict Validation, FreeRTOS Restart Task, WebSocket (Port 82), Visual Keyboard UI
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
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// ======== WiFi Manager ========
WiFiManager wm;

// ======== Camera Pins (AI-THINKER) ========
#define PWDN_GPIO_NUM   32
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM    0
#define SIOD_GPIO_NUM   26
#define SIOC_GPIO_NUM   27
#define Y9_GPIO_NUM     35
#define Y8_GPIO_NUM     34
#define Y7_GPIO_NUM     39
#define Y6_GPIO_NUM     36
#define Y5_GPIO_NUM     21
#define Y4_GPIO_NUM     19
#define Y3_GPIO_NUM     18
#define Y2_GPIO_NUM      5
#define VSYNC_GPIO_NUM  25
#define HREF_GPIO_NUM   23
#define PCLK_GPIO_NUM   22

// ======== Motor Pins ========
#define MOTOR_R_PIN_1 14
#define MOTOR_R_PIN_2 15
#define MOTOR_L_PIN_1 13
#define MOTOR_L_PIN_2 12

// ======== LED Pin ========
#define LED_GPIO_NUM 4

// ======== Motor Speed ========
int MOTOR_R_Speed = 100;
int MOTOR_L_Speed = 100;

// ======== Servers ========
#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t camera_httpd = NULL;
httpd_handle_t stream_httpd = NULL;

AsyncWebServer wsServer(82);
AsyncWebSocket ws("/ws");

void startCameraServer();
void startWebSocketServer();

// ======== FreeRTOS Restart Task ========
struct RestartTaskParams {
  bool resetWifi;
};

static void restartTask(void *pvParameters) {
  RestartTaskParams *params = (RestartTaskParams *)pvParameters;
  bool resetWifi = params->resetWifi;
  delete params;

  vTaskDelay(pdMS_TO_TICKS(600)); // انتظر حتى يصل الرد للمتصفح

  if (resetWifi) {
    Serial.println("🔄 Resetting WiFi settings...");
    wm.resetSettings();
    vTaskDelay(pdMS_TO_TICKS(300));
  }

  Serial.println("♻️ Restarting ESP32...");
  ESP.restart();
  vTaskDelete(NULL);
}

static void scheduleRestart(bool resetWifi = false) {
  RestartTaskParams *params = new RestartTaskParams{resetWifi};
  xTaskCreate(restartTask, "restart_task", 2048, params, 1, NULL);
}

// ======== Strict Command Validation ========
static bool query_equals(const char *query, const char *key, const char *value) {
  char expected[64];
  snprintf(expected, sizeof(expected), "%s=%s", key, value);
  return strcmp(query, expected) == 0;
}

// ======== Motor Control Logic (Shared) ========
void executeCommand(const char *cmd) {
  if (strcmp(cmd, "forward") == 0) {
    analogWrite(MOTOR_R_PIN_1, 0); analogWrite(MOTOR_R_PIN_2, MOTOR_R_Speed);
    analogWrite(MOTOR_L_PIN_1, MOTOR_L_Speed); analogWrite(MOTOR_L_PIN_2, 0);
  } else if (strcmp(cmd, "backward") == 0) {
    analogWrite(MOTOR_R_PIN_1, MOTOR_R_Speed); analogWrite(MOTOR_R_PIN_2, 0);
    analogWrite(MOTOR_L_PIN_1, 0); analogWrite(MOTOR_L_PIN_2, MOTOR_L_Speed);
  } else if (strcmp(cmd, "left") == 0) {
    analogWrite(MOTOR_R_PIN_1, 0); analogWrite(MOTOR_R_PIN_2, MOTOR_R_Speed);
    analogWrite(MOTOR_L_PIN_1, 0); analogWrite(MOTOR_L_PIN_2, MOTOR_L_Speed);
  } else if (strcmp(cmd, "right") == 0) {
    analogWrite(MOTOR_R_PIN_1, MOTOR_R_Speed); analogWrite(MOTOR_R_PIN_2, 0);
    analogWrite(MOTOR_L_PIN_1, MOTOR_L_Speed); analogWrite(MOTOR_L_PIN_2, 0);
  } else if (strcmp(cmd, "stop") == 0) {
    analogWrite(MOTOR_R_PIN_1, 0); analogWrite(MOTOR_R_PIN_2, 0);
    analogWrite(MOTOR_L_PIN_1, 0); analogWrite(MOTOR_L_PIN_2, 0);
  } else if (strcmp(cmd, "led=on") == 0) {
    digitalWrite(LED_GPIO_NUM, HIGH);
  } else if (strcmp(cmd, "led=off") == 0) {
    digitalWrite(LED_GPIO_NUM, LOW);
  } else if (strcmp(cmd, "plus") == 0) {
    MOTOR_R_Speed = min(MOTOR_R_Speed + 85, 255);
    MOTOR_L_Speed = min(MOTOR_L_Speed + 85, 255);
  } else if (strcmp(cmd, "minus") == 0) {
    MOTOR_R_Speed = max(MOTOR_R_Speed - 85, 85);
    MOTOR_L_Speed = max(MOTOR_L_Speed - 85, 85);
  }
}

// ======== HTML / UI ========
static const char PROGMEM INDEX_HTML[] = R"rawliteral(
<html>
  <head>
    <title>ESP32-CAM Robot</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <meta charset="UTF-8"/>
    <style>
      * { box-sizing: border-box; margin: 0; padding: 0; }
      body { font-family: Arial; background: #111; color: #eee; display: flex; flex-direction: column; align-items: center; padding-bottom: 20px;}

      .header { width: 100%; background: linear-gradient(135deg, #667eea, #764ba2); padding: 12px; text-align: center; }
      .header h2 { font-size: 18px; margin-bottom: 4px;}
      .header p  { font-size: 12px; opacity: .85; }

      .info-box { background: #1e1e1e; border: 1px solid #333; border-radius: 8px; padding: 8px 16px; font-size: 13px; margin: 10px 0; text-align: center; width: 92%; max-width: 480px;}
      #wsStatus.ok { color: #1D9E75; }
      #wsStatus.err { color: #E24B4A; }

      img#photo { width: 92%; max-width: 480px; border-radius: 10px; border: 2px solid #2f4468; margin: 10px 0; }

      #joy-container { display: flex; flex-direction: column; align-items: center; gap: 12px; margin: 10px 0; width: 100%; }
      canvas#joy { touch-action: none; cursor: grab; }
      #cmd-label { font-size: 15px; font-weight: bold; background: #1e1e1e; border: 1px solid #444; border-radius: 20px; padding: 6px 24px; min-width: 140px; text-align: center; transition: background .15s, color .15s; }
      #cmd-label.active { background: #185FA5; color: #fff; }

      .stats { display: flex; gap: 10px; }
      .stat-box { background: #1e1e1e; border: 1px solid #333; border-radius: 8px; padding: 8px 16px; text-align: center; }
      .stat-box small { font-size: 11px; color: #888; display: block; }
      .stat-box span  { font-size: 17px; font-family: monospace; }

      /* Visual Keyboard UI */
      #keyHint { margin: 12px auto; background: #1e1e1e; border: 1px solid #333; border-radius: 10px; padding: 12px 20px; display: inline-block; text-align: center; width: 92%; max-width: 480px;}
      .key { display: inline-flex; align-items: center; justify-content: center; width: 38px; height: 38px; background: #2a2a2a; border: 1px solid #444; border-bottom: 3px solid #333; border-radius: 6px; font-family: monospace; font-size: 13px; color: #ccc; transition: all .1s; user-select: none; }
      .key.active { background: #185FA5; border-color: #378ADD; border-bottom-color: #0C447C; color: #fff; transform: translateY(2px); }
      .stop-key { background: #2a1a1a; border-color: #663333; color: #cc6666; }
      .stop-key.active { background: #7f1d1d; border-color: #E24B4A; color: #fff; }
      .small-key { width: 30px; height: 30px; font-size: 12px; }

      .controls { display: flex; gap: 10px; flex-wrap: wrap; justify-content: center; margin: 10px 0; }
      .ctrl-btn { background: #1e1e1e; color: #eee; border: 1px solid #444; border-radius: 8px; padding: 10px 18px; font-size: 14px; cursor: pointer; }
      .ctrl-btn:active { opacity: .7; }
      .ctrl-btn.led-on  { background: #BA7517; border-color: #BA7517; color: #FAEEDA; }
      .ctrl-btn.danger  { border-color: #e74c3c; color: #e74c3c; }
    </style>
  </head>
  <body>
    <div class="header">
      <h2>🤖 AHMED ALQASSABI</h2>
      <p>📱 +96899848382 &nbsp;|&nbsp; 📧 ahmed4050@gmail.com</p>
    </div>

    <div class="info-box">
      <strong>WS:</strong> <span id="wsStatus" style="color:#888">جاري الاتصال...</span>
      &nbsp;|&nbsp;
      <strong>IP:</strong> <span id="ipInfo">—</span>
      &nbsp;|&nbsp;
      <strong>RAM:</strong> <span id="heapInfo">—</span>
    </div>

    <img src="" id="photo" alt="camera">

    <div id="joy-container">
      <div id="cmd-label">ضع إصبعك هنا</div>
      <canvas id="joy" width="240" height="240"></canvas>
      <div class="stats">
        <div class="stat-box"><small>سرعة</small><span id="vspd">0%</span></div>
        <div class="stat-box"><small>اتجاه</small><span id="vdir">—</span></div>
      </div>
    </div>

    <div id="keyHint">
      <div style="font-size:11px; color:#888; margin-bottom:8px;">تحكم بلوحة المفاتيح</div>
      <div style="display:flex; gap:6px; justify-content:center; margin-bottom:6px;">
        <div style="width:10px"></div><kbd class="key" id="k-w">W</kbd><kbd class="key" id="k-up">↑</kbd><div style="width:10px"></div>
      </div>
      <div style="display:flex; gap:6px; justify-content:center; margin-bottom:6px;">
        <kbd class="key" id="k-a">A</kbd><kbd class="key stop-key" id="k-space">SPC</kbd><kbd class="key" id="k-d">D</kbd><kbd class="key" id="k-right">→</kbd>
      </div>
      <div style="display:flex; gap:6px; justify-content:center; margin-bottom:6px;">
        <kbd class="key" id="k-left">←</kbd><kbd class="key" id="k-s">S</kbd><kbd class="key" id="k-down">↓</kbd>
      </div>
      <div style="display:flex; gap:6px; justify-content:center; margin-top:4px;">
        <kbd class="key small-key" id="k-plus">+</kbd><kbd class="key small-key" id="k-minus">−</kbd><kbd class="key small-key" id="k-l">L</kbd>
      </div>
      <div style="font-size:10px; color:#555; margin-top:8px;">+/− سرعة &nbsp;|&nbsp; L إضاءة</div>
      <div id="keyIndicator" style="font-size:22px; margin-top:8px; color:#378ADD; min-height:30px;">●</div>
    </div>

    <div class="controls">
      <button id="ledBtn" class="ctrl-btn" onclick="toggleLED()">💡 LED: OFF</button>
      <button class="ctrl-btn danger" onclick="resetWiFi()">🔄 Reset WiFi</button>
      <button class="ctrl-btn danger" onclick="restartESP()">♻️ Restart</button>
    </div>

    <script>
      // ======== WebSocket ========
      var ws, wsReady = false, wsQueue = [];
      var lastCmd = '', lastSent = 0;

      function initWS() {
        var wsUrl = "ws://" + window.location.hostname + ":82/ws";
        ws = new WebSocket(wsUrl);

        ws.onopen = function() {
          wsReady = true;
          var st = document.getElementById("wsStatus");
          st.textContent = "متصل"; st.className = "ok";
          wsQueue.forEach(function(cmd) { ws.send(cmd); });
          wsQueue = [];
        };

        ws.onclose = function() {
          wsReady = false;
          var st = document.getElementById("wsStatus");
          st.textContent = "منقطع — يعاود الاتصال..."; st.className = "err";
          setTimeout(initWS, 2000);
        };

        ws.onerror = function() { ws.close(); };

        ws.onmessage = function(e) {
          try {
            var d = JSON.parse(e.data);
            if (d.heap) document.getElementById("heapInfo").textContent = Math.round(d.heap / 1024) + " KB";
            if (d.ip) document.getElementById("ipInfo").textContent = d.ip;
          } catch(err) {}
        };
      }

      function sendCmd(cmd) {
        var now = Date.now();
        if (cmd === lastCmd && now - lastSent < 80) return;
        lastCmd = cmd; lastSent = now;
        if (wsReady) ws.send(cmd);
        else wsQueue.push(cmd);
      }

      // ======== Init ========
      window.onload = function() {
        document.getElementById("photo").src = window.location.href.slice(0,-1) + ":81/stream";
        initWS();
        initJoystick();
      };

      // ======== Buttons ========
      var ledOn = false;
      function toggleLED() {
        ledOn = !ledOn;
        var btn = document.getElementById("ledBtn");
        btn.textContent = ledOn ? "💡 LED: ON" : "💡 LED: OFF";
        btn.classList.toggle("led-on", ledOn);
        sendCmd(ledOn ? 'led=on' : 'led=off');
        updateKeyIndicator(ledOn ? 'led=on' : 'led=off');
      }

      function resetWiFi() {
        if (confirm("إعادة ضبط WiFi؟\n\nسيفتح الجهاز شبكة ESP32-CAM-Setup")) {
          fetch("/action?reset=wifi").then(() => alert("ابحث عن شبكة: ESP32-CAM-Setup"));
        }
      }
      function restartESP() {
        if (confirm("إعادة تشغيل الجهاز؟")) {
          fetch("/action?reset=restart").then(() => alert("انتظر 10 ثواني ثم حدّث الصفحة"));
        }
      }

      // ======== Keyboard Logic ========
      var pressedKeys = {};
      var keyMap = {
        'ArrowUp':'forward', 'ArrowDown':'backward', 'ArrowLeft':'left', 'ArrowRight':'right',
        'w':'forward', 'W':'forward', 's':'backward', 'S':'backward', 'a':'left', 'A':'left', 'd':'right', 'D':'right',
        ' ':'stop', 'l':'led=on', 'L':'led=on', '+':'plus', '=':'plus', '-':'minus'
      };

      var keyToId = {
        'forward': ['k-w', 'k-up'], 'backward': ['k-s', 'k-down'], 'left': ['k-a', 'k-left'], 'right': ['k-d', 'k-right'],
        'stop': ['k-space'], 'plus': ['k-plus'], 'minus': ['k-minus'], 'led=on': ['k-l'], 'led=off': ['k-l']
      };
      var activeKeyIds = [];

      function updateKeyIndicator(cmd) {
        activeKeyIds.forEach(function(id) {
          var el = document.getElementById(id);
          if (el) el.classList.remove('active');
        });
        activeKeyIds = keyToId[cmd] || [];
        activeKeyIds.forEach(function(id) {
          var el = document.getElementById(id);
          if (el) el.classList.add('active');
        });
        var labels = { forward:'↑', backward:'↓', left:'←', right:'→', stop:'●' };
        var indicator = document.getElementById('keyIndicator');
        if (indicator) indicator.textContent = labels[cmd] || '';
      }

      document.addEventListener('keydown', function(e) {
        if (e.repeat) return;
        var cmd = keyMap[e.key];
        if (!cmd) return;
        e.preventDefault();
        pressedKeys[e.key] = true;

        if (cmd === 'led=on') {
          toggleLED(); return;
        }
        if (cmd === 'plus' || cmd === 'minus') {
           sendCmd(cmd); updateKeyIndicator(cmd); return;
        }
        sendCmd(cmd);
        updateKeyIndicator(cmd);
      });

      document.addEventListener('keyup', function(e) {
        var cmd = keyMap[e.key];
        if (!cmd) return;
        delete pressedKeys[e.key];
        var movementKeys = ['ArrowUp','ArrowDown','ArrowLeft','ArrowRight','w','s','a','d','W','S','A','D'];
        var stillMoving = movementKeys.some(function(k) { return pressedKeys[k]; });
        if (!stillMoving) {
          sendCmd('stop');
          updateKeyIndicator('stop');
        }
      });

      // ======== Joystick Logic ========
      function initJoystick() {
        var canvas = document.getElementById("joy"), ctx = canvas.getContext("2d");
        var W = canvas.width, H = canvas.height, cx = W/2, cy = H/2;
        var outerR = 100, innerR = 36, kx = 0, ky = 0, dragging = false;
        var cmdLabels = { forward:"للأمام ↑", backward:"للخلف ↓", left:"يسار ←", right:"يمين →", stop:"توقف" };

        function getCmd(x, y) {
          var d = Math.sqrt(x*x + y*y);
          if (d < 0.25) return "stop";
          var a = Math.atan2(y, x) * 180 / Math.PI;
          if (a > -135 && a <= -45) return "forward";
          if (a > 45 && a <= 135) return "backward";
          if (a > -45 && a <= 45) return "right";
          return "left";
        }

        function draw() {
          ctx.clearRect(0,0,W,H);
          ctx.beginPath(); ctx.arc(cx,cy,outerR,0,Math.PI*2); ctx.fillStyle = "#1e1e1e"; ctx.fill();
          ctx.strokeStyle = "#444"; ctx.lineWidth = 1.5; ctx.stroke();
          ctx.strokeStyle = "#333"; ctx.lineWidth = 0.5;
          ctx.beginPath(); ctx.moveTo(cx-outerR+12,cy); ctx.lineTo(cx+outerR-12,cy); ctx.stroke();
          ctx.beginPath(); ctx.moveTo(cx,cy-outerR+12); ctx.lineTo(cx,cy+outerR-12); ctx.stroke();
          var px = cx + kx*outerR, py = cy + ky*outerR;
          ctx.strokeStyle = "rgba(55,138,221,0.4)"; ctx.lineWidth = 2;
          ctx.beginPath(); ctx.moveTo(cx,cy); ctx.lineTo(px,py); ctx.stroke();
          ctx.beginPath(); ctx.arc(px,py,innerR,0,Math.PI*2);
          ctx.fillStyle = dragging ? "#185FA5" : "#2a2a2a"; ctx.fill();
          ctx.strokeStyle = "#378ADD"; ctx.lineWidth = 2; ctx.stroke();
        }

        function update(ex, ey) {
          var rect = canvas.getBoundingClientRect();
          var dx = (ex - rect.left - cx) / outerR, dy = (ey - rect.top - cy) / outerR;
          var dist = Math.sqrt(dx*dx + dy*dy);
          if (dist > 1) { kx = dx/dist; ky = dy/dist; } else { kx = dx; ky = dy; }
          var cmd = getCmd(kx, ky);
          document.getElementById("vspd").textContent = Math.round(Math.min(1, dist) * 100) + "%";
          document.getElementById("vdir").textContent = cmdLabels[cmd] || "—";
          var lbl = document.getElementById("cmd-label");
          lbl.textContent = cmdLabels[cmd]; lbl.classList.toggle("active", cmd !== "stop");
          if (cmd !== lastCmd) { lastCmd = cmd; sendCmd(cmd); }
          draw();
        }

        function release() {
          if (!dragging) return;
          dragging = false; kx = 0; ky = 0; lastCmd = '';
          document.getElementById("vspd").textContent = "0%"; document.getElementById("vdir").textContent = "—";
          document.getElementById("cmd-label").textContent = "ضع إصبعك هنا"; document.getElementById("cmd-label").classList.remove("active");
          sendCmd("stop"); draw();
        }

        canvas.addEventListener("mousedown", function(e){ dragging=true; update(e.clientX,e.clientY); });
        canvas.addEventListener("mousemove", function(e){ if(dragging) update(e.clientX,e.clientY); });
        canvas.addEventListener("mouseup", release); canvas.addEventListener("mouseleave", release);
        canvas.addEventListener("touchstart", function(e){ e.preventDefault(); dragging=true; update(e.touches[0].clientX,e.touches[0].clientY); }, {passive:false});
        canvas.addEventListener("touchmove", function(e){ e.preventDefault(); if(dragging) update(e.touches[0].clientX,e.touches[0].clientY); }, {passive:false});
        canvas.addEventListener("touchend", function(e){ e.preventDefault(); release(); }, {passive:false});
        canvas.addEventListener("touchcancel", function(e){ e.preventDefault(); release(); }, {passive:false});
        draw();
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

static esp_err_t action_handler(httpd_req_t *req) {
  char query[256];
  int len = httpd_req_get_url_query_len(req) + 1;

  if (len <= 1 || len > (int)sizeof(query)) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid request");
    return ESP_OK;
  }
  if (httpd_req_get_url_query_str(req, query, len) != ESP_OK) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Failed to read query");
    return ESP_OK;
  }

  // --- Strict Validation for Reset/Restart ONLY via HTTP ---
  if (query_equals(query, "reset", "wifi")) {
    Serial.println("🔄 WiFi Reset requested via Web!");
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "Resetting WiFi...", HTTPD_RESP_USE_STRLEN);
    scheduleRestart(true);
    return ESP_OK;
  }

  if (query_equals(query, "reset", "restart")) {
    Serial.println("♻️ ESP32 Restart requested via Web!");
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "Restarting...", HTTPD_RESP_USE_STRLEN);
    scheduleRestart(false);
    return ESP_OK;
  }

  httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Unknown command via HTTP");
  return ESP_OK;
}

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    } else {
      if (fb->width > 400) {
        if (fb->format != PIXFORMAT_JPEG) {
          bool ok = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
          esp_camera_fb_return(fb);
          fb = NULL;
          if (!ok) { Serial.println("JPEG compression failed"); res = ESP_FAIL; }
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
    if (res == ESP_OK)
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    if (res == ESP_OK)
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));

    // Memory Leak Fix
    if (fb) { esp_camera_fb_return(fb); fb = NULL; _jpg_buf = NULL; }
    else if (_jpg_buf) { free(_jpg_buf); _jpg_buf = NULL; }

    if (res != ESP_OK) break;
  }
  return res;
}

// ======== Server Initialization ========
void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t index_uri  = { "/", HTTP_GET, index_handler, NULL };
  httpd_uri_t action_uri = { "/action", HTTP_GET, action_handler, NULL };
  httpd_uri_t stream_uri = { "/stream", HTTP_GET, stream_handler, NULL };

  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &action_uri);
  }
  config.server_port += 1;
  config.ctrl_port += 1;
  if (httpd_start(&stream_httpd, &config) == ESP_OK)
    httpd_register_uri_handler(stream_httpd, &stream_uri);
}

// ======== WebSocket Backend ========
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("WS client #%u connected — IP: %s\n", client->id(), client->remoteIP().toString().c_str());
    char welcome[128];
    snprintf(welcome, sizeof(welcome), "{\"type\":\"hello\",\"heap\":%d,\"ip\":\"%s\"}", ESP.getFreeHeap(), WiFi.localIP().toString().c_str());
    client->text(welcome);
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("WS client #%u disconnected\n", client->id());
    // أوقف المحركات تلقائياً عند انقطاع الاتصال
    analogWrite(MOTOR_R_PIN_1, 0); analogWrite(MOTOR_R_PIN_2, 0);
    analogWrite(MOTOR_L_PIN_1, 0); analogWrite(MOTOR_L_PIN_2, 0);
    Serial.println("Motors stopped — client disconnected");
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->opcode != WS_TEXT) return;
    char msg[64] = {0};
    size_t copyLen = min(len, sizeof(msg) - 1);
    memcpy(msg, data, copyLen);
    executeCommand(msg);
  }
}

void startWebSocketServer() {
  ws.onEvent(onWsEvent);
  wsServer.addHandler(&ws);
  wsServer.begin();
  Serial.println("WebSocket server started on port 82");
}

// ======== Setup & Loop ========
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  delay(1000);

  Serial.begin(115200);
  Serial.setDebugOutput(false);
  
  pinMode(MOTOR_R_PIN_1, OUTPUT); pinMode(MOTOR_R_PIN_2, OUTPUT);
  pinMode(MOTOR_L_PIN_1, OUTPUT); pinMode(MOTOR_L_PIN_2, OUTPUT);
  pinMode(LED_GPIO_NUM, OUTPUT); digitalWrite(LED_GPIO_NUM, LOW);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0; config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM; config.pin_d2 = Y4_GPIO_NUM; config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM; config.pin_d6 = Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM; config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM; config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM; config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000; config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_VGA; config.jpeg_quality = 10; config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_HVGA; config.jpeg_quality = 12; config.fb_count = 1;
  }

  esp_camera_init(&config);

  wm.setAPStaticIPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
  wm.setCaptivePortalEnable(true);
  if (!wm.autoConnect("ESP32-CAM-Setup")) { ESP.restart(); }

  startCameraServer();
  startWebSocketServer();
}

void loop() {
  static unsigned long lastClean = 0;
  if (millis() - lastClean > 5000) {
    lastClean = millis();
    ws.cleanupClients(); // يحذف العملاء المنقطعين من الذاكرة

    uint32_t freeHeap = ESP.getFreeHeap();
    if (ws.count() > 0) {
      char status[128];
      snprintf(status, sizeof(status), "{\"heap\":%d}", freeHeap);
      ws.textAll(status);
    }
    if (freeHeap < 50000) Serial.println("WARNING: Low memory!");
  }
  delay(10); // وقت تأخير منخفض بفضل AsyncWebServer
}
