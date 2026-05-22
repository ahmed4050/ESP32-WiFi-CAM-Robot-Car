/*
  ╔══════════════════════════════════════════════════════════════╗
  ║         ESP32-CAM Robot Car — Enhanced Edition               ║
  ║  Author  : Ahmed Alqassabi                                   ║
  ║  Features:                                                   ║
  ║   ✅ mDNS  → http://esp32cam.local                          ║
  ║   ✅ Watchdog Timer (WDT) — إعادة تشغيل تلقائي عند التعليق ║
  ║   ✅ Multi-Client WebSocket مع أولوية للعميل الأول           ║
  ║   ✅ تحكم مستقل بسرعة كل محرك (معايرة الانحراف)            ║
  ║   ✅ تحسين جودة البث (FPS ودقة ديناميكية)                   ║
  ║   ✅ Serial Monitor تشخيصي متكامل                           ║
  ╚══════════════════════════════════════════════════════════════╝
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
#include <ESPmDNS.h>

// ═══════════════════════════════════════════════
// ⚙️  إعدادات Watchdog
// ═══════════════════════════════════════════════
#define WDT_TIMEOUT_SEC  10     // إعادة تشغيل إذا تعلق البرنامج أكثر من 10 ثوانٍ

// ═══════════════════════════════════════════════
// ⚙️  إعدادات WebSocket (Multi-Client)
// ═══════════════════════════════════════════════
#define MAX_WS_CLIENTS   4      // أقصى عدد عملاء متزامنين
#define PRIORITY_CLIENT  true   // تفعيل أولوية العميل الأول

// ═══════════════════════════════════════════════
// 📸 Camera Pins (AI-THINKER)
// ═══════════════════════════════════════════════
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

// ═══════════════════════════════════════════════
// 🔧 Motor Pins
// ═══════════════════════════════════════════════
#define MOTOR_R_PIN_1 14
#define MOTOR_R_PIN_2 15
#define MOTOR_L_PIN_1 13
#define MOTOR_L_PIN_2 12

// ═══════════════════════════════════════════════
// 💡 LED Pin
// ═══════════════════════════════════════════════
#define LED_GPIO_NUM 4

// ═══════════════════════════════════════════════
// 🚗 Motor Speed — تحكم مستقل لكل محرك
//    عدّل TRIM لتصحيح الانحراف:
//    قيمة موجبة  → زيادة سرعة اليمين
//    قيمة سالبة → زيادة سرعة اليسار
// ═══════════════════════════════════════════════
int  MOTOR_R_Speed = 170;
int  MOTOR_L_Speed = 170;
int  SPEED_TRIM     = 0;    // معايرة الانحراف (-50 إلى +50)
#define SPEED_MIN    85
#define SPEED_MAX   255
#define SPEED_STEP   85

// ═══════════════════════════════════════════════
// 📡 Streaming
// ═══════════════════════════════════════════════
#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY     = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART         = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// بث ديناميكي — يتكيف مع قوة الإشارة
#define STREAM_QUALITY_GOOD   8    // جودة عالية عند إشارة قوية
#define STREAM_QUALITY_WEAK  14    // جودة أقل عند إشارة ضعيفة
#define RSSI_GOOD_THRESHOLD  -65   // dBm

// ═══════════════════════════════════════════════
// 🌐 Servers
// ═══════════════════════════════════════════════
httpd_handle_t camera_httpd = NULL;
httpd_handle_t stream_httpd = NULL;

AsyncWebServer wsServer(82);
AsyncWebSocket ws("/ws");

WiFiManager wm;

// معرّف العميل ذو الأولوية
uint32_t priorityClientId = 0;
bool     hasPriorityClient = false;

// ═══════════════════════════════════════════════
// Forward declarations
// ═══════════════════════════════════════════════
void startCameraServer();
void startWebSocketServer();
void stopAllMotors();
int  getEffectiveSpeed(int base, bool isRight);

// ═══════════════════════════════════════════════
// ♻️  FreeRTOS Restart Task
// ═══════════════════════════════════════════════
struct RestartTaskParams { bool resetWifi; };

static void restartTask(void *pvParameters) {
  RestartTaskParams *params = (RestartTaskParams *)pvParameters;
  bool resetWifi = params->resetWifi;
  delete params;

  vTaskDelay(pdMS_TO_TICKS(600));

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

// ═══════════════════════════════════════════════
// 🔒 Strict Command Validation
// ═══════════════════════════════════════════════
static bool query_equals(const char *query, const char *key, const char *value) {
  char expected[64];
  snprintf(expected, sizeof(expected), "%s=%s", key, value);
  return strcmp(query, expected) == 0;
}

// ═══════════════════════════════════════════════
// 🚗 تحكم مستقل بسرعة كل محرك (معايرة الانحراف)
// ═══════════════════════════════════════════════
int getEffectiveSpeed(int base, bool isRight) {
  int trimmed = isRight ? base + SPEED_TRIM : base - SPEED_TRIM;
  return constrain(trimmed, SPEED_MIN, SPEED_MAX);
}

void stopAllMotors() {
  analogWrite(MOTOR_R_PIN_1, 0); analogWrite(MOTOR_R_PIN_2, 0);
  analogWrite(MOTOR_L_PIN_1, 0); analogWrite(MOTOR_L_PIN_2, 0);
}

// ═══════════════════════════════════════════════
// 🎮 Motor Control Logic
// ═══════════════════════════════════════════════
void executeCommand(const char *cmd) {
  int rSpeed = getEffectiveSpeed(MOTOR_R_Speed, true);
  int lSpeed = getEffectiveSpeed(MOTOR_L_Speed, false);

  if (strcmp(cmd, "forward") == 0) {
    analogWrite(MOTOR_R_PIN_1, 0);      analogWrite(MOTOR_R_PIN_2, rSpeed);
    analogWrite(MOTOR_L_PIN_1, lSpeed); analogWrite(MOTOR_L_PIN_2, 0);
  } else if (strcmp(cmd, "backward") == 0) {
    analogWrite(MOTOR_R_PIN_1, rSpeed); analogWrite(MOTOR_R_PIN_2, 0);
    analogWrite(MOTOR_L_PIN_1, 0);      analogWrite(MOTOR_L_PIN_2, lSpeed);
  } else if (strcmp(cmd, "left") == 0) {
    analogWrite(MOTOR_R_PIN_1, 0);      analogWrite(MOTOR_R_PIN_2, rSpeed);
    analogWrite(MOTOR_L_PIN_1, 0);      analogWrite(MOTOR_L_PIN_2, lSpeed);
  } else if (strcmp(cmd, "right") == 0) {
    analogWrite(MOTOR_R_PIN_1, rSpeed); analogWrite(MOTOR_R_PIN_2, 0);
    analogWrite(MOTOR_L_PIN_1, lSpeed); analogWrite(MOTOR_L_PIN_2, 0);
  } else if (strcmp(cmd, "stop") == 0) {
    stopAllMotors();
  } else if (strcmp(cmd, "led=on") == 0) {
    digitalWrite(LED_GPIO_NUM, HIGH);
  } else if (strcmp(cmd, "led=off") == 0) {
    digitalWrite(LED_GPIO_NUM, LOW);
  } else if (strcmp(cmd, "plus") == 0) {
    MOTOR_R_Speed = min(MOTOR_R_Speed + SPEED_STEP, SPEED_MAX);
    MOTOR_L_Speed = min(MOTOR_L_Speed + SPEED_STEP, SPEED_MAX);
  } else if (strcmp(cmd, "minus") == 0) {
    MOTOR_R_Speed = max(MOTOR_R_Speed - SPEED_STEP, SPEED_MIN);
    MOTOR_L_Speed = max(MOTOR_L_Speed - SPEED_STEP, SPEED_MIN);
  } else if (strncmp(cmd, "trim=", 5) == 0) {
    // أمر معايرة: trim=+10 أو trim=-15
    SPEED_TRIM = constrain(atoi(cmd + 5), -50, 50);
    Serial.printf("⚙️  Trim set to %d\n", SPEED_TRIM);
  }
}

// ═══════════════════════════════════════════════
// 🌐 HTML / UI
// ═══════════════════════════════════════════════
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

      .info-box { background: #1e1e1e; border: 1px solid #333; border-radius: 8px; padding: 8px 16px; font-size: 12px; margin: 10px 0; text-align: center; width: 92%; max-width: 480px; display:flex; flex-wrap:wrap; gap:6px; justify-content:center;}
      .info-pill { background:#252525; border:1px solid #333; border-radius:12px; padding:3px 10px; }
      #wsStatus.ok  { color: #1D9E75; }
      #wsStatus.err { color: #E24B4A; }
      #clientBadge  { font-size:10px; color:#888; }
      #clientBadge.priority { color:#f0c040; }

      img#photo { width: 92%; max-width: 480px; border-radius: 10px; border: 2px solid #2f4468; margin: 10px 0; }

      #joy-container { display: flex; flex-direction: column; align-items: center; gap: 12px; margin: 10px 0; width: 100%; }
      canvas#joy { touch-action: none; cursor: grab; }
      #cmd-label { font-size: 15px; font-weight: bold; background: #1e1e1e; border: 1px solid #444; border-radius: 20px; padding: 6px 24px; min-width: 140px; text-align: center; transition: background .15s, color .15s; }
      #cmd-label.active { background: #185FA5; color: #fff; }

      .stats { display: flex; gap: 10px; }
      .stat-box { background: #1e1e1e; border: 1px solid #333; border-radius: 8px; padding: 8px 16px; text-align: center; }
      .stat-box small { font-size: 11px; color: #888; display: block; }
      .stat-box span  { font-size: 17px; font-family: monospace; }

      /* Trim slider */
      #trimSection { width:92%; max-width:480px; background:#1e1e1e; border:1px solid #333; border-radius:8px; padding:10px 16px; margin:6px 0; }
      #trimSection label { font-size:12px; color:#888; }
      #trimSlider { width:100%; accent-color:#378ADD; margin-top:4px; }
      #trimVal { font-size:13px; color:#378ADD; font-family:monospace; }

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
      <span class="info-pill"><strong>WS:</strong> <span id="wsStatus" style="color:#888">جاري الاتصال...</span></span>
      <span class="info-pill"><strong>IP:</strong> <span id="ipInfo">—</span></span>
      <span class="info-pill"><strong>RAM:</strong> <span id="heapInfo">—</span></span>
      <span class="info-pill"><strong>RSSI:</strong> <span id="rssiInfo">—</span></span>
      <span class="info-pill" id="clientBadge">متصل</span>
    </div>

    <img src="" id="photo" alt="camera">

    <div id="joy-container">
      <div id="cmd-label">ضع إصبعك هنا</div>
      <canvas id="joy" width="240" height="240"></canvas>
      <div class="stats">
        <div class="stat-box"><small>سرعة</small><span id="vspd">0%</span></div>
        <div class="stat-box"><small>اتجاه</small><span id="vdir">—</span></div>
        <div class="stat-box"><small>عملاء</small><span id="clientCount">—</span></div>
      </div>
    </div>

    <!-- معايرة الانحراف -->
    <div id="trimSection">
      <label>⚙️ معايرة الانحراف: <span id="trimVal">0</span></label>
      <input id="trimSlider" type="range" min="-50" max="50" value="0"
        oninput="document.getElementById('trimVal').textContent=this.value"
        onchange="sendCmd('trim='+this.value)">
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
      var isPriority = false;

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
          wsReady = false; isPriority = false;
          var st = document.getElementById("wsStatus");
          st.textContent = "منقطع — يعاود الاتصال..."; st.className = "err";
          document.getElementById("clientBadge").textContent = "منقطع";
          document.getElementById("clientBadge").className = "";
          setTimeout(initWS, 2000);
        };

        ws.onerror = function() { ws.close(); };

        ws.onmessage = function(e) {
          try {
            var d = JSON.parse(e.data);
            if (d.heap)    document.getElementById("heapInfo").textContent    = Math.round(d.heap / 1024) + " KB";
            if (d.ip)      document.getElementById("ipInfo").textContent      = d.ip;
            if (d.rssi)    document.getElementById("rssiInfo").textContent    = d.rssi + " dBm";
            if (d.clients) document.getElementById("clientCount").textContent = d.clients;
            if (d.priority !== undefined) {
              isPriority = d.priority;
              var badge = document.getElementById("clientBadge");
              badge.textContent = isPriority ? "⭐ متحكم رئيسي" : "👁 مشاهد";
              badge.className   = isPriority ? "priority" : "";
            }
            if (d.type === "hello") {
              document.getElementById("ipInfo").textContent = d.ip;
            }
          } catch(err) {}
        };
      }

      function sendCmd(cmd) {
        // العميل غير ذو الأولوية لا يرسل أوامر حركة
        var moveCommands = ['forward','backward','left','right','stop','plus','minus'];
        if (!isPriority && moveCommands.indexOf(cmd) !== -1 && wsReady) return;

        var now = Date.now();
        if (cmd === lastCmd && now - lastSent < 80) return;
        lastCmd = cmd; lastSent = now;
        if (wsReady) ws.send(cmd);
        else wsQueue.push(cmd);
      }

      window.onload = function() {
        document.getElementById("photo").src = window.location.href.slice(0,-1) + ":81/stream";
        initWS();
        initJoystick();
      };

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
        'w':'forward', 'W':'forward', 's':'backward', 'S':'backward',
        'a':'left', 'A':'left', 'd':'right', 'D':'right',
        ' ':'stop', 'l':'led=on', 'L':'led=on', '+':'plus', '=':'plus', '-':'minus'
      };
      var keyToId = {
        'forward':['k-w','k-up'], 'backward':['k-s','k-down'],
        'left':['k-a','k-left'], 'right':['k-d','k-right'],
        'stop':['k-space'], 'plus':['k-plus'], 'minus':['k-minus'],
        'led=on':['k-l'], 'led=off':['k-l']
      };
      var activeKeyIds = [];

      function updateKeyIndicator(cmd) {
        activeKeyIds.forEach(function(id) {
          var el = document.getElementById(id); if (el) el.classList.remove('active');
        });
        activeKeyIds = keyToId[cmd] || [];
        activeKeyIds.forEach(function(id) {
          var el = document.getElementById(id); if (el) el.classList.add('active');
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
        if (cmd === 'led=on')               { toggleLED(); return; }
        if (cmd === 'plus' || cmd === 'minus') { sendCmd(cmd); updateKeyIndicator(cmd); return; }
        sendCmd(cmd);
        updateKeyIndicator(cmd);
      });

      document.addEventListener('keyup', function(e) {
        var cmd = keyMap[e.key];
        if (!cmd) return;
        delete pressedKeys[e.key];
        var movementKeys = ['ArrowUp','ArrowDown','ArrowLeft','ArrowRight','w','s','a','d','W','S','A','D'];
        var stillMoving = movementKeys.some(function(k) { return pressedKeys[k]; });
        if (!stillMoving) { sendCmd('stop'); updateKeyIndicator('stop'); }
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
          if (a > -135 && a <= -45)  return "forward";
          if (a > 45   && a <= 135)  return "backward";
          if (a > -45  && a <= 45)   return "right";
          return "left";
        }

        function draw() {
          ctx.clearRect(0,0,W,H);
          ctx.beginPath(); ctx.arc(cx,cy,outerR,0,Math.PI*2);
          ctx.fillStyle = "#1e1e1e"; ctx.fill();
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
          document.getElementById("vspd").textContent = "0%";
          document.getElementById("vdir").textContent = "—";
          document.getElementById("cmd-label").textContent = "ضع إصبعك هنا";
          document.getElementById("cmd-label").classList.remove("active");
          sendCmd("stop"); draw();
        }

        canvas.addEventListener("mousedown",   function(e){ dragging=true; update(e.clientX,e.clientY); });
        canvas.addEventListener("mousemove",   function(e){ if(dragging) update(e.clientX,e.clientY); });
        canvas.addEventListener("mouseup",     release);
        canvas.addEventListener("mouseleave",  release);
        canvas.addEventListener("touchstart",  function(e){ e.preventDefault(); dragging=true; update(e.touches[0].clientX,e.touches[0].clientY); }, {passive:false});
        canvas.addEventListener("touchmove",   function(e){ e.preventDefault(); if(dragging) update(e.touches[0].clientX,e.touches[0].clientY); }, {passive:false});
        canvas.addEventListener("touchend",    function(e){ e.preventDefault(); release(); }, {passive:false});
        canvas.addEventListener("touchcancel", function(e){ e.preventDefault(); release(); }, {passive:false});
        draw();
      }
    </script>
  </body>
</html>
)rawliteral";

// ═══════════════════════════════════════════════
// 🌐 HTTP Handlers
// ═══════════════════════════════════════════════
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

// ═══════════════════════════════════════════════
// 📸 Stream Handler — جودة ديناميكية حسب RSSI
// ═══════════════════════════════════════════════
static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb        = NULL;
  esp_err_t    res        = ESP_OK;
  size_t       _jpg_buf_len = 0;
  uint8_t     *_jpg_buf   = NULL;
  char         part_buf[64];

  // اختر الجودة بناءً على قوة الإشارة
  int quality = (WiFi.RSSI() >= RSSI_GOOD_THRESHOLD) ? STREAM_QUALITY_GOOD : STREAM_QUALITY_WEAK;
  sensor_t *s = esp_camera_sensor_get();
  if (s) s->set_quality(s, quality);

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;

  while (true) {
    esp_task_wdt_reset(); // أعد ضبط الـ Watchdog في حلقة البث

    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    } else {
      if (fb->width > 400) {
        if (fb->format != PIXFORMAT_JPEG) {
          bool ok = frame2jpg(fb, quality, &_jpg_buf, &_jpg_buf_len);
          esp_camera_fb_return(fb); fb = NULL;
          if (!ok) { Serial.println("JPEG compression failed"); res = ESP_FAIL; }
        } else {
          _jpg_buf_len = fb->len;
          _jpg_buf     = fb->buf;
        }
      }
    }
    if (res == ESP_OK) {
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));

    if (fb)        { esp_camera_fb_return(fb); fb = NULL; _jpg_buf = NULL; }
    else if (_jpg_buf) { free(_jpg_buf); _jpg_buf = NULL; }

    if (res != ESP_OK) break;
  }
  return res;
}

// ═══════════════════════════════════════════════
// 🚀 Server Initialization
// ═══════════════════════════════════════════════
void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t index_uri  = { "/",       HTTP_GET, index_handler,  NULL };
  httpd_uri_t action_uri = { "/action", HTTP_GET, action_handler, NULL };
  httpd_uri_t stream_uri = { "/stream", HTTP_GET, stream_handler, NULL };

  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &action_uri);
  }
  config.server_port += 1;
  config.ctrl_port   += 1;
  if (httpd_start(&stream_httpd, &config) == ESP_OK)
    httpd_register_uri_handler(stream_httpd, &stream_uri);
}

// ═══════════════════════════════════════════════
// 🔌 WebSocket — Multi-Client مع أولوية للأول
// ═══════════════════════════════════════════════
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {

  if (type == WS_EVT_CONNECT) {
    Serial.printf("WS client #%u connected — IP: %s  (total: %u)\n",
                  client->id(), client->remoteIP().toString().c_str(), server->count());

    // أولوية للعميل الأول
    bool grantPriority = false;
    if (PRIORITY_CLIENT && !hasPriorityClient) {
      hasPriorityClient = true;
      priorityClientId  = client->id();
      grantPriority     = true;
    }

    // رفض الاتصال إذا تجاوز الحد
    if (server->count() > MAX_WS_CLIENTS) {
      Serial.printf("⚠️  Max clients reached, closing #%u\n", client->id());
      client->close();
      return;
    }

    char welcome[160];
    snprintf(welcome, sizeof(welcome),
             "{\"type\":\"hello\",\"heap\":%d,\"ip\":\"%s\",\"rssi\":%d,\"clients\":%u,\"priority\":%s}",
             ESP.getFreeHeap(), WiFi.localIP().toString().c_str(),
             WiFi.RSSI(), server->count(),
             grantPriority ? "true" : "false");
    client->text(welcome);

  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("WS client #%u disconnected (remaining: %u)\n", client->id(), server->count());

    if (client->id() == priorityClientId) {
      hasPriorityClient = false;
      priorityClientId  = 0;
      Serial.println("⭐ Priority client disconnected — motors stopped");
    }
    stopAllMotors();

  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->opcode != WS_TEXT) return;

    char msg[64] = {0};
    size_t copyLen = min(len, sizeof(msg) - 1);
    memcpy(msg, data, copyLen);

    // أوامر الحركة فقط من العميل ذي الأولوية
    bool isMoveCmd = (strcmp(msg,"forward")==0 || strcmp(msg,"backward")==0 ||
                      strcmp(msg,"left")==0    || strcmp(msg,"right")==0    ||
                      strcmp(msg,"stop")==0    || strcmp(msg,"plus")==0     ||
                      strcmp(msg,"minus")==0);

    if (PRIORITY_CLIENT && isMoveCmd && client->id() != priorityClientId) {
      // تجاهل أوامر الحركة من غير العميل ذي الأولوية
      return;
    }

    executeCommand(msg);
  }
}

void startWebSocketServer() {
  ws.onEvent(onWsEvent);
  wsServer.addHandler(&ws);
  wsServer.begin();
  Serial.println("WebSocket server started on port 82");
}

// ═══════════════════════════════════════════════
// ⚡ Setup
// ═══════════════════════════════════════════════
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  delay(1000);

  Serial.begin(115200);
  Serial.setDebugOutput(false);

  // 🐕 Watchdog Timer
  esp_task_wdt_init(WDT_TIMEOUT_SEC, true);
  esp_task_wdt_add(NULL); // سجّل المهمة الرئيسية

  pinMode(MOTOR_R_PIN_1, OUTPUT); pinMode(MOTOR_R_PIN_2, OUTPUT);
  pinMode(MOTOR_L_PIN_1, OUTPUT); pinMode(MOTOR_L_PIN_2, OUTPUT);
  pinMode(LED_GPIO_NUM,   OUTPUT); digitalWrite(LED_GPIO_NUM, LOW);
  stopAllMotors();

  // 📸 Camera init
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0; config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0=Y2_GPIO_NUM; config.pin_d1=Y3_GPIO_NUM;
  config.pin_d2=Y4_GPIO_NUM; config.pin_d3=Y5_GPIO_NUM;
  config.pin_d4=Y6_GPIO_NUM; config.pin_d5=Y7_GPIO_NUM;
  config.pin_d6=Y8_GPIO_NUM; config.pin_d7=Y9_GPIO_NUM;
  config.pin_xclk=XCLK_GPIO_NUM; config.pin_pclk=PCLK_GPIO_NUM;
  config.pin_vsync=VSYNC_GPIO_NUM; config.pin_href=HREF_GPIO_NUM;
  config.pin_sccb_sda=SIOD_GPIO_NUM; config.pin_sccb_scl=SIOC_GPIO_NUM;
  config.pin_pwdn=PWDN_GPIO_NUM; config.pin_reset=RESET_GPIO_NUM;
  config.xclk_freq_hz=20000000; config.pixel_format=PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size=FRAMESIZE_VGA;  config.jpeg_quality=STREAM_QUALITY_GOOD; config.fb_count=2;
  } else {
    config.frame_size=FRAMESIZE_HVGA; config.jpeg_quality=STREAM_QUALITY_WEAK; config.fb_count=1;
  }
  esp_camera_init(&config);

  // 📶 WiFi
  wm.setAPStaticIPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
  wm.setCaptivePortalEnable(true);
  if (!wm.autoConnect("ESP32-CAM-Setup")) { ESP.restart(); }

  // 📊 Serial diagnostics
  Serial.println("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("✅ WiFi Connected Successfully!");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.print  ("📍 IP Address  : "); Serial.println(WiFi.localIP());
  Serial.print  ("📶 RSSI        : "); Serial.print(WiFi.RSSI()); Serial.println(" dBm");
  Serial.print  ("🔌 MAC Address : "); Serial.println(WiFi.macAddress());
  Serial.print  ("💾 Free Heap   : "); Serial.print(ESP.getFreeHeap()/1024); Serial.println(" KB");
  Serial.print  ("🐕 Watchdog    : "); Serial.print(WDT_TIMEOUT_SEC); Serial.println(" sec");
  Serial.print  ("👥 Max Clients : "); Serial.println(MAX_WS_CLIENTS);

  // 📡 mDNS
  if (MDNS.begin("esp32cam")) {
    Serial.println("\n📡 mDNS Responder Started");
    Serial.println("🌐 Access URLs:");
    Serial.println("   • http://esp32cam.local");
    Serial.println("   • http://esp32cam.local:81/stream");
    MDNS.addService("http", "tcp", 80);
    MDNS.addService("ws",   "tcp", 82);
    Serial.println("✓  HTTP & WebSocket services advertised");
  } else {
    Serial.println("❌ Error starting mDNS");
  }
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");

  startCameraServer();
  startWebSocketServer();
}

// ═══════════════════════════════════════════════
// 🔁 Loop
// ═══════════════════════════════════════════════
void loop() {
  esp_task_wdt_reset(); // أعد ضبط الـ Watchdog

  static unsigned long lastClean  = 0;
  static unsigned long lastStatus = 0;

  unsigned long now = millis();

  // كل 5 ثوانٍ: تنظيف العملاء + إرسال حالة
  if (now - lastClean > 5000) {
    lastClean = now;
    ws.cleanupClients();

    uint32_t freeHeap = ESP.getFreeHeap();
    if (ws.count() > 0) {
      char status[160];
      snprintf(status, sizeof(status),
               "{\"heap\":%d,\"rssi\":%d,\"clients\":%u}",
               freeHeap, WiFi.RSSI(), ws.count());
      ws.textAll(status);
    }
    if (freeHeap < 50000) Serial.println("⚠️  WARNING: Low memory!");
  }

  // كل 30 ثانية: طباعة حالة مختصرة في Serial
  if (now - lastStatus > 30000) {
    lastStatus = now;
    Serial.printf("📊 Heap: %d KB | RSSI: %d dBm | WS clients: %u | Speed: R=%d L=%d Trim=%d\n",
                  ESP.getFreeHeap()/1024, WiFi.RSSI(), ws.count(),
                  MOTOR_R_Speed, MOTOR_L_Speed, SPEED_TRIM);
  }

  delay(10);
}
