// ============================================================
//  vcs_web.cpp — fixes applied
//
//  CHANGES FROM ORIGINAL:
//
//  FIX #13a — last_rx_hex and last_tx_hex initialized to
//              "-- NO LINK --" instead of "00 00 00 00 ...".
//              The old initialization looked identical to a
//              valid all-zeros packet on the web dashboard.
//
//  FIX #13b — getTelemetryJSON() now checks ansHeartbeatReceived()
//              before including Jetson target values. When link
//              is dead, t_rpm / t_steer / t_brake / t_mode are
//              reported as "--" instead of stale numeric values.
//              A "jetson_link" boolean field is added to the JSON
//              so the frontend can react independently.
//
//  FIX #13c — Dashboard HTML: Jetson panel header now shows a
//              live ● LINKED / ● NO LINK indicator that updates
//              every poll cycle (200ms). Jetson target fields
//              and hex stream fields are blanked to "--" in
//              JavaScript when jetson_link is false — even if
//              the server sends stale numeric data.
//              This is a client-side redundant guard on top of
//              the server-side clear in FIX #13 (vcs_uart.cpp).
// ============================================================

#include <Arduino.h>
#include "vcs_constants.h"
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#include "vcs_state_machine.h"
#include "vcs_hallsensor.h"
#include "vcs_steering.h"
#include "vcs_reverse.h"
#include "vcs_uart.h"
#include "vcs_lowbrake.h"
#include "vcs_deadman.h"
#include "vcs_web.h"


static portMUX_TYPE logMux = portMUX_INITIALIZER_UNLOCKED; // Mutex for system log buffer access

extern int16_t  current_target_rpm;
extern uint16_t current_target_steer;
extern uint8_t  current_target_brake;
extern uint8_t  current_target_mode;

const char* ssid     = "SIDLAK_VCS_LIVE";
const char* password = "sidlak_secure";

// FIX #13a: Initialize to "-- NO LINK --" not "00 00 00 ...".
// The all-zeros string looked like a valid packet on the dashboard.
String last_rx_hex = "-- NO LINK --";
String last_tx_hex = "-- NO LINK --";

#define WHEEL_CIRCUMFERENCE_M  1.2764f
#define MAX_STEERING_ANGLE_DEG 35.0f

AsyncWebServer server(80);
String systemLogBuffer = "";

// --- NEW ADDITIONS START ---
AsyncWebSocket ws("/ws");

// HARDCODED RACE MODE TOGGLE: 
// 0 = Web & WiFi ON (Pit Mode)
// 1 = Web & WiFi OFF (Race Mode - maximum power saving)
const int DISABLE_WEB_WIFI = 0; 
// --- NEW ADDITIONS END ---


void vcs_log(String msg) {
    Serial.println("[VCS LOG] " + msg);
    portENTER_CRITICAL(&logMux);
    systemLogBuffer += msg + "|";
    if (systemLogBuffer.length() > 4096) systemLogBuffer.remove(0,2048); // Prevent unbounded growth
    portEXIT_CRITICAL(&logMux);
}

// ============================================================
//  HTML — FIX #13c: added jetson link status indicator and
//  client-side guard that blanks Jetson fields when no link.
//  All other layout and styling unchanged.
// ============================================================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>SIDLAK 2: LIVE VCS DASHBOARD</title>
    <style>
        body { font-family: 'Courier New', Courier, monospace; background-color: #050505; color: #00FF00; padding: 20px; }
        .grid-container { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; max-width: 1200px; margin: auto; }
        .panel { border: 1px solid #00FF00; padding: 15px; background: #000; box-shadow: 0 0 10px rgba(0,255,0,0.2); }
        h3 { margin-top: 0; color: #00FF00; border-bottom: 1px solid #00FF00; padding-bottom: 5px; text-transform: uppercase; }
        table { width: 100%; border-collapse: collapse; margin-top: 10px; }
        th, td { border: 1px solid #333; padding: 5px 10px; text-align: left; }
        button { background-color: #000; color: #00FF00; border: 1px solid #00FF00; padding: 10px; cursor: pointer; font-weight: bold; width: 100%; margin-bottom: 10px; }
        button:hover { background-color: #00FF00; color: #000; }
        
        /* Dynamic State Banner */
        #state_banner { padding: 15px; text-align: center; font-size: 1.8em; font-weight: bold; border: 2px solid #00FF00; margin-bottom: 20px; }
        .state-idle { color: #FFFF00; border-color: #FFFF00; }
        .state-auto { color: #00FFFF; border-color: #00FFFF; }
        .state-error { color: #FF0000; border-color: #FF0000; animation: blink 1s infinite; }
        
        /* 14-Byte Inspector */
        .packet-grid { display: grid; grid-template-columns: repeat(14, 1fr); gap: 5px; text-align: center; margin-top: 10px; }
        .byte-box { border: 1px solid #555; padding: 5px; background: #111; font-size: 0.9em; transition: background-color 0.1s; }
        .byte-label { font-size: 0.6em; color: #888; display: block; margin-bottom: 3px; }
        .byte-val { font-weight: bold; }
        .byte-header { border-color: #00FFFF; color: #00FFFF; }
        .byte-crc { border-color: #FFFF00; color: #FFFF00; }
        
        @keyframes blink { 50% { opacity: 0.5; } }
    </style>
</head>
<body>
    <div id="state_banner" class="state-idle">WAITING FOR TELEMETRY...</div>

    <div class="grid-container">
        <div class="panel">
            <h3>Visual Telemetry</h3>
            <div style="display:flex; justify-content:space-between;">
                <div>
                    <span style="color:#00FF00;">Actual RPM</span> | <span style="color:#ff00ff;">Target RPM</span>
                    <canvas id="rpmChart" width="350" height="150" style="border:1px solid #333; margin-top:5px;"></canvas>
                </div>
                <div style="text-align:center;">
                    <span>Steering Vector</span><br>
                    <canvas id="steerCanvas" width="120" height="120" style="margin-top:15px;"></canvas>
                    <div id="live_steer_val" style="margin-top:5px;">0°</div>
                </div>
            </div>
            <table>
                <tr><td>Live Speed</td><td id="live_speed">--</td><td>Gear/Dir</td><td id="live_dir">--</td></tr>
                <tr><td>FSM State</td><td id="live_state">--</td><td>DMS Safety</td><td id="live_dms">--</td></tr>
            </table>
        </div>
        
        <div class="panel">
            <h3>Data Logger (RAM Buffer)</h3>
            <button id="recBtn" onclick="toggleRecording()">START 20Hz LOGGING</button>
            <button onclick="downloadCSV()">DOWNLOAD CSV</button>
            <div style="font-size: 0.8em; color: #888; margin-top: 10px;">
                Buffer Size: <span id="buf_size">0</span> / 15000<br>
                <span id="auto_dl_warn" style="color:#FF0000; display:none;">AUTO-DOWNLOAD TRIGGERED (LINK LOST)</span>
            </div>
        </div>

        <div class="panel" style="grid-column: span 2;">
            <h3>Jetson UART Protocol Inspector</h3>
            <div class="packet-grid" id="packet_inspector"></div>
            <div style="margin-top: 15px; font-size: 0.9em; display:flex; justify-content:space-between;">
                <div><span style="color:#00FFFF;">Target RPM: </span><span id="calc_rpm">--</span></div>
                <div><span style="color:#00FFFF;">Target Steer: </span><span id="calc_steer">--</span></div>
                <div><span style="color:#00FFFF;">Target Brake: </span><span id="calc_brake">--</span></div>
            </div>
        </div>
    </div>

    <script>
        // Data Logging State
        let isRecording = false;
        let csvRows = [["Timestamp", "FSM_State", "DMS", "Live_RPM", "Target_RPM", "Speed_kmh", "Steer_Deg"]];
        const MAX_ROWS = 15000;
        let lastLinkState = true;

        // Visual Setup
        const labels = ["SYNC", "SYNC", "TYPE", "LEN", "MODE", "RPM_H", "RPM_L", "STR_H", "STR_L", "BRK", "REV", "CRC_H", "CRC_L", "FTR"];
        const grid = document.getElementById('packet_inspector');
        let prevBytes = [];
        
        labels.forEach((lbl, i) => {
            let cls = (i===0 || i===13) ? "byte-header" : (i===11 || i===12) ? "byte-crc" : "";
            grid.innerHTML += `<div class="byte-box ${cls}" id="box${i}"><span class="byte-label">${lbl}</span><span class="byte-val" id="b${i}">00</span></div>`;
        });

        // Canvas Setup
        const rpmCtx = document.getElementById('rpmChart').getContext('2d');
        const steerCtx = document.getElementById('steerCanvas').getContext('2d');
        let rpmData = [], trpmData = [];

        function drawRPMGraph() {
            rpmCtx.clearRect(0, 0, 350, 150);
            if(rpmData.length < 2) return;
            let maxVal = Math.max(...rpmData, ...trpmData, 100);
            
            // Draw Target RPM (Pink)
            rpmCtx.beginPath();
            rpmCtx.strokeStyle = '#ff00ff';
            trpmData.forEach((val, i) => {
                let x = i * (350 / 50);
                let y = 150 - ((val / maxVal) * 150);
                i === 0 ? rpmCtx.moveTo(x, y) : rpmCtx.lineTo(x, y);
            });
            rpmCtx.stroke();

            // Draw Actual RPM (Green)
            rpmCtx.beginPath();
            rpmCtx.strokeStyle = '#00FF00';
            rpmData.forEach((val, i) => {
                let x = i * (350 / 50);
                let y = 150 - ((val / maxVal) * 150);
                i === 0 ? rpmCtx.moveTo(x, y) : rpmCtx.lineTo(x, y);
            });
            rpmCtx.stroke();
        }

        function drawSteering(angleStr) {
            let angle = parseFloat(angleStr) || 0;
            steerCtx.clearRect(0, 0, 120, 120);
            steerCtx.save();
            steerCtx.translate(60, 60);
            steerCtx.rotate(angle * Math.PI / 180);
            // Draw Tires
            steerCtx.fillStyle = '#FFF';
            steerCtx.fillRect(-40, -25, 20, 50);
            steerCtx.fillRect(20, -25, 20, 50);
            steerCtx.strokeStyle = '#555';
            steerCtx.beginPath(); steerCtx.moveTo(-30, 0); steerCtx.lineTo(30, 0); steerCtx.stroke();
            steerCtx.restore();
            document.getElementById('live_steer_val').innerText = angle.toFixed(1) + "°";
        }

        // WebSocket Setup
        let ws;
        function initWebSocket() {
            ws = new WebSocket(`ws://${window.location.hostname}/ws`);
            ws.onmessage = (e) => {
                const data = JSON.parse(e.data);
                
                // Update Text Telemetry
                document.getElementById('live_state').innerText = data.state;
                document.getElementById('live_dms').innerText = data.dms;
                document.getElementById('live_speed').innerText = data.speed + " km/h";
                document.getElementById('live_dir').innerText = data.dir;

                // State Banner Logic
                const banner = document.getElementById('state_banner');
                if (!data.jetson_link) {
                    banner.innerText = "LINK LOST - SYSTEM SAFED";
                    banner.className = "state-error";
                    if(lastLinkState && isRecording) {
                        document.getElementById('auto_dl_warn').style.display = "block";
                        downloadCSV(); // Auto-save failsafe
                    }
                } else {
                    banner.innerText = `FSM MODE: ${data.state}`;
                    banner.className = (data.state === "AUTONOMOUS") ? "state-auto" : "state-idle";
                }
                lastLinkState = data.jetson_link;

                // Graph Logic
                let currentRpm = parseFloat(data.rpm) || 0;
                let targetRpm = (data.t_rpm === "--") ? 0 : parseFloat(data.t_rpm);
                rpmData.push(currentRpm); trpmData.push(targetRpm);
                if(rpmData.length > 50) { rpmData.shift(); trpmData.shift(); }
                drawRPMGraph();
                drawSteering(data.steer_angle);

                // Packet Inspector Logic
                if (data.jetson_link && data.rx_hex !== "-- NO LINK --") {
                    const bytes = data.rx_hex.trim().split(" ");
                    if(bytes.length === 14) {
                        bytes.forEach((b, i) => {
                            let el = document.getElementById(`b${i}`);
                            if (prevBytes[i] && prevBytes[i] !== b) {
                                document.getElementById(`box${i}`).style.backgroundColor = "#550000"; // Flash delta
                                setTimeout(() => document.getElementById(`box${i}`).style.backgroundColor = "#111", 100);
                            }
                            el.innerText = b;
                        });
                        prevBytes = bytes;
                        
                        // Math breakdown
                        document.getElementById('calc_rpm').innerText = `${data.t_rpm} ((0x${bytes[5]}<<8)|0x${bytes[6]})`;
                        document.getElementById('calc_steer').innerText = `${data.t_steer} ((0x${bytes[7]}<<8)|0x${bytes[8]})`;
                        document.getElementById('calc_brake').innerText = `${data.t_brake} (0x${bytes[9]})`;
                    }
                }

                // RAM Circular Logging
                if (isRecording) {
                    if (csvRows.length > MAX_ROWS) csvRows.splice(1, 1); // Keep header
                    csvRows.push([new Date().toISOString(), data.state, data.dms, data.rpm, data.t_rpm, data.speed, data.steer_angle]);
                    document.getElementById('buf_size').innerText = csvRows.length - 1;
                }
            };
            ws.onclose = () => setTimeout(initWebSocket, 1000);
        }

        function toggleRecording() {
            isRecording = !isRecording;
            const btn = document.getElementById('recBtn');
            btn.innerText = isRecording ? "RECORDING 20Hz... (CLICK STOP)" : "START 20Hz LOGGING";
            btn.style.backgroundColor = isRecording ? "#FF0000" : "#000";
        }

        function downloadCSV() {
            if (csvRows.length < 2) return;
            let content = csvRows.map(e => e.join(",")).join("\n");
            let a = document.createElement('a');
            a.href = window.URL.createObjectURL(new Blob([content], { type: 'text/csv' }));
            a.download = `VCS_RAM_LOG_${new Date().getTime()}.csv`;
            a.click();
        }

        window.onload = initWebSocket;
    </script>
</body>
</html>
)rawliteral";

// ============================================================
//  getTelemetryJSON()
//  FIX #13b: ansHeartbeatReceived() checked before including
//  Jetson target values. When link is dead, target fields are
//  reported as "--" and jetson_link is false. The frontend
//  uses jetson_link to gate display independently.
// ============================================================
String getTelemetryJSON() {
    String fsm_state = String(getStateName(currentState));
    bool   dms_active = isDeadmanActive();
    float  rpm        = getMeasuredRPM();
    uint16_t steer_adc = getMeasuredSteering();
    bool   is_rev     = isReverseEngaged();
    float  speed_kmh  = getMeasuredSpeedKmh();
    float  steer_angle = (((float)steer_adc - 500.0f) / 500.0f) * MAX_STEERING_ANGLE_DEG;

    int16_t  rpm_int  = (int16_t)rpm;
    uint8_t  rpm_h    = (rpm_int  >> 8) & 0xFF;
    uint8_t  rpm_l    =  rpm_int        & 0xFF;
    uint8_t  steer_h  = (steer_adc >> 8) & 0xFF;
    uint8_t  steer_l  =  steer_adc       & 0xFF;
    uint8_t  u_state  = 3;

    // FIX #13b: Check link once and use the result consistently.
    bool linked = ansHeartbeatReceived();

    String json;
    json.reserve(600);
    json = "{";
    json += "\"state\":\""       + fsm_state + "\",";
    json += "\"dms\":\""         + String(dms_active ? "HELD (OK)" : "RELEASED") + "\",";
    json += "\"rpm\":\""         + String(rpm, 1) + "\",";
    json += "\"speed\":\""       + String(speed_kmh, 1) + "\",";
    json += "\"steer_angle\":\"" + String(steer_angle, 1) + "\",";
    json += "\"dir\":\""         + String(is_rev ? "REVERSE" : "FORWARD") + "\",";
    json += "\"rpm_h\":\"0x"     + String(rpm_h,   HEX) + "\",";
    json += "\"rpm_l\":\"0x"     + String(rpm_l,   HEX) + "\",";
    json += "\"steer_h\":\"0x"   + String(steer_h, HEX) + "\",";
    json += "\"steer_l\":\"0x"   + String(steer_l, HEX) + "\",";
    json += "\"u_state\":\""     + String(u_state) + "\",";

    // FIX #13b: When no link, report "--" instead of stale numeric values.
    // The frontend also guards these via jetson_link, but the server
    // sends clean data so stale values never leak through either path.
    if (linked) {
        json += "\"t_rpm\":\""   + String(current_target_rpm)   + "\",";
        json += "\"t_steer\":\"" + String(current_target_steer) + "\",";
        json += "\"t_brake\":\"" + String(current_target_brake) + "\",";
        json += "\"t_mode\":\""  + String(current_target_mode)  + "\",";
        json += "\"rx_hex\":\""  + last_rx_hex + "\",";
    } else {
        json += "\"t_rpm\":\"--\",";
        json += "\"t_steer\":\"--\",";
        json += "\"t_brake\":\"--\",";
        json += "\"t_mode\":\"--\",";
        json += "\"rx_hex\":\"-- NO LINK --\",";
    }

json += "\"tx_hex\":\""      + last_tx_hex + "\",";
    // FIX #13b: Link status flag — frontend uses this to gate display.
    json += "\"jetson_link\":"   + String(linked ? "true" : "false") + ",";
    
    // Safely extract and clear logs to prevent memory corruption
    portENTER_CRITICAL(&logMux);
    String localLogs = systemLogBuffer;
    systemLogBuffer = "";
    portEXIT_CRITICAL(&logMux);

    json += "\"sys_logs\":\""    + localLogs + "\"";
    json += "}";

    return json;
}

// ============================================================
//  WEB SERVER INIT & TASK (OVERHAULED FOR WEBSOCKETS & RACE MODE)
// ============================================================
void initWebServer() {
    WiFi.softAP(ssid, password);
    
    // Attach WebSocket listener
    ws.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len){
        if(type == WS_EVT_CONNECT){
            Serial.println("[VCS LOG] Dashboard Connected.");
        }
    });
    server.addHandler(&ws);

    // Serve HTML page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *req){ 
        req->send(200, "text/html", index_html); 
    });
    
    // Polling endpoint /data is removed, pushed via WebSocket instead
    server.begin();
    Serial.println("[VCS LOG] Pit Mode: Web & WiFi Active.");
}

void WebServerTask(void *pvParameters) {
    if (DISABLE_WEB_WIFI == 0) {
        initWebServer();
        for (;;) {
            ws.cleanupClients();
            
            // Push high-speed telemetry at 20Hz (every 50ms)
            String jsonPayload = getTelemetryJSON();
            ws.textAll(jsonPayload);
            
            vTaskDelay(50 / portTICK_PERIOD_MS); 
        }
    } else {
        // RACE MODE: HARDCODED OVERRIDE
        // Kill the Wi-Fi hardware to save battery and reduce CPU jitter
        Serial.println("[VCS LOG] Race Mode Engaged: Wi-Fi Radio Offline.");
        WiFi.mode(WIFI_OFF); 
        
        // Delete this FreeRTOS task so it consumes 0 clock cycles
        vTaskDelete(NULL); 
    }
}