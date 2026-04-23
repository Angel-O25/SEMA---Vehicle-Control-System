#include <Arduino.h>
#include "vcs_constants.h"

#if defined(ESP32_VCS)

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#include "vcs_state_machine.h"
#include "vcs_hallsensor.h"
#include "vcs_steering.h"
#include "vcs_threespeed.h"
#include "vcs_reverse.h"
#include "vcs_uart.h"     
#include "vcs_lowbrake.h" 
#include "vcs_deadman.h"  
#include "vcs_web.h"

extern DriveMode current_drive_mode;

const char* ssid = "SIDLAK_VCS_LIVE";
const char* password = "sidlak_secure"; 

#define WHEEL_CIRCUMFERENCE_M 1.2764f 
#define MAX_STEERING_ANGLE_DEG 35.0f  

AsyncWebServer server(80);
String systemLogBuffer = "";

void vcs_log(String msg) {
    Serial.println("[VCS LOG] " + msg); 
    systemLogBuffer += msg + "|"; 
}

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>SIDLAK 2: LIVE VCS DASHBOARD</title>
    <style>
        body { font-family: 'Courier New', Courier, monospace; background-color: #050505; color: #00FF00; padding: 20px; }
        .grid-container { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; max-width: 1100px; }
        .panel { border: 1px solid #00FF00; padding: 15px; background: #000; margin-bottom: 20px; box-shadow: 0 0 10px rgba(0,255,0,0.2); }
        h3 { margin-top: 0; color: #00FF00; border-bottom: 1px solid #00FF00; padding-bottom: 5px; text-transform: uppercase; }
        table { width: 100%; border-collapse: collapse; margin-top: 10px; }
        th, td { border: 1px solid #333; padding: 10px; text-align: left; }
        button { background-color: #000; color: #00FF00; border: 1px solid #00FF00; padding: 12px; cursor: pointer; font-weight: bold; width: 100%; margin-bottom: 10px; text-transform: uppercase; }
        button:hover { background-color: #00FF00; color: #000; }
        #logBox { height: 300px; overflow-y: scroll; background: #000; color: #0f0; padding: 10px; border: 1px solid #333; font-size: 0.9em; margin-top: 10px; }
        .recording { background-color: #ff0000 !important; color: #fff !important; border: none; animation: blink 1s infinite; }
        @keyframes blink { 50% { opacity: 0.5; } }
    </style>
</head>
<body>
    <h2>SIDLAK 2: LIVE VEHICLE CONTROL SYSTEM</h2>
    <div class="grid-container">
        <div class="panel">
            <h3>Physical Telemetry</h3>
            <table>
                <tr><td>FSM State</td><td id="live_state" style="font-weight:bold;">--</td></tr>
                <tr><td>DMS Safety</td><td id="live_dms">--</td></tr>
                <tr><td>Live Speed</td><td id="live_speed">--</td></tr>
                <tr><td>Wheel RPM</td><td id="live_rpm">--</td></tr>
                <tr><td>Steer Angle</td><td id="live_steer">--</td></tr>
                <tr><td>Gear/Dir</td><td id="live_dir">--</td></tr>
            </table>
        </div>
        <div class="panel">
            <h3>Data Logger</h3>
            <button id="recBtn" onclick="toggleRecording()">START RECORDING</button>
            <button onclick="downloadCSV()">DOWNLOAD TELEMETRY (CSV)</button>
            <button onclick="clearLogs()" style="border-color: #444; color: #888;">CLEAR LOGS</button>
        </div>
    </div>
    <div class="panel">
        <h3>System Activity Log</h3>
        <div id="logBox"></div>
    </div>

    <script>
        let isRecording = false;
        let csvRows = [["Timestamp", "FSM_State", "DMS", "RPM", "Speed_kmh", "Steer_Deg", "Dir"]];

        function appendLog(msg) {
            const log = document.getElementById('logBox');
            log.innerHTML += `[${new Date().toLocaleTimeString()}] ${msg}<br>`;
            log.scrollTop = log.scrollHeight;
        }

        function toggleRecording() {
            isRecording = !isRecording;
            const btn = document.getElementById('recBtn');
            btn.innerText = isRecording ? "STOPPING & SAVING..." : "START RECORDING";
            btn.classList.toggle('recording', isRecording);
            appendLog(isRecording ? "SESSION STARTED" : "SESSION PAUSED");
        }

        function clearLogs() {
            document.getElementById('logBox').innerHTML = '';
            csvRows = [["Timestamp", "FSM_State", "DMS", "RPM", "Speed_kmh", "Steer_Deg", "Dir"]];
            appendLog("Logs cleared.");
        }

        function downloadCSV() {
            if(csvRows.length < 2) return alert("No data recorded!");
            let content = csvRows.map(e => e.join(",")).join("\n");
            let blob = new Blob([content], { type: 'text/csv' });
            let a = document.createElement('a');
            a.href = window.URL.createObjectURL(blob);
            a.download = `SIDLAK_LIVE_LOG_${new Date().getTime()}.csv`;
            a.click();
        }

        setInterval(() => {
            fetch('/data').then(r => r.json()).then(data => {
                document.getElementById('live_state').innerText = data.state;
                document.getElementById('live_dms').innerText = data.dms;
                document.getElementById('live_rpm').innerText = data.rpm;
                document.getElementById('live_speed').innerText = data.speed + " km/h";
                document.getElementById('live_steer').innerText = data.steer_angle + "°";
                document.getElementById('live_dir').innerText = data.dir;
                
                if(isRecording) {
                    csvRows.push([new Date().toLocaleTimeString(), data.state, data.dms, data.rpm, data.speed, data.steer_angle, data.dir]);
                }

                if (data.sys_logs && data.sys_logs !== "") {
                    data.sys_logs.split('|').forEach(msg => {
                        if(msg.length > 0) appendLog("ESP32: " + msg);
                    });
                }
            }).catch(err => console.error("Telemetry Break:", err));
        }, 200);
        
        appendLog("System Online: Ready for live telemetry.");
    </script>
</body>
</html>
)rawliteral";

String getTelemetryJSON() {
    // 1. Hardware Readings
    String fsm_state = String(getStateName(currentState));
    bool dms_active = isDeadmanActive();
    float rpm = getMeasuredRPM();
    uint16_t steer_adc = getMeasuredSteering();
    bool is_rev = isReverseEngaged();

    // 2. Calculations
    float speed_kmh = (rpm * WHEEL_CIRCUMFERENCE_M * 60.0f) / 1000.0f;
    float steer_angle = (((float)steer_adc - 500.0f) / 500.0f) * MAX_STEERING_ANGLE_DEG;

    // 3. JSON Construction
    String json = "{";
    json += "\"state\":\"" + fsm_state + "\",";
    json += "\"dms\":\"" + String(dms_active ? "HELD (OK)" : "RELEASED") + "\",";
    json += "\"rpm\":\"" + String(rpm, 1) + "\",";
    json += "\"speed\":\"" + String(speed_kmh, 1) + "\",";
    json += "\"steer_angle\":\"" + String(steer_angle, 1) + "\",";
    json += "\"dir\":\"" + String(is_rev ? "REVERSE" : "FORWARD") + "\",";
    json += "\"sys_logs\":\"" + systemLogBuffer + "\"";
    json += "}";
    
    systemLogBuffer = ""; 
    return json;
}

void initWebServer() {
    WiFi.softAP(ssid, password);
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *req){ req->send(200, "text/html", index_html); });
    server.on("/data", HTTP_GET, [](AsyncWebServerRequest *req){ req->send(200, "application/json", getTelemetryJSON()); });
    server.begin();
}

void WebServerTask(void *pvParameters) { initWebServer(); for(;;) vTaskDelay(100 / portTICK_PERIOD_MS); }
#endif