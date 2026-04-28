#include <Arduino.h>
#include "vcs_constants.h"
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

// Added scope for target tracking to prevent compiler errors
extern int16_t  current_target_rpm;
extern uint16_t current_target_steer;
extern uint8_t  current_target_brake;
extern uint8_t  current_target_mode;

// Network credentials sanitized for baseline security
const char* ssid = "SIDLAK_VCS_LIVE";
const char* password = "sidlak_secure"; 
String last_rx_hex = "00 00 00 00 00 00 00 00 00 00 00 00 00 00";
String last_tx_hex = "00 00 00 00 00 00 00 00 00 00 00 00 00 00";


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
                <tr><td>3-Speed Switch</td><td id="live_3speed">--</td></tr>
            </table>
        </div>
        <div class="panel">
            <h3>Data Logger</h3>
            <button id="recBtn" onclick="toggleRecording()">START RECORDING</button>
            <button onclick="downloadCSV()">DOWNLOAD TELEMETRY (CSV)</button>
            <button onclick="clearLogs()" style="border-color: #444; color: #888;">CLEAR LOGS</button>
        </div>

        <div class="panel">
            <h3>Jetson Target Requests (UART)</h3>
            <table>
                <tr><td>Target Mode</td><td id="live_t_mode" style="color:#ff00ff; font-weight:bold;">--</td></tr>
                <tr><td>Target RPM</td><td id="live_t_rpm" style="color:#ff00ff;">--</td></tr>
                <tr><td>Target Steer</td><td id="live_t_steer" style="color:#ff00ff;">--</td></tr>
                <tr><td>Target Brake</td><td id="live_t_brake" style="color:#ff00ff;">--</td></tr>
            </table>
        </div>
    </div>
    <div class="panel">

    <div class="panel" style="grid-column: span 2;">
            <h3>Raw UART Stream (Hex)</h3>
            <div style="display: flex; gap: 10px;">
                <div style="flex: 1;">
                    <span style="color: #00FFFF;">[JETSON -> ESP32]</span>
                    <div id="rx_hex" style="background: #111; padding: 10px; border: 1px solid #00FFFF; margin-top: 5px;">--</div>
                </div>
                <div style="flex: 1;">
                    <span style="color: #FFFF00;">[ESP32 -> JETSON]</span>
                    <div id="tx_hex" style="background: #111; padding: 10px; border: 1px solid #FFFF00; margin-top: 5px;">--</div>
                </div>
            </div>
        </div>

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
                document.getElementById('live_3speed').innerText = data.three_speed;
                document.getElementById('live_t_rpm').innerText = data.t_rpm;
                document.getElementById('live_t_steer').innerText = data.t_steer;
                document.getElementById('live_t_brake').innerText = data.t_brake + "%";
                document.getElementById('live_t_mode').innerText = data.t_mode;
                document.getElementById('rx_hex').innerText = data.rx_hex;
                document.getElementById('tx_hex').innerText = data.tx_hex;
                
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
    String fsm_state = String(getStateName(currentState));
    bool dms_active = isDeadmanActive();
    float rpm = getMeasuredRPM();
    uint16_t steer_adc = getMeasuredSteering();
    bool is_rev = isReverseEngaged();

    float speed_kmh = getMeasuredSpeedKmh();
    float steer_angle = (((float)steer_adc - 500.0f) / 500.0f) * MAX_STEERING_ANGLE_DEG;

    int16_t rpm_int = (int16_t)rpm;
    uint8_t rpm_h = (rpm_int >> 8) & 0xFF;
    uint8_t rpm_l = rpm_int & 0xFF;
    uint8_t steer_h = (steer_adc >> 8) & 0xFF;
    uint8_t steer_l = steer_adc & 0xFF;
    uint8_t u_state = 3; 

    String speed_mode = "UNKNOWN";
    if (current_drive_mode == DRIVE_LOW) speed_mode = "LOW (30%)";
    else if (current_drive_mode == DRIVE_MED) speed_mode = "MID (60%)";
    else if (current_drive_mode == DRIVE_HIGH) speed_mode = "HIGH (100%)";

    String json;
    json.reserve(512); 


    
    json = "{";
    json += "\"state\":\"" + fsm_state + "\",";
    json += "\"dms\":\"" + String(dms_active ? "HELD (OK)" : "RELEASED") + "\",";
    json += "\"rpm\":\"" + String(rpm, 1) + "\",";
    json += "\"speed\":\"" + String(speed_kmh, 1) + "\",";
    json += "\"steer_angle\":\"" + String(steer_angle, 1) + "\",";
    json += "\"dir\":\"" + String(is_rev ? "REVERSE" : "FORWARD") + "\",";
    
    json += "\"rpm_h\":\"0x" + String(rpm_h, HEX) + "\",";
    json += "\"rpm_l\":\"0x" + String(rpm_l, HEX) + "\",";
    json += "\"steer_h\":\"0x" + String(steer_h, HEX) + "\",";
    json += "\"steer_l\":\"0x" + String(steer_l, HEX) + "\",";
    json += "\"u_state\":\"" + String(u_state) + "\",";
    json += "\"three_speed\":\"" + speed_mode + "\",";

    json += "\"t_rpm\":\"" + String(current_target_rpm) + "\",";
    json += "\"t_steer\":\"" + String(current_target_steer) + "\",";
    json += "\"t_brake\":\"" + String(current_target_brake) + "\",";
    json += "\"t_mode\":\"" + String(current_target_mode) + "\",";
    json += "\"rx_hex\":\"" + last_rx_hex + "\",";
    json += "\"tx_hex\":\"" + last_tx_hex + "\",";
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

void WebServerTask(void *pvParameters) { 
    initWebServer(); 
    for(;;) vTaskDelay(100 / portTICK_PERIOD_MS); 
}