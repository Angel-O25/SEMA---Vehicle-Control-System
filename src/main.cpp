// ============================================================
//  main.cpp — SIDLAK 2 VCS
//  Team Wired PH0017003 | Shell Eco-marathon 2026
//
//  Changes from previous revision:
//  • Added DebugTask (priority 1, Core 0) — serial debug mode
//  • DisplayLoop now calls updateDebugDisplay() when in debug mode
// ============================================================

#include <Arduino.h>
#include "vcs_pins.h"
#include "vcs_constants.h"
#include "vcs_state_machine.h"
#include "vcs_uart.h"
#include "vcs_throttle.h"
#include "vcs_lowbrake.h"
#include "vcs_deadman.h"
#include "vcs_relays.h"
#include "vcs_steering.h"
#include "vcs_hallsensor.h"
#include "vcs_reverse.h"
#include "vcs_display.h"
#include "vcs_simulation.h"
#include "vcs_web.h"
#include "vcs_debug.h"
#include <esp_task_wdt.h>

TaskHandle_t ControlTaskHandle = NULL;
static constexpr bool VCS_VERBOSE_TASK_LOGS = false;
extern void WebServerTask(void *pvParameters);

static constexpr uint32_t WDT_TIMEOUT_SEC = 5;

// Cached steering reading: set by CommTask, consumed by DisplayLoop.
// Prevents the slew-rate limit in getMeasuredSteering() from being
// applied more than once per control cycle.
static volatile uint16_t cachedSteering = COMM_STEER_CENTER;

// ─────────────────────────────────────────────────────────────
//  TASKS
// ─────────────────────────────────────────────────────────────
void ControlTask(void *pvParameters) {
    esp_task_wdt_add(NULL);
    for (;;) {
        esp_task_wdt_reset();
        updateStateMachine();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void runCommCycle() {
    cachedSteering = getMeasuredSteering();

    handleIncomingUART();
    updateDeadman();
    updateLowBrake();
    updateReverse();
    updateUART();
    updateHallCalculations();
    updateThrottle(getMeasuredRPM(), getTargetRPM());
    updateSteeringPID(getTargetSteering(), isAutonomousMode());
    updateRelays(isAutonomousMode());
}

void ESP32_CommLoop(void *pvParameters) {
    initSteering();   // LEDC must init on Core 0 — cannot be called from setup() (Core 1)
    esp_task_wdt_add(NULL);
    for (;;) {
        esp_task_wdt_reset();
        runCommCycle();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void ESP32_UILoop(void *pvParameters) {
    esp_task_wdt_add(NULL);
    uint8_t gear = 1;
    for (;;) {
        esp_task_wdt_reset();
        broadcastVehicleTelemetry(gear);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void DisplayLoop(void *pvParameters) {
    // No esp_task_wdt_add here — display is non-critical, a hang
    // should not reboot the vehicle control system.
    for (;;) {
        if (g_debug_mode) {
            updateDebugDisplay();
        } else {
            updateDisplay(getMeasuredRPM(), cachedSteering);
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

// ─────────────────────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────────────────────
void setup() {
    dacWrite(PIN_THROTTLE_OUT, 0);   // MUST be absolute first line
    pinMode(PIN_STEER_ENA, OUTPUT);
    digitalWrite(PIN_STEER_ENA, HIGH);  // disable stepper before any init

    Serial.begin(115200);
    Serial.println(F("\n--- VCS SIDLAK2 ESP32 ---"));
    Serial.println(F("Type 'debug' for debug/calibration mode"));

    esp_task_wdt_init(WDT_TIMEOUT_SEC, true);

    Serial.print(F("1. State...   ")); initState_Machine();  Serial.println(F("OK"));
    Serial.print(F("2. UART...    ")); initUART();            Serial.println(F("OK"));
    Serial.print(F("3. Throttle.. ")); initThrottle();        Serial.println(F("OK"));
    Serial.print(F("4. Brake...   ")); initLowBrake();        Serial.println(F("OK"));
    Serial.print(F("5. Deadman... ")); initDeadman();         Serial.println(F("OK"));
    Serial.print(F("6. Relays...  ")); initRelays();          Serial.println(F("OK"));
    Serial.print(F("7. Steering.. ")); initSteering();        Serial.println(F("OK"));
    Serial.print(F("8. Hall...    ")); initHallSensors();     Serial.println(F("OK"));
    Serial.print(F("9. Reverse... ")); initReverse();         Serial.println(F("OK"));
    Serial.print(F("10. Display.. ")); initDisplay();         Serial.println(F("OK"));
    Serial.println(F("--- BOOT COMPLETE ---"));

    xTaskCreatePinnedToCore(ControlTask,    "Control",   4096, NULL, 5, &ControlTaskHandle, 1);
    xTaskCreatePinnedToCore(ESP32_CommLoop, "Comms",     4096, NULL, 4, NULL,               0);
    xTaskCreatePinnedToCore(ESP32_UILoop,   "UI",        4096, NULL, 2, NULL,               0);
    xTaskCreatePinnedToCore(DisplayLoop,    "Display",   4096, NULL, 1, NULL,               1);
    xTaskCreatePinnedToCore(WebServerTask,  "WebServer", 8192, NULL, 1, NULL,               0);
    xTaskCreatePinnedToCore(DebugTask,      "Debug",     4096, NULL, 1, NULL,               1);
}

void loop() {
    // All work in FreeRTOS tasks
}