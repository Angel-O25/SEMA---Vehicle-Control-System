// main.cpp
// ============================================================
//  CHANGES FROM ORIGINAL (call-site fixes only):
//
//  FIX #2 call site — updateStateMachine(getSystemFaults())
//    changed to updateStateMachine().
//    The faults parameter was removed in vcs_state_machine.cpp
//    (FIX #2) — the function now reads g_systemFaults directly.
//    Update vcs_state_machine.h: void updateStateMachine();
//
//  FIX #12 call site — broadcastVehicleTelemetry()
//    changed to broadcastVehicleTelemetry(gear).
//    FIX #12 in vcs_uart.cpp moved gear computation to the caller
//    so vcs_uart does not depend on vcs_threespeed internals.
//    Gear is computed in UITask() before the call.
//    Update vcs_uart.h: void broadcastVehicleTelemetry(uint8_t gear);
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
#include "vcs_threespeed.h"
#include "vcs_reverse.h"
#include "vcs_display.h"
#include "vcs_simulation.h"
#include "vcs_web.h"

// ============================================================
//  TASK HANDLES & FORWARD DECLARATIONS
// ============================================================
TaskHandle_t ControlTaskHandle = NULL;
static constexpr bool VCS_VERBOSE_TASK_LOGS = false;

extern void WebServerTask(void *pvParameters);

// ============================================================
//  FIX 1 — SIGNAL STALENESS GUARD
// ============================================================
constexpr uint32_t SIGNAL_TIMEOUT_MS = 100;

struct SignalFrame {
    float    value        = 0.0f;
    uint32_t timestamp_ms = 0;
    bool     valid() const {
        return (millis() - timestamp_ms) < SIGNAL_TIMEOUT_MS;
    }
};

static SemaphoreHandle_t g_signalMutex = NULL;
SignalFrame g_steerCmd;
SignalFrame g_throttleCmd;
SignalFrame g_brakeCmd;

// ============================================================
//  FIX 3 — HALL SENSOR ISR DEBOUNCE (EMI mitigation)
// ============================================================
constexpr uint32_t MIN_PULSE_WIDTH_US = 800;

static volatile uint32_t g_lastHallEdge_us = 0;
static volatile uint32_t g_hallFalseEdges  = 0;

extern void hallSensorISR_impl();

void IRAM_ATTR hallSensorISR() {
    uint32_t now = micros();
    uint32_t dt  = now - g_lastHallEdge_us;

    if (dt < MIN_PULSE_WIDTH_US) {
        g_hallFalseEdges++;
        return;
    }

    g_lastHallEdge_us = now;
    hallSensorISR_impl();
}

uint32_t getHallFalseEdgeCount() { return g_hallFalseEdges; }

// ============================================================
//  TASK BODIES
// ============================================================

void ControlTask(void *pvParameters) {
    for (;;) {
        if (isAutonomousMode()) {
            bool stale = false;
            if (xSemaphoreTake(g_signalMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                stale = !g_steerCmd.valid()    ||
                        !g_throttleCmd.valid() ||
                        !g_brakeCmd.valid();
                xSemaphoreGive(g_signalMutex);
            }
            if (stale) triggerFault(FAULT_SIGNAL_TIMEOUT);
        }

        // FIX #2 call site: removed getSystemFaults() argument.
        // updateStateMachine() now reads g_systemFaults directly.
        // Old: updateStateMachine(getSystemFaults());
        updateStateMachine();

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void CommTask() {
    handleIncomingUART();

    updateDeadman();
    updateLowBrake();
    updateThreeSpeed();
    updateReverse();
    updateUART();

    updateRelays(isAutonomousMode());
    if (VCS_VERBOSE_TASK_LOGS) {
        Serial.println(F(" -> CommTask Finished!"));
    }
}

void UITask() {
    // FIX #12 call site: gear is computed here (caller responsibility)
    // so broadcastVehicleTelemetry() does not need to depend on
    // vcs_threespeed internals. Matches the updated function signature.
    uint8_t gear = 1;   // default: normal
    if      (current_drive_mode == DRIVE_LOW)  gear = 0;
    else if (current_drive_mode == DRIVE_HIGH) gear = 2;

    // Old: broadcastVehicleTelemetry();
    broadcastVehicleTelemetry(gear);

    if (VCS_VERBOSE_TASK_LOGS) {
        Serial.println(F("   -> UITask Finished!"));
    }
}

// ============================================================
//  TASK WRAPPERS (unchanged)
// ============================================================
void ESP32_CommLoop(void *pvParameters) {
    for (;;) {
        CommTask();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void ESP32_UILoop(void *pvParameters) {
    for (;;) {
        UITask();
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void DisplayLoop(void *pvParameters) {
    for (;;) {
        updateDisplay(getMeasuredRPM(), getMeasuredSteering(), current_drive_mode);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

// ============================================================
//  SETUP (unchanged)
// ============================================================
void setup() {
    dacWrite(PIN_THROTTLE_OUT, 0);

    Serial.begin(115200);

    Serial.println(F("\n--- VCS v1.5: ESP32 38-PIN ---"));
    Serial.println(F("--- VCS v1.5 DIAGNOSTIC BOOT ---"));
    Serial.println(F("Testing modules one by one..."));

    g_signalMutex = xSemaphoreCreateMutex();
    if (!g_signalMutex) {
        Serial.println(F("[FATAL] Mutex alloc failed"));
        while (1) delay(1000);
    }

    Serial.print(F("1. State... "));
    initState_Machine();
    Serial.println(F("OK"));
    delay(500);

    Serial.print(F("2. UART... "));
    initUART();
    Serial.println(F("OK"));
    delay(500);

    Serial.print(F("3. Throttle/Brake... "));
    initThrottle();
    initLowBrake();
    Serial.println(F("OK"));
    delay(500);

    Serial.print(F("4. Safety... "));
    initDeadman();
    initRelays();
    Serial.println(F("OK"));
    delay(500);

    Serial.print(F("5. Steering... "));
    initSteering();
    Serial.println(F("OK"));
    delay(500);

    Serial.print(F("6. Sensors... "));
    initHallSensors();
    initThreeSpeed();
    initReverse();
    Serial.println(F("OK"));
    delay(500);

    Serial.print(F("7. Display... "));
    initDisplay();
    Serial.println(F("OK"));

    Serial.println(F("--- SURVIVED BOOT SEQUENCE ---"));

    xTaskCreatePinnedToCore(ControlTask,    "Control",   4096, NULL, 5, &ControlTaskHandle, 1);
    xTaskCreatePinnedToCore(ESP32_CommLoop, "Comms",     4096, NULL, 4, NULL,               0);
    xTaskCreatePinnedToCore(ESP32_UILoop,   "UI",        4096, NULL, 2, NULL,               0);
    xTaskCreatePinnedToCore(DisplayLoop,    "Display",   4096, NULL, 1, NULL,               1);
    xTaskCreatePinnedToCore(WebServerTask,  "WebServer", 8192, NULL, 1, NULL,               0);
}

// ============================================================
//  LOOP — unused; all work happens in FreeRTOS tasks
// ============================================================
void loop() {
}