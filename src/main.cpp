// main.cpp
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
//  The original code had no timeout on Jetson commands.
//  In autonomous mode the FSM will now trip FAULT_SIGNAL_TIMEOUT
//  if no valid command has arrived within 100ms (3 missed cycles
//  at 30Hz upstream). The struct lives here; vcs_uart.h updates
//  the values when a frame is parsed.
// ============================================================
constexpr uint32_t SIGNAL_TIMEOUT_MS = 100;

struct SignalFrame {
    float    value        = 0.0f;
    uint32_t timestamp_ms = 0;
    bool     valid() const {
        return (millis() - timestamp_ms) < SIGNAL_TIMEOUT_MS;
    }
};

// Written by handleIncomingUART() (now on Core 0 only — see FIX 2).
// Read by ControlTask via the staleness check below.
// A lightweight mutex guards the read/write boundary.
static SemaphoreHandle_t g_signalMutex = NULL;
SignalFrame g_steerCmd;
SignalFrame g_throttleCmd;
SignalFrame g_brakeCmd;

// ============================================================
//  FIX 2 — UART CONSOLIDATED ON CORE 0
//  v1.5 bug: handleIncomingUART() ran on Core 1 (ControlTask)
//  while updateUART() ran on Core 0 (CommTask). Both touched
//  shared UART receive buffers and command state with no lock.
//
//  Fix: remove handleIncomingUART() from ControlTask entirely.
//  CommTask (Core 0) now calls both in sequence — RX then TX —
//  so the UART hardware is only ever accessed from one core.
// ============================================================

// ============================================================
//  FIX 3 — HALL SENSOR ISR DEBOUNCE (EMI mitigation)
//  Wraps the existing hallISR with a minimum-pulse-width check
//  to reject ghost edges caused by motor switching noise.
//  MIN_PULSE_WIDTH_US must be < the period of maximum RPM.
//  Example: 300 RPM, 4 magnets → period = 50ms → use 800µs.
// ============================================================
constexpr uint32_t MIN_PULSE_WIDTH_US = 800;

static volatile uint32_t g_lastHallEdge_us = 0;
static volatile uint32_t g_hallFalseEdges  = 0;

// The real ISR lives in vcs_hallsensor — called only after the
// debounce gate below passes.
extern void hallSensorISR_impl();

void IRAM_ATTR hallSensorISR() {
    uint32_t now = micros();
    uint32_t dt  = now - g_lastHallEdge_us;

    if (dt < MIN_PULSE_WIDTH_US) {
        g_hallFalseEdges++;   // count for telemetry / tuning
        return;               // reject — too short to be a real magnet pass
    }

    g_lastHallEdge_us = now;
    hallSensorISR_impl();     // forward to the module's real handler
}

// Diagnostic accessor — expose to telemetry / web layer
uint32_t getHallFalseEdgeCount() { return g_hallFalseEdges; }

// ============================================================
//  TASK BODIES
// ============================================================

// High-priority control loop: FSM tick only. 100Hz on Core 1.
// FIX 2: handleIncomingUART() removed from here — moved to CommTask.
void ControlTask(void *pvParameters) {
    for (;;) {
        // FIX 1: Staleness watchdog — autonomous commands must be fresh
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

        updateStateMachine(getSystemFaults());
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// Inputs / TX / relays. Runs from ESP32_CommLoop on Core 0.
// FIX 2: handleIncomingUART() added here so all UART (RX + TX)
// is handled on Core 0 only. No more cross-core UART access.
void CommTask() {
    handleIncomingUART();     // FIX 2 — was in ControlTask (Core 1)

    updateDeadman();
    updateLowBrake();
    updateThreeSpeed();
    updateReverse();
    updateUART();

    // FSM tick is owned by ControlTask only (single-writer design).
    updateRelays(isAutonomousMode());
    if (VCS_VERBOSE_TASK_LOGS) {
        Serial.println(F(" -> CommTask Finished!"));
    }
}

// Telemetry broadcast. Runs from ESP32_UILoop on Core 0.
void UITask() {
    broadcastVehicleTelemetry();
    if (VCS_VERBOSE_TASK_LOGS) {
        Serial.println(F("   -> UITask Finished!"));
    }
}

// ============================================================
//  TASK WRAPPERS
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
        vTaskDelay(50 / portTICK_PERIOD_MS);  // 20Hz UI refresh
    }
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
    // CRITICAL: DAC output state is undefined at power-on. Drive throttle
    // to 0 BEFORE anything else so the motor controller never sees a
    // spurious commanded throttle during boot.
    dacWrite(PIN_THROTTLE_OUT, 0);

    Serial.begin(115200);

    Serial.println(F("\n--- VCS v1.5: ESP32 38-PIN ---"));
    Serial.println(F("--- VCS v1.5 DIAGNOSTIC BOOT ---"));
    Serial.println(F("Testing modules one by one..."));

    // FIX 1 + 2: create mutex before any tasks start
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

    // FIX 3: initHallSensors() must attach its interrupt to
    // hallSensorISR() defined in this file, NOT to its own internal
    // handler directly. The debounce wrapper here forwards valid
    // edges to hallSensorISR_impl() inside vcs_hallsensor.
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

    // Spin up FreeRTOS tasks AFTER all modules are initialized.
    // Pinning per VCS Technical Spec §9:
    //   Core 1: control (FSM/throttle), display
    //   Core 0: comms (inputs/UART/TX), telemetry, web server
    //
    // FIX 3 — WebServerTask moved to Core 0 (was Core 1 in v1.5).
    // Keeping it on Core 1 allowed HTTP handling to introduce jitter
    // on the FSM/control loop at the same priority level.
    xTaskCreatePinnedToCore(ControlTask,    "Control",   4096, NULL, 5, &ControlTaskHandle, 1);
    xTaskCreatePinnedToCore(ESP32_CommLoop, "Comms",     4096, NULL, 4, NULL,               0);
    xTaskCreatePinnedToCore(ESP32_UILoop,   "UI",        4096, NULL, 2, NULL,               0);
    xTaskCreatePinnedToCore(DisplayLoop,    "Display",   4096, NULL, 1, NULL,               1);
    xTaskCreatePinnedToCore(WebServerTask,  "WebServer", 8192, NULL, 1, NULL,               0); // FIX 3: was core 1
}

// ============================================================
//  LOOP — unused; all work happens in FreeRTOS tasks
// ============================================================
void loop() {
}