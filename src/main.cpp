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
#include <esp_task_wdt.h>

// ============================================================
//  TASK HANDLES & FORWARD DECLARATIONS
// ============================================================
TaskHandle_t ControlTaskHandle = NULL;
static constexpr bool VCS_VERBOSE_TASK_LOGS = false;
extern void WebServerTask(void *pvParameters);
static uint16_t cachedSteering = COMM_STEER_CENTER;  // For display task

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

// ============================================================
//  TASK BODIES
// ============================================================

void ControlTask(void *pvParameters) {
    esp_task_wdt_init(10, true); // 10 second timeout, panic on trigger
    for (;;) {
        updateStateMachine();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void CommTask() {
    cachedSteering = getMeasuredSteering(); // For display task

    handleIncomingUART();
    updateDeadman();
    updateLowBrake();
    updateThreeSpeed();
    updateReverse();
    updateUART();
    updateHallCalculations();
    updateThrottle(getMeasuredRPM(), getTargetRPM());
    updateSteeringPID(getTargetSteering(), isAutonomousMode());

    updateRelays(isAutonomousMode());
    if (VCS_VERBOSE_TASK_LOGS) {
        Serial.println(F(" -> CommTask Finished!"));
    }

    updateSteeringPID(getTargetSteering(), isAutonomousMode());
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

    Serial.print(F("1. State... "));
    initState_Machine();
    Serial.println(F("OK"));
    delay(10);

    Serial.print(F("2. UART... "));
    initUART();
    Serial.println(F("OK"));
    delay(10);

    Serial.print(F("3. Throttle/Brake... "));
    initThrottle();
    initLowBrake();
    Serial.println(F("OK"));
    delay(10);

    Serial.print(F("4. Safety... "));
    initDeadman();
    initRelays();
    Serial.println(F("OK"));
    delay(10);

    Serial.print(F("5. Steering... "));
    initSteering();
    Serial.println(F("OK"));
    delay(10);

    Serial.print(F("6. Sensors... "));
    initHallSensors();
    initThreeSpeed();
    initReverse();
    Serial.println(F("OK"));
    delay(10);

    Serial.print(F("7. Display... "));
    initDisplay();
    Serial.println(F("OK"));
    delay(10);
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