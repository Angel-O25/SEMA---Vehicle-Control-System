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

// Defined in the AP web-server module
extern void WebServerTask(void *pvParameters);

// ============================================================
//  TASK BODIES
// ============================================================

// High-priority control loop: FSM tick + UART RX. 100Hz on Core 1.
void ControlTask(void *pvParameters) {
    for (;;) {
        handleIncomingUART();
        updateStateMachine(getSystemFaults());
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// Inputs / TX / relays. Runs from ESP32_CommLoop on Core 0.
void CommTask() {
    updateDeadman();
    updateLowBrake();
    updateThreeSpeed();
    updateReverse();
    updateUART();

    Serial.println(F(" -> Updating State Machine..."));
    updateStateMachine(0);

    Serial.println(F(" -> Updating Relays..."));
    updateRelays(isAutonomousMode());

    Serial.println(F(" -> CommTask Finished!"));
}

// Telemetry broadcast. Runs from ESP32_UILoop on Core 0.
void UITask() {
    Serial.println(F("   -> Running Telemetry..."));
    broadcastVehicleTelemetry();
    Serial.println(F("   -> UITask Finished!"));
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
    Serial2.begin(115200, SERIAL_8N1, JETSON_RX_PIN, JETSON_TX_PIN);

    Serial.println(F("\n--- VCS v1.5: ESP32 38-PIN ---"));
    Serial.println(F("--- VCS v1.5 DIAGNOSTIC BOOT ---"));
    Serial.println(F("Testing modules one by one..."));

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

    // Spin up FreeRTOS tasks AFTER all modules are initialized.
    // Pinning per VCS Technical Spec §9:
    //   Core 1: control (FSM/UART/throttle), display
    //   Core 0: comms (inputs/TX), telemetry, web server
    xTaskCreatePinnedToCore(ControlTask,    "Control",   4096, NULL, 5, &ControlTaskHandle, 1);
    xTaskCreatePinnedToCore(ESP32_CommLoop, "Comms",     4096, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(ESP32_UILoop,   "UI",        4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(DisplayLoop,    "Display",   4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(WebServerTask,  "WebServer", 4096, NULL, 1, NULL, 1);
}

// ============================================================
//  LOOP — unused; all work happens in FreeRTOS tasks
// ============================================================
void loop() {
}