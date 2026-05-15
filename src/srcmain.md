# Project Files Export

Export time: 5/15/2026, 6:13:41 PM

Source directory: `src`

Output file: `project-files.md`

## Directory Structure

```
src
└── main.cpp
```

## File Statistics

- Total files: 1
- Total size: 6.7 KB

### File Type Distribution

| Extension | Files | Total Size |
| --- | --- | --- |
| .cpp | 1 | 6.7 KB |

## File Contents

### main.cpp

```cpp
// main.cpp
// main.cpp — Option A revision
// ============================================================
//  CHANGES FROM PREVIOUS REVISION:
//
//  • WDT properly initialised. Previous version called
//    esp_task_wdt_init() from inside ControlTask but never
//    subscribed the task (esp_task_wdt_add) or fed it
//    (esp_task_wdt_reset) — so the watchdog never armed.
//    Now: init once in setup(), subscribe in each long-running
//    task, feed every tick.
//
//  • Duplicate updateSteeringPID() call removed from CommTask.
//
//  • cachedSteering wired through to DisplayLoop instead of
//    DisplayLoop calling getMeasuredSteering() directly.
//
//  • Dead SignalFrame struct removed — staleness is enforced
//    by ansHeartbeatReceived() in vcs_uart.cpp.
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

// Watchdog timeout. 5 seconds is long enough that no real control loop
// should ever hit it, but short enough that a true hang reboots fast.
static constexpr uint32_t WDT_TIMEOUT_SEC = 5;

// Cached single-tick steering reading. Computed once at the top of
// CommTask, consumed by DisplayLoop. Prevents the slew-rate limit
// inside getMeasuredSteering() from being applied multiple times
// per control cycle.
static volatile uint16_t cachedSteering = COMM_STEER_CENTER;

// ============================================================
//  TASK BODIES
// ============================================================

void ControlTask(void *pvParameters) {
    esp_task_wdt_add(NULL);   // subscribe this task to the watchdog
    for (;;) {
        esp_task_wdt_reset();  // feed the dog
        updateStateMachine();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void CommTask() {
    // Read steering ONCE per tick so the slew-rate limit in
    // getMeasuredSteering() applies exactly once per control cycle.
    cachedSteering = getMeasuredSteering();

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
}

void UITask() {
    uint8_t gear = 1;   // default: normal
    if      (current_drive_mode == DRIVE_LOW)  gear = 0;
    else if (current_drive_mode == DRIVE_HIGH) gear = 2;

    broadcastVehicleTelemetry(gear);

    if (VCS_VERBOSE_TASK_LOGS) {
        Serial.println(F("   -> UITask Finished!"));
    }
}

// ============================================================
//  TASK WRAPPERS
// ============================================================
void ESP32_CommLoop(void *pvParameters) {
    esp_task_wdt_add(NULL);
    for (;;) {
        esp_task_wdt_reset();
        CommTask();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void ESP32_UILoop(void *pvParameters) {
    esp_task_wdt_add(NULL);
    for (;;) {
        esp_task_wdt_reset();
        UITask();
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void DisplayLoop(void *pvParameters) {
    // Display task is non-critical for safety, but still subscribe so a
    // wedged I2C bus doesn't silently freeze the OLED forever.
    esp_task_wdt_add(NULL);
    for (;;) {
        esp_task_wdt_reset();
        // Use cachedSteering set by CommTask — avoids double slew-limiting.
        updateDisplay(getMeasuredRPM(), cachedSteering, current_drive_mode);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
    dacWrite(PIN_THROTTLE_OUT, 0);

    Serial.begin(115200);

    Serial.println(F("\n--- VCS v1.5: ESP32 38-PIN ---"));
    Serial.println(F("--- VCS v1.5 DIAGNOSTIC BOOT ---"));
    Serial.println(F("Testing modules one by one..."));

    // Initialize the watchdog ONCE here. Each task that wants protection
    // calls esp_task_wdt_add(NULL) in its body, then esp_task_wdt_reset()
    // every iteration. If any subscribed task fails to feed the dog
    // within WDT_TIMEOUT_SEC, the ESP32 reboots — this is the only
    // recovery path from a hung task now that FAULT_STATE is gone.
    esp_task_wdt_init(WDT_TIMEOUT_SEC, true);

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
```

