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

// ============================================================
//  BOARD-SPECIFIC SCHEDULING & INCLUDES
// ============================================================
#if defined(NANO_33_BLE)
    #include <mbed.h>
    using namespace rtos; 
    using namespace std::chrono_literals; 

    rtos::Thread control_thread(osPriorityRealtime);
    rtos::Thread comm_thread(osPriorityHigh);
    rtos::Thread ui_thread(osPriorityNormal);

#elif defined(NANO_ATMEGA328)
    #include <avr/wdt.h>
    static uint32_t lastControlTime = 0;
    static uint32_t lastCommTime    = 0;
    static uint32_t lastUITime      = 0;

#elif defined(ESP32_VCS)
    TaskHandle_t ControlTaskHandle;
#endif

// ============================================================
//  TASK LOGIC
// ============================================================

void ControlTask() {
    updateHallCalculations();
    updateThrottle(getMeasuredRPM(), getTargetRPM());

    static uint8_t steerDivider = 0;
    if (++steerDivider >= 10) {
        updateSteeringPID(getTargetSteering(), isAutonomousMode());
        steerDivider = 0;
    }

    #if defined(NANO_33_BLE)
        mbed::Watchdog::get_instance().kick();
    #elif defined(NANO_ATMEGA328)
        wdt_reset();
    #endif
}

// Update this function above your setup()
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

void UITask() {
    Serial.println(F("   -> Running Telemetry..."));
    
    // Test 1: Let's see if the telemetry string builder is crashing the RAM
    broadcastVehicleTelemetry(); 

    Serial.println(F("   -> Running Display Update..."));
    
    // Test 2: Let's see if pushing pixels to the OLED is crashing the I2C bus or RAM
    //updateDisplay(getMeasuredRPM(), getMeasuredSteering(), current_drive_mode);

    Serial.println(F("   -> UITask Finished!"));
}

// ============================================================
//  THREAD WRAPPERS
// ============================================================
#if defined(NANO_33_BLE)
void ControlTaskThread() { while(1) { ControlTask(); rtos::ThisThread::sleep_for(1ms); } }
void CommTaskThread()    { while(1) { CommTask();    rtos::ThisThread::sleep_for(10ms); } }
void UITaskThread()      { while(1) { UITask();      rtos::ThisThread::sleep_for(50ms); } }

#elif defined(ESP32_VCS)
void ESP32_ControlLoop(void * pvParameters) {
    for(;;) { ControlTask(); vTaskDelay(1 / portTICK_PERIOD_MS); }
}
#endif

// ============================================================
//  SETUP
// ============================================================
void setup() {
    Serial.begin(115200); 
    delay(1000);

    #if defined(NANO_33_BLE)
        Serial.println(F("--- VCS v1.5: NANO 33 BLE ---"));
    #elif defined(ESP32_VCS)
        Serial.println(F("--- VCS v1.5: ESP32 38-PIN ---"));
    #elif defined(NANO_ATMEGA328)
        Serial.println(F("--- VCS v1.5: ATMEGA328 NANO ---"));
    #endif

    Serial.println(F("\n--- VCS v1.5 DIAGNOSTIC BOOT ---"));
    Serial.println(F("Testing modules one by one..."));

    // Single Initialization Phase
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
    // initDisplay(); 
    Serial.println(F("DISABLED FOR RAM STABILITY"));

    Serial.println(F("--- SURVIVED BOOT SEQUENCE ---"));

    // =========================================================
    // SECURITY FIX: Removed the duplicate initialization block 
    // that was causing hardware interrupts to re-trigger.
    // =========================================================

    #if defined(NANO_33_BLE)
        mbed::Watchdog::get_instance().start(2000);
        control_thread.start(ControlTaskThread);
        comm_thread.start(CommTaskThread);
        ui_thread.start(UITaskThread);
    #elif defined(ESP32_VCS)
        xTaskCreatePinnedToCore(ESP32_ControlLoop, "Control", 4096, NULL, 10, &ControlTaskHandle, 1);
    #endif
}

// ============================================================
//  LOOP
// ============================================================

// Update your loop() at the bottom
void loop() {
    #if defined(NANO_ATMEGA328)
        uint32_t now = millis();

        // Slowed down from 1ms to 1000ms (1 second) for debugging
        if (now - lastControlTime >= 1000) { 
            lastControlTime = now;
            Serial.println(F("[LOOP] Running ControlTask..."));
            ControlTask();
        }

        // Slowed down from 10ms to 1000ms
        if (now - lastCommTime >= 1000) { 
            lastCommTime = now;
            Serial.println(F("[LOOP] Running CommTask..."));
            CommTask();
        }

        // Slowed down from 50ms to 1000ms
        if (now - lastUITime >= 1000) { 
            lastUITime = now;
            Serial.println(F("[LOOP] Running UITask..."));
            UITask();
        }
    #endif
}