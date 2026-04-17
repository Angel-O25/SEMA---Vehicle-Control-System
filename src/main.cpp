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
//  BOARD-SPECIFIC INCLUDES
// ============================================================
#ifdef NANO_33_BLE
    #include <mbed.h>
    using namespace rtos;
    using namespace mbed;

    Thread control_thread(osPriorityRealtime);
    Thread comm_thread(osPriorityHigh);
    Thread ui_thread(osPriorityNormal);

#elif defined(NANO_ATMEGA328)
    #include <SoftwareSerial.h>
    #include <avr/wdt.h>

    static uint32_t lastControlTime = 0;
    static uint32_t lastCommTime    = 0;
    static uint32_t lastUITime      = 0;
#endif

// ============================================================
//  TASK DEFINITIONS (shared logic, board-specific scheduling)
// ============================================================

// --- 1. CONTROL TASK (1 kHz / 1 ms) ---
void ControlTask() {
    updateHallCalculations();
    updateThrottle(getMeasuredRPM(), getTargetRPM());

    static uint8_t steerDivider = 0;
    if (++steerDivider >= 10) {
        updateSteeringPID(getTargetSteering(), isAutonomousMode());
        steerDivider = 0;
    }

    #ifdef NANO_33_BLE
        Watchdog::get_instance().kick();
    #elif defined(NANO_ATMEGA328)
        wdt_reset();
    #endif
}

// --- 2. COMM & STATE TASK (100 Hz / 10 ms) ---
void CommTask() {
    updateDeadman();
    updateLowBrake();
    updateThreeSpeed();
    updateReverse();
    updateUART();
    updateStateMachine(0);
    updateRelays(isAutonomousMode());
}

// --- 3. UI & TELEMETRY TASK (20 Hz / 50 ms) ---
void UITask() {
    broadcastVehicleTelemetry();
    updateDisplay(getMeasuredRPM(), getMeasuredSteering(), current_drive_mode);
}

// ============================================================
//  NANO 33 BLE — MBED THREAD WRAPPERS
// ============================================================
#ifdef NANO_33_BLE
void ControlTaskThread() {
    auto lastWakeTime = Kernel::Clock::now();
    for (;;) {
        ControlTask();
        ThisThread::sleep_until(lastWakeTime + std::chrono::milliseconds(1));
        lastWakeTime = Kernel::Clock::now();
    }
}

void CommTaskThread() {
    for (;;) {
        CommTask();
        ThisThread::sleep_for(std::chrono::milliseconds(10));
    }
}

void UITaskThread() {
    for (;;) {
        UITask();
        ThisThread::sleep_for(std::chrono::milliseconds(50));
    }
}
#endif

// ============================================================
//  SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(1000);

    #ifdef NANO_33_BLE
        Serial.println("--- VCS v1.5 SEM AUTONOMOUS BOOTING (NANO 33 BLE) ---");
    #elif defined(NANO_ATMEGA328)
        Serial.println("--- VCS v1.5 SEM AUTONOMOUS BOOTING (ATMEGA328 NANO) ---");
    #endif

    // Hardware Module Initialization (identical for both boards)
    initState_Machine();
    initUART();
    initThrottle();
    initLowBrake();
    initDeadman();
    initRelays();
    initSteering();
    initHallSensors();
    initThreeSpeed();
    initReverse();
    initDisplay();

    // Board-specific watchdog and scheduler startup
    #ifdef NANO_33_BLE
        Watchdog::get_instance().start(2000);
        control_thread.start(ControlTaskThread);
        comm_thread.start(CommTaskThread);
        ui_thread.start(UITaskThread);

    #elif defined(NANO_ATMEGA328)
        wdt_enable(WDTO_2S);
    #endif
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
    #ifdef NANO_33_BLE
        // Mbed OS owns scheduling; loop() is just the idle thread
        ThisThread::sleep_for(std::chrono::milliseconds(1000));

    #elif defined(NANO_ATMEGA328)
        uint32_t now = millis();

        if (now - lastControlTime >= 1) {
            lastControlTime = now;
            ControlTask();
        }

        if (now - lastCommTime >= 10) {
            lastCommTime = now;
            CommTask();
        }

        if (now - lastUITime >= 50) {
            lastUITime = now;
            UITask();
        }
    #endif
}