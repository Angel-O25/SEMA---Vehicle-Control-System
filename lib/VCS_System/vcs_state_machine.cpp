// ============================================================
//  vcs_state_machine.cpp — Option A revision
//
//  CHANGES FROM PREVIOUS REVISION:
//
//  • FAULT_STATE removed entirely. The watchdog timer (configured in
//    main.cpp) handles task hangs. Recoverable soft faults — UART CRC,
//    heartbeat loss while autonomous, sensor dropouts — now demote the
//    FSM to MANUAL_STATE instead of locking it in FAULT.
//    Driver retakes control immediately, no actuator glitch.
//
//  • requestSoftwareEstop() / isEstopLatched() / s_estopLatched removed.
//    No software E-stop button is wired to the ESP32. The hardware
//    E-stop (if present) bypasses the embedded system on the power bus.
//
//  • triggerFault / getSystemFaults / clearFault / clearAllFaults retained
//    for telemetry visibility. Jetson dashboard can still show which
//    fault bits are active even though they no longer drive a dedicated
//    FAULT state.
//
//  • getSystemFaults() deadlock fixed — previous version took the mutex
//    but never released it, so the second caller hung forever.
//    triggerFault / clearFault / clearAllFaults now also mutex-protected
//    for consistency.
//
//  • Hall ISR detach removed from state-entry actions. Previously the
//    ISR detached on FAULT_STATE entry; with FAULT_STATE gone, the
//    ISR stays attached across the lifetime of the firmware after the
//    initial INIT -> IDLE transition.
// ============================================================

#include "vcs_state_machine.h"
#include "vcs_uart.h"
#include "vcs_pins.h"
#include "vcs_deadman.h"
#include "vcs_lowbrake.h"
#include "vcs_throttle.h"
#include "vcs_reverse.h"
#include "vcs_web.h"
#include "vcs_hallsensor.h"
#include "vcs_steering.h"
#include "vcs_constants.h"

// Hall ISR attach/detach lives in vcs_hallsensor.cpp
extern void hall_interrupts_attach();
extern void hall_interrupts_detach();

// Brake helper lives in vcs_lowbrake.cpp / vcs_relays.cpp
extern void forceBrakeEngagement(bool engage);

// Fault register — protected by faultMux for thread-safe access from any task.
static portMUX_TYPE faultMux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t     g_systemFaults = 0;

// File-scope statics for MANUAL_STATE timer logic.
static uint32_t s_signalLostTime = 0;
static uint32_t s_lastLogTime    = 0;

// =========================================================
// Global state
// =========================================================
VcsState currentState      = INIT_STATE;
uint32_t dmsStartTime      = 0;
uint32_t stoppingStartTime = 0;

// =========================================================
// INIT
// =========================================================
void initState_Machine() {
    currentState      = INIT_STATE;
    dmsStartTime      = 0;
    stoppingStartTime = 0;
    s_signalLostTime  = 0;
    s_lastLogTime     = 0;

    portENTER_CRITICAL(&faultMux);
    g_systemFaults = VCS_FAULT_NONE;
    portEXIT_CRITICAL(&faultMux);
}

// =========================================================
// FSM TICK — call from EXACTLY ONE task (task_control).
// =========================================================
void updateStateMachine() {
    static VcsState lastState = INIT_STATE;

    // ---------------------------------------------------------
    // 1. PRIORITY SAFETY OVERRIDES
    //
    //    Option A behaviour: any active fault OR heartbeat loss
    //    while AUTONOMOUS demotes the FSM to MANUAL.
    //    No FAULT_STATE — driver takes over immediately.
    //
    //    Faults during MANUAL/IDLE/STOPPING are non-fatal; they
    //    are visible in telemetry but do not force a state change.
    // ---------------------------------------------------------
    if (currentState == AUTONOMOUS_STATE) {
        uint32_t faults = getSystemFaults();
        bool linkLost   = !ansHeartbeatReceived();

        if (faults != VCS_FAULT_NONE || linkLost) {
            currentState = MANUAL_STATE;
            vcs_log(linkLost ? "LINK lost in AUTO -> MANUAL"
                              : "FAULT detected in AUTO -> MANUAL");
        }
    }

    // ---------------------------------------------------------
    // 2. STATE TRANSITION LOGIC
    // ---------------------------------------------------------
    switch (currentState) {

        case INIT_STATE:
            if (millis() > 2000) currentState = IDLE_STATE;
            break;

        case IDLE_STATE:
            // ESP32 owns the car. Once init completes, default to MANUAL —
            // a human driver doesn't need the Jetson up to drive.
            currentState = MANUAL_STATE;
            break;

        case MANUAL_STATE: {
            if (isDeadmanActive() && !isReverseEngaged()) {
                s_signalLostTime = 0;

                if (getANSCommandMode() == 1 || getANSCommandMode() == 3) {
                    if (dmsStartTime == 0) {
                        dmsStartTime = millis();
                        vcs_log("DMS HELD: 1s countdown for AUTO...");
                    }
                    if (millis() - dmsStartTime > DMS_HOLD_REQUIRED_MS) {
                        currentState = AUTONOMOUS_STATE;
                        dmsStartTime = 0;
                        vcs_log("FSM: -> AUTONOMOUS");
                    }
                } else {
                    if (millis() - s_lastLogTime >= 2000) {
                        vcs_log("WAITING: Jetson must send AUTO (mode=1)");
                        s_lastLogTime = millis();
                    }
                }
            } else {
                // 50ms grace period to prevent switch bounce resetting timer
                if (dmsStartTime != 0) {
                    if (s_signalLostTime == 0) s_signalLostTime = millis();
                    if (millis() - s_signalLostTime > 50) {
                        dmsStartTime     = 0;
                        s_signalLostTime = 0;
                    }
                } else {
                    s_signalLostTime = 0;
                }
            }
            break;
        }

        case AUTONOMOUS_STATE:
            // Hard override: throttle pedal → MANUAL immediately (no grace)
            if (isThrottlePedalPressed()) {
                currentState = MANUAL_STATE;
                vcs_log("SAFETY: throttle pedal -> MANUAL");
            }
            // Deadman released: 2s grace period before dropping to MANUAL.
            // Prevents a momentary finger slip from aborting the run.
            else if (!isDeadmanActive()) {
                if (s_signalLostTime == 0) {
                    s_signalLostTime = millis();
                    vcs_log("DMS released — 2s grace started");
                }
                if (millis() - s_signalLostTime > 2000u) {
                    currentState     = MANUAL_STATE;
                    s_signalLostTime = 0;
                    vcs_log("SAFETY: deadman released -> MANUAL");
                }
            } else {
                // Deadman re-engaged: reset grace timer
                s_signalLostTime = 0;
            }
            // Brake / stop-line → STOPPING
            if (currentState == AUTONOMOUS_STATE &&
                (isPhysicalBrakePressed() || (getTargetBrake() > 10))) {
                currentState      = STOPPING_STATE;
                stoppingStartTime = millis();
                vcs_log("FSM: brake / stop-line -> STOPPING");
            }
            // Jetson soft-exit
            if (currentState == AUTONOMOUS_STATE && getANSCommandMode() == 2) {
                currentState = MANUAL_STATE;
                vcs_log("LINK: Jetson soft-exit -> MANUAL");
            }
            break;

        case STOPPING_STATE:
            // 3 s elapsed and stop-line cleared → resume AUTONOMOUS
            if ((millis() - stoppingStartTime >= 3000) && !(getTargetBrake() > 0)) {
                currentState = AUTONOMOUS_STATE;
                vcs_log("FSM: stop cleared -> AUTONOMOUS");
            }
            // Driver intervention during stop → MANUAL
            else if (!isDeadmanActive() || isThrottlePedalPressed()) {
                currentState = MANUAL_STATE;
                vcs_log("SAFETY: override during STOPPING -> MANUAL");
            }
            break;
    }

    // ---------------------------------------------------------
    // 3. ENTRY ACTIONS (run once per state change)
    // ---------------------------------------------------------
    if (currentState != lastState) {

        // Engage brake whenever we leave a driving state
        if (!isDrivingState()) {
            forceBrakeEngagement(true);
        }

        // Hall ISR management:
        //   - Attach the first time we leave INIT (MC is up by now).
        //   - No detach on fault — FAULT_STATE no longer exists,
        //     and we want continuous RPM data even during MANUAL.
        if (currentState == IDLE_STATE && lastState == INIT_STATE) {
            hall_interrupts_attach();
        }

        Serial.print(F("VCS_STATE_MACHINE: -> "));
        Serial.println(getStateName(currentState));
        lastState = currentState;
    }
}

// =========================================================
// FAULT MANAGEMENT (telemetry only — does not drive FSM state)
// All accesses mutex-protected so any task can read/write safely.
// =========================================================
uint32_t getSystemFaults() {
    portENTER_CRITICAL(&faultMux);
    uint32_t f = g_systemFaults;
    portEXIT_CRITICAL(&faultMux);
    return f;
}

void triggerFault(uint16_t fault_code) {
    portENTER_CRITICAL(&faultMux);
    g_systemFaults |= (uint32_t)fault_code;
    portEXIT_CRITICAL(&faultMux);
}

void clearFault(uint16_t fault_code) {
    portENTER_CRITICAL(&faultMux);
    g_systemFaults &= ~(uint32_t)fault_code;
    portEXIT_CRITICAL(&faultMux);
}

void clearAllFaults() {
    portENTER_CRITICAL(&faultMux);
    g_systemFaults = VCS_FAULT_NONE;
    portEXIT_CRITICAL(&faultMux);
}

// =========================================================
// HELPERS
// =========================================================
const char* getStateName(VcsState state) {
    switch (state) {
        case INIT_STATE:       return "INIT";
        case IDLE_STATE:       return "IDLE";
        case MANUAL_STATE:     return "MANUAL";
        case AUTONOMOUS_STATE: return "AUTONOMOUS";
        case STOPPING_STATE:   return "STOPPING";
        default:               return "UNKNOWN";
    }
}

bool isAutonomousMode()        { return currentState == AUTONOMOUS_STATE; }
uint32_t getDMSHoldStartTime() { return dmsStartTime; }
bool isDrivingState() {
    return (currentState == MANUAL_STATE || currentState == AUTONOMOUS_STATE);
}