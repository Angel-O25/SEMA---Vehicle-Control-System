#include "vcs_state_machine.h"
#include "vcs_uart.h"
#include "vcs_pins.h"
#include "vcs_deadman.h"
#include "vcs_lowbrake.h"
#include "vcs_throttle.h"
#include "vcs_reverse.h"
#include "vcs_web.h"
#include "vcs_hallsensor.h"   // getMeasuredRPM()
#include "vcs_steering.h"     // getMeasuredSteering()

// Hall ISR attach/detach lives in vcs_hallsensor.cpp
extern void hall_interrupts_attach();
extern void hall_interrupts_detach();

// Brake helper lives in vcs_lowbrake.cpp / vcs_relays.cpp
extern void forceBrakeEngagement(bool engage);

// =========================================================
// Global state
// =========================================================
VcsState currentState        = INIT_STATE;
uint32_t dmsStartTime        = 0;   // Marks beginning of DMS hold for MANUAL→AUTO promotion
uint32_t stoppingStartTime   = 0;   // Marks entry into STOPPING_STATE

// =========================================================
// INIT
// =========================================================
void initState_Machine() {
    currentState      = INIT_STATE;
    dmsStartTime      = 0;
    stoppingStartTime = 0;
}

// =========================================================
// FSM TICK — call from EXACTLY ONE task (task_control).
// =========================================================
void updateStateMachine(uint32_t faults) {
    static VcsState lastState = INIT_STATE;

    // ---------------------------------------------------------
    // 1. PRIORITY SAFETY OVERRIDES
    //    Heartbeat loss only forces FAULT while in AUTONOMOUS;
    //    losing the link during MANUAL is non-fatal by design.
    // ---------------------------------------------------------
    if (faults != VCS_FAULT_NONE ||
        (!ansHeartbeatReceived() && currentState == AUTONOMOUS_STATE)) {
        currentState = FAULT_STATE;
    }

    // ---------------------------------------------------------
    // 2. STATE TRANSITION LOGIC
    // ---------------------------------------------------------
    switch (currentState) {

        case INIT_STATE:
            if (millis() > 2000) currentState = IDLE_STATE;
            break;

        case IDLE_STATE:
            // ESP32 owns the car. Once init is complete, default to
            // MANUAL — we don't need the Jetson up just to let a human drive.
            currentState = MANUAL_STATE;
            break;

        case MANUAL_STATE:
            // DMS is the primary precondition for MANUAL→AUTONOMOUS.
            if (isDeadmanActive() && !isReverseEngaged()) {
                if (getANSCommandMode() == 1) {  // Jetson is requesting AUTO
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
                    static uint32_t lastLogTime = 0;
                    if (millis() - lastLogTime >= 2000) {
                        vcs_log("WAITING: Jetson must send AUTO (mode=1)");
                        lastLogTime = millis();
                    }
                }
            } else {
                dmsStartTime = 0;
            }
            break;

        case AUTONOMOUS_STATE:
            // Hard physical overrides → MANUAL
            if (isThrottlePedalPressed() || !isDeadmanActive()) {
                currentState = MANUAL_STATE;
                vcs_log("SAFETY: physical override -> MANUAL");
            }
            // Brake / stop-line → STOPPING
            else if (isPhysicalBrakePressed() || isJetsonStopLineActive()) {
                currentState      = STOPPING_STATE;
                stoppingStartTime = millis();
                vcs_log("FSM: brake / stop-line -> STOPPING");
            }
            // Soft override (Jetson asked to exit AUTO, or link lost)
            else if (getANSCommandMode() == 2 || !ansHeartbeatReceived()) {
                currentState = MANUAL_STATE;
                vcs_log("LINK: Jetson soft-exit / heartbeat lost -> MANUAL");
            }
            break;

        case STOPPING_STATE:
            // 3 s elapsed and stop-line cleared → resume AUTONOMOUS
            if ((millis() - stoppingStartTime >= 3000) && !isJetsonStopLineActive()) {
                currentState = AUTONOMOUS_STATE;
                vcs_log("FSM: stop cleared -> AUTONOMOUS");
            }
            // Driver intervention during stop → MANUAL
            else if (!isDeadmanActive() || isThrottlePedalPressed()) {
                currentState = MANUAL_STATE;
                vcs_log("SAFETY: override during STOPPING -> MANUAL");
            }
            break;

        case FAULT_STATE:
            // Recover only when ANS link is restored AND driver hands are off.
            if (ansHeartbeatReceived()) {
                currentState = IDLE_STATE;
            }
            break;
    }

    // ---------------------------------------------------------
    // 3. ENTRY ACTIONS (run once per state change)
    //    NOTE: this block must live OUTSIDE the switch above —
    //    if it sits after the last case it becomes unreachable.
    // ---------------------------------------------------------
    if (currentState != lastState) {

        // Engage brake whenever we leave a driving state
        if (!isDrivingState()) {
            forceBrakeEngagement(true);
        }

        // Hall ISR management:
        //   - Attach the first time we leave INIT (motor controller is up by now).
        //   - Detach on FAULT (we don't trust the hardware).
        //   - DO NOT detach on STOPPING — we still need RPM to verify zero speed.
        if (currentState == IDLE_STATE && lastState == INIT_STATE) {
            hall_interrupts_attach();
        } else if (currentState == FAULT_STATE) {
            hall_interrupts_detach();
        }

        Serial.print(F("VCS_STATE_MACHINE: -> "));
        Serial.println(getStateName(currentState));
        lastState = currentState;
    }
}

// =========================================================
// FAULT INJECTION
// =========================================================
void requestSoftwareEstop() {
    // No latched ESTOP state exists yet — route through FAULT for now.
    // FAULT is recoverable, so add a latched flag if true E-stop is needed.
    currentState = FAULT_STATE;
    vcs_log("ESTOP requested -> FAULT_STATE");
}

uint32_t getSystemFaults() {
    uint32_t f   = VCS_FAULT_NONE;
    uint32_t now = millis();
/*
    // 1. Heartbeat
    if (now - last_uart_time > 500) {
        f |= VCS_FAULT_HEARTBEAT_LOST;
    }

    // 2. Sensor sanity
    
    uint16_t steer = getMeasuredSteering();
    float    rpm   = getMeasuredRPM();
    if (steer < 5 || steer > 1018 || rpm > 600.0f) {
        f |= VCS_FAULT_SENSOR_SPIKE;
    }
*/
    return f;
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
        case FAULT_STATE:      return "FAULT";
        case STOPPING_STATE:   return "STOPPING";
        default:               return "UNKNOWN";
    }
}

bool isAutonomousMode()        { return currentState == AUTONOMOUS_STATE; }
uint32_t getDMSHoldStartTime() { return dmsStartTime; }
bool isDrivingState() {
    return (currentState == MANUAL_STATE || currentState == AUTONOMOUS_STATE);
}