// ============================================================
//  vcs_state_machine.cpp — Rev fixes applied
//
//  CHANGES FROM ORIGINAL (fixes only — no new features):
//
//  FIX #2  — updateStateMachine() no longer takes a faults parameter.
//             It reads g_systemFaults directly. Eliminates risk of
//             callers passing a stale or wrong fault word.
//             clearFault() and clearAllFaults() added — g_systemFaults
//             was previously write-only (no way to clear a resolved fault).
//
//  FIX #3  — FAULT_STATE recovery now requires BOTH heartbeat restored
//             AND g_systemFaults == VCS_FAULT_NONE. Previously recovered
//             on heartbeat alone, so a hardware fault would clear just
//             because the Jetson reconnected.
//
//  FIX #4  — s_signalLostTime and s_lastLogTime promoted from static
//             locals inside the MANUAL_STATE case to file-scope statics.
//             Static locals inside a switch case cannot be explicitly
//             reset on state re-entry (e.g. AUTONOMOUS → MANUAL).
//             File-scope statics are resettable in initState_Machine().
//
//  FIX #9  — Removed isJetsonStopLineActive() call. That function is
//             defined in vcs_uart.cpp and encodes stop-line semantics
//             inside the UART module. The check (getTargetBrake() > 0)
//             is now inlined here where the semantic decision belongs.
//             Remove isJetsonStopLineActive() declaration from vcs_uart.h.
//
//  FIX #10 — requestSoftwareEstop() now sets a latched flag. Once set,
//             FAULT_STATE will not self-recover. Prevents the original
//             FAULT → IDLE cycling after a commanded E-stop.
//             Only a power-cycle clears the latch.
//
//  NOTE: vcs_state_machine.h must be updated to match:
//    - updateStateMachine() signature: remove uint32_t faults parameter
//    - Add declarations: clearFault(uint16_t), clearAllFaults()
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

static uint32_t g_systemFaults = 0;
portMUX_TYPE logMux = portMUX_INITIALIZER_UNLOCKED;
// =========================================================
// FIX #4: Promoted static locals from MANUAL_STATE switch
// case to file scope. Static locals inside a switch case
// persist correctly in C++ but cannot be explicitly reset
// on state re-entry (e.g. AUTONOMOUS → MANUAL transitions).
// File-scope statics with clear names are resettable in
// initState_Machine().
// =========================================================
static uint32_t s_signalLostTime = 0;
static uint32_t s_lastLogTime    = 0;

// FIX #10: Latched E-stop flag. Once set by requestSoftwareEstop(),
// FAULT_STATE will not recover until power-cycle.
// Shell Eco-marathon rules require a commanded E-stop to be
// non-auto-recoverable.
static bool s_estopLatched = false;

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
    // FIX #4: Explicitly reset promoted statics on re-init.
    s_signalLostTime  = 0;
    s_lastLogTime     = 0;
    // FIX #10: E-stop latch intentionally survives re-init.
    // Only a power-cycle clears it.
}

// =========================================================
// FSM TICK — call from EXACTLY ONE task (task_control).
//
// FIX #2: Signature changed — faults parameter removed.
// g_systemFaults is read directly. Update vcs_state_machine.h
// to match: void updateStateMachine();
// =========================================================
void updateStateMachine() {
    static VcsState lastState = INIT_STATE;

    // ---------------------------------------------------------
    // 1. PRIORITY SAFETY OVERRIDES
    //    FIX #2: Read g_systemFaults directly instead of the
    //    caller-supplied faults parameter that was previously
    //    passed in (and could be stale).
    //    Heartbeat loss only forces FAULT while in AUTONOMOUS;
    //    losing the link during MANUAL is non-fatal by design.
    // ---------------------------------------------------------
    if (g_systemFaults != VCS_FAULT_NONE ||
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

        case MANUAL_STATE: {
            // FIX #4: s_signalLostTime and s_lastLogTime are now
            // file-scope statics (see top of file), resettable on
            // state re-entry. Behaviour is otherwise unchanged.
            if (isDeadmanActive() && !isReverseEngaged()) {

                s_signalLostTime = 0;

                if (getANSCommandMode() == 1) {
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
                // 50ms grace period to prevent switch bounce resetting the timer
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
            // Hard physical overrides → MANUAL
            if (isThrottlePedalPressed() || !isDeadmanActive()) {
                currentState = MANUAL_STATE;
                vcs_log("SAFETY: physical override -> MANUAL");
            }
            // FIX #9: Replaced isJetsonStopLineActive() with the inline
            // check it contained. Stop-line semantics belong in the state
            // machine, not in vcs_uart.cpp. Remove isJetsonStopLineActive()
            // from vcs_uart.h and vcs_uart.cpp.
            else if (isPhysicalBrakePressed() || (getTargetBrake() > 0)) {
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
            // FIX #9: Same inline check replacing isJetsonStopLineActive().
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

        case FAULT_STATE:
            // FIX #3: Recovery now requires ALL three conditions:
            //   1. E-stop not latched (FIX #10)
            //   2. Heartbeat restored
            //   3. All fault bits cleared (was missing — previously
            //      recovered on heartbeat alone, so a hardware fault
            //      would clear just because the Jetson reconnected)
            if (!s_estopLatched &&
                ansHeartbeatReceived() &&
                g_systemFaults == VCS_FAULT_NONE) {
                currentState = IDLE_STATE;
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
        //   - Detach on FAULT (we don't trust the hardware).
        //   - DO NOT detach on STOPPING — we still need RPM.
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
// FAULT MANAGEMENT
// FIX #2: clearFault() and clearAllFaults() added.
// g_systemFaults was previously write-only — once a fault
// bit was set via triggerFault() there was no way to clear
// it even after the condition resolved.
// =========================================================
uint32_t getSystemFaults() {
    portENTER_CRITICAL(&logMux);
    return g_systemFaults;
}

void triggerFault(uint16_t fault_code) {
    g_systemFaults |= fault_code;
}

// FIX #2: Clear a specific fault bit. Call after verifying
// the fault condition is resolved.
void clearFault(uint16_t fault_code) {
    g_systemFaults &= ~(uint32_t)fault_code;
}

// FIX #2: Clear all fault bits. Call only after verifying
// all fault conditions are fully resolved.
void clearAllFaults() {
    g_systemFaults = VCS_FAULT_NONE;
}

// =========================================================
// ESTOP
// FIX #10: requestSoftwareEstop() now sets a latched flag.
// Once latched, FAULT_STATE will not recover automatically.
// The original code routed through FAULT which is recoverable
// — that is not a true E-stop. Power-cycle required to clear.
// =========================================================
void requestSoftwareEstop() {
    s_estopLatched = true;
    currentState   = FAULT_STATE;
    vcs_log("ESTOP latched -> FAULT_STATE (power-cycle to clear)");
}

// =========================================================
// HELPERS (unchanged)
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

bool isEstopLatched() {
    return s_estopLatched;
}