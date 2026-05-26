// ============================================================
//  vcs_state_machine.cpp
//
//  States: INIT → IDLE → MANUAL ↔ AUTONOMOUS
//  STOPPING_STATE removed — Jetson mission_fsm handles all
//  stop-line timing. VCS stays in AUTONOMOUS and physically
//  brakes when Jetson sends getTargetBrake() > 0. No VCS-side
//  timer duplicates the Jetson's 3s stop-line dwell.
//
//  MANUAL → AUTONOMOUS entry:
//    Jetson sends mode=1 AND no throttle AND no physical brake
//    → DMS_HOLD_REQUIRED_MS (1s) confirm → AUTONOMOUS
//    Deadman hold NOT required for entry (Jetson handles that gate)
//
//  AUTONOMOUS exit conditions:
//    - Throttle pedal pressed          → MANUAL immediately
//    - Physical brake switch pressed   → MANUAL immediately
//    - Deadman released                → 2s grace → MANUAL
//    - Jetson sends mode=0 or mode=2   → MANUAL
//    - Link lost (heartbeat timeout)   → MANUAL
//    - System fault                    → MANUAL
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

extern void hall_interrupts_attach();
extern void forceBrakeEngagement(bool engage);

// Throttle ADC value — defined in vcs_throttle.cpp
extern uint16_t current_throttle_adc;

static portMUX_TYPE faultMux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t     g_systemFaults = 0;

// Shared grace-period timer (reused across states)
static uint32_t s_graceStartTime = 0;
static uint32_t s_lastLogTime    = 0;
// IDLE brake-clear timer — tracks when IDLE was entered so we can
// wait for the actuator to fully retract before accepting input.
static uint32_t s_idleEntryMs    = 0;

// =========================================================
// Global state
// =========================================================
VcsState currentState      = INIT_STATE;
uint32_t dmsStartTime      = 0;
uint32_t stoppingStartTime = 0;   // retained for API compat, unused

// =========================================================
// INIT
// =========================================================
void initState_Machine() {
    currentState      = INIT_STATE;
    dmsStartTime      = 0;
    stoppingStartTime = 0;
    s_graceStartTime  = 0;
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
    // PRIORITY SAFETY OVERRIDES — checked every tick
    // ---------------------------------------------------------
    if (currentState == AUTONOMOUS_STATE) {
        uint32_t faults = getSystemFaults();
        bool linkLost   = !ansHeartbeatReceived();
        if (faults != VCS_FAULT_NONE || linkLost) {
            currentState     = MANUAL_STATE;
            s_graceStartTime = 0;
            vcs_log(linkLost ? "LINK lost -> MANUAL" : "FAULT -> MANUAL");
        }
    }

    // ---------------------------------------------------------
    // STATE TRANSITIONS
    // ---------------------------------------------------------
    switch (currentState) {

        // ── INIT ─────────────────────────────────────────────
        // ── INIT ─────────────────────────────────────────────
        // Brakes ON. Wait for minimum boot time, then accept a
        // DMS press to signal driver is seated → IDLE.
        case INIT_STATE: {
            static bool s_dms_seen = false;
            if (isDeadmanActive()) s_dms_seen = true;   // latch: brief press is enough

            if (millis() > 2000 && s_dms_seen) {
                currentState = IDLE_STATE;
                s_dms_seen   = false;
                vcs_log("INIT: driver present -> IDLE");
            }
            if (millis() > 2000 && millis() - s_lastLogTime > 3000) {
                vcs_log("INIT: press deadman to proceed");
                s_lastLogTime = millis();
            }
            break;
        }

        // ── IDLE ─────────────────────────────────────────────
        // Brakes are retracting. Nothing moves until actuator is
        // fully clear (BRAKE_RETRACT_MS + 200ms buffer).
        // After brakes clear:
        //   - throttle pedal OR brake button → MANUAL
        //   - DMS hold + Jetson AUTO → AUTONOMOUS countdown
        // No DMS required in MANUAL mode — DMS is only for AUTO.
        case IDLE_STATE: {
            const uint32_t BRAKE_CLEAR_MS = BRAKE_RETRACT_MS + 200u;
            bool brakes_clear = (s_idleEntryMs > 0) &&
                                 (millis() - s_idleEntryMs >= BRAKE_CLEAR_MS);

            if (!brakes_clear) {
                // Still retracting — log progress, block all transitions
                if (millis() - s_lastLogTime > 500) {
                    uint32_t remaining = BRAKE_CLEAR_MS -
                                         (millis() - s_idleEntryMs);
                    Serial.printf("[IDLE] Brakes retracting... %lums remaining\n",
                                  (unsigned long)remaining);
                    s_lastLogTime = millis();
                }
                break;
            }

            // Brakes fully clear — log once
            static bool s_idle_ready_logged = false;
            if (!s_idle_ready_logged) {
                vcs_log("IDLE: brakes clear — ready (throttle=MANUAL, DMS+AUTO=AUTONOMOUS)");
                s_idle_ready_logged = true;
            }

            // Driver input → MANUAL
            if (isThrottlePedalPressed() || isPhysicalBrakePressed()) {
                currentState         = MANUAL_STATE;
                dmsStartTime         = 0;
                s_idle_ready_logged  = false;
                vcs_log("IDLE: driver input -> MANUAL");
                break;
            }

            // DMS hold + Jetson AUTO → AUTONOMOUS countdown
            bool jetsonWantsAuto = (getANSCommandMode() == 1 ||
                                    getANSCommandMode() == 3);
            if (isDeadmanActive() && jetsonWantsAuto) {
                if (dmsStartTime == 0) {
                    dmsStartTime = millis();
                    vcs_log("IDLE: DMS + Jetson ready — AUTO countdown");
                }
                if (millis() - dmsStartTime > DMS_HOLD_REQUIRED_MS) {
                    currentState        = AUTONOMOUS_STATE;
                    dmsStartTime        = 0;
                    s_idle_ready_logged = false;
                    vcs_log("FSM: IDLE -> AUTONOMOUS");
                }
            } else {
                if (dmsStartTime != 0) dmsStartTime = 0;
            }
            break;
        }

        // ── MANUAL ───────────────────────────────────────────
        case MANUAL_STATE: {
            bool jetsonWantsAuto  = (getANSCommandMode() == 1 ||
                                     getANSCommandMode() == 3);
            bool driverInputActive = isThrottlePedalPressed() ||
                                     isPhysicalBrakePressed();

            // ── DEBUG: print entry condition values every 2s ──────────
            {
                static uint32_t lastDbg = 0;
                if (millis() - lastDbg > 2000) {
                    lastDbg = millis();
                    Serial.printf(
                        "[AUTO_ENTRY] dms=%d mode=%d throttle_raw=%u throttle_pressed=%d "
                        "brake_pressed=%d jetsonAuto=%d driverInput=%d\n",
                        (int)isDeadmanActive(),
                        (int)getANSCommandMode(),
                        (unsigned)current_throttle_adc,
                        (int)isThrottlePedalPressed(),
                        (int)isPhysicalBrakePressed(),
                        (int)jetsonWantsAuto,
                        (int)driverInputActive
                    );
                }
            }

            if (isDeadmanActive() && jetsonWantsAuto && !driverInputActive) {
                if (dmsStartTime == 0) {
                    dmsStartTime = millis();
                    vcs_log("AUTO: DMS held + Jetson ready — countdown started");
                }
                if (millis() - dmsStartTime > DMS_HOLD_REQUIRED_MS) {
                    currentState     = AUTONOMOUS_STATE;
                    dmsStartTime     = 0;
                    s_graceStartTime = 0;
                    vcs_log("FSM: -> AUTONOMOUS");
                }
            } else {
                if (dmsStartTime != 0) {
                    dmsStartTime = 0;
                    if (driverInputActive) vcs_log("Driver input: countdown reset");
                }
                if (!jetsonWantsAuto && millis() - s_lastLogTime >= 2000) {
                    vcs_log("WAITING: Jetson must send AUTO (mode=1)");
                    s_lastLogTime = millis();
                }
            }
            break;
        }

        // ── AUTONOMOUS ───────────────────────────────────────
        case AUTONOMOUS_STATE: {

            // ── DEBUG: print exit condition values every 2s ──────────
            {
                static uint32_t lastDbg2 = 0;
                if (millis() - lastDbg2 > 2000) {
                    lastDbg2 = millis();
                    Serial.printf(
                        "[AUTO_EXIT_CHECK] throttle_raw=%u throttle=%d brake=%d "
                        "dms=%d mode=%d grace=%lums\n",
                        (unsigned)current_throttle_adc,
                        (int)isThrottlePedalPressed(),
                        (int)isPhysicalBrakePressed(),
                        (int)isDeadmanActive(),
                        (int)getANSCommandMode(),
                        s_graceStartTime ? (unsigned long)(millis() - s_graceStartTime) : 0UL
                    );
                }
            }

            // 1. Throttle pedal → MANUAL immediately (no grace)
            if (isThrottlePedalPressed()) {
                currentState     = MANUAL_STATE;
                s_graceStartTime = 0;
                vcs_log("SAFETY: throttle pedal -> MANUAL");
                break;
            }

            // 2. Physical brake switch → MANUAL immediately
            //    (driver is physically braking = takeover)
            //    Note: Jetson brake COMMAND (getTargetBrake() > 0) does NOT
            //    change FSM state — it only actuates the brake hardware.
            //    Stop-line timing is handled by Jetson mission_fsm.
            if (isPhysicalBrakePressed()) {
                currentState     = MANUAL_STATE;
                s_graceStartTime = 0;
                vcs_log("SAFETY: brake switch -> MANUAL");
                break;
            }

            // 3. Deadman released → 2s grace → MANUAL
            if (!isDeadmanActive()) {
                if (s_graceStartTime == 0) {
                    s_graceStartTime = millis();
                    vcs_log("DMS released — 2s grace started");
                }
                if (millis() - s_graceStartTime > 2000u) {
                    currentState     = MANUAL_STATE;
                    s_graceStartTime = 0;
                    vcs_log("SAFETY: deadman released -> MANUAL");
                }
            } else {
                s_graceStartTime = 0;   // re-held: reset grace timer
            }

            // 4. Jetson explicit soft-exit (mode=2 only).
            //    mode=0 (IDLE) is transient during FSM state changes — do NOT
            //    exit on mode=0 or the VCS oscillates when Jetson briefly idles.
            if (currentState == AUTONOMOUS_STATE && getANSCommandMode() == 2) {
                currentState     = MANUAL_STATE;
                s_graceStartTime = 0;
                vcs_log("LINK: Jetson sent MANUAL -> exiting AUTONOMOUS");
            }
            break;
        }

        // STOPPING_STATE intentionally removed.
        // Brake actuation in AUTONOMOUS follows getTargetBrake() in the
        // brake task directly — no FSM state change needed for stop lines.

        default:
            break;
    }

    // ---------------------------------------------------------
    // ENTRY ACTIONS (run once per state change)
    // ---------------------------------------------------------
    if (currentState != lastState) {
        if (currentState == IDLE_STATE) {
            // IDLE entry: start brake retraction and record timestamp.
            // Nothing moves until s_idleEntryMs + BRAKE_RETRACT_MS + 200ms.
            s_idleEntryMs = millis();
            forceBrakeEngagement(false);   // begin retract
            vcs_log("IDLE entered: brakes retracting");
            hall_interrupts_attach();
        } else if (!isDrivingState()) {
            forceBrakeEngagement(true);    // INIT / FAULT: brakes ON
            s_idleEntryMs = 0;
        }
        Serial.print(F("VCS_STATE_MACHINE: -> "));
        Serial.println(getStateName(currentState));
        lastState = currentState;
    }
}

// =========================================================
// FAULT MANAGEMENT
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
        case STOPPING_STATE:   return "STOPPING";  // kept for API compat
        default:               return "UNKNOWN";
    }
}

bool isAutonomousMode()        { return currentState == AUTONOMOUS_STATE; }
uint32_t getDMSHoldStartTime() { return dmsStartTime; }
bool isDrivingState() {
    return (currentState == MANUAL_STATE || currentState == AUTONOMOUS_STATE);
}