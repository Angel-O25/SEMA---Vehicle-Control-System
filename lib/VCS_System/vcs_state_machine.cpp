// ============================================================
//  vcs_state_machine.cpp — SIDLAK 2 VCS
//  Team Wired PH0017003 | Shell Eco-marathon 2026
//
//  Key change from previous version:
//  AUTONOMOUS entry NO LONGER requires Jetson to send Mode=1.
//  DMS hold alone triggers AUTONOMOUS from both IDLE and MANUAL.
//  Reason: cmd_vel timeout caused uart_bridge to drop Mode=1
//  briefly, so VCS never saw a stable Mode=1 and never transitioned.
//
//  New flow:
//    DMS held 1s in IDLE/MANUAL → VCS enters AUTONOMOUS
//    VCS sends State=3 → Jetson uart_bridge confirms AUTONOMOUS
//
//  AUTONOMOUS exits on: throttle pedal, brake switch,
//  DMS released (2s grace), Jetson sends Mode=2 (explicit manual)
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
extern uint16_t current_throttle_adc;

static portMUX_TYPE faultMux    = portMUX_INITIALIZER_UNLOCKED;
static uint32_t     g_systemFaults = 0;

static uint32_t s_graceStartTime = 0;
static uint32_t s_lastLogTime    = 0;
static uint32_t s_idleEntryMs    = 0;

// ── Global state ──────────────────────────────────────────────
VcsState currentState      = INIT_STATE;
uint32_t dmsStartTime      = 0;
uint32_t stoppingStartTime = 0;

// ─────────────────────────────────────────────────────────────
void initState_Machine() {
    currentState      = INIT_STATE;
    dmsStartTime      = 0;
    stoppingStartTime = 0;
    s_graceStartTime  = 0;
    s_lastLogTime     = 0;
    s_idleEntryMs     = 0;

    portENTER_CRITICAL(&faultMux);
    g_systemFaults = VCS_FAULT_NONE;
    portEXIT_CRITICAL(&faultMux);
}

// ─────────────────────────────────────────────────────────────
void updateStateMachine() {
    static VcsState lastState = INIT_STATE;

    // ── Priority safety overrides ─────────────────────────────
    if (currentState == AUTONOMOUS_STATE) {
        static uint32_t s_auto_entry_ms = 0;
        if (lastState != AUTONOMOUS_STATE) s_auto_entry_ms = millis();  // reset on entry

        bool grace = (millis() - s_auto_entry_ms < 3000);  // 3s grace period
        uint32_t faults = getSystemFaults();
        bool linkLost   = !ansHeartbeatReceived();

        if (!grace && (faults != VCS_FAULT_NONE || linkLost)) {
            currentState     = MANUAL_STATE;
            s_graceStartTime = 0;
            vcs_log(linkLost ? "LINK lost -> MANUAL" : "FAULT -> MANUAL");
        }
    }

    // ── State transitions ─────────────────────────────────────
    switch (currentState) {

    // ── INIT ─────────────────────────────────────────────────
    // Brakes ON. Wait 2s min boot, then either DMS pressed → IDLE.
    case INIT_STATE: {
        static bool s_dms_seen = false;

        // Either DMS = driver is seated
        if (isEitherDeadmanHeld()) s_dms_seen = true;

        if (millis() > 2000 && s_dms_seen) {
            currentState = IDLE_STATE;
            s_dms_seen   = false;
            vcs_log("INIT: driver present -> IDLE");
        }

        // Log every 3s so driver knows what to do
        if (millis() > 500 && millis() - s_lastLogTime > 3000) {
            s_lastLogTime = millis();
            Serial.printf("[INIT] Press DMS to proceed. L=%d R=%d (0=pressed)\r\n",
                          digitalRead(PIN_DMS_LEFT),
                          digitalRead(PIN_DMS_RIGHT));
        }
        break;
    }

    // ── IDLE ─────────────────────────────────────────────────
    // Brakes retracting. Block all transitions until actuator clears.
    // After brakes clear:
    //   throttle/brake button  → MANUAL
    //   DMS held 1s            → AUTONOMOUS  (no Jetson mode required)
    case IDLE_STATE: {
        const uint32_t BRAKE_CLEAR_MS = BRAKE_RETRACT_MS + 200u;
        bool brakes_clear = (s_idleEntryMs > 0) &&
                             (millis() - s_idleEntryMs >= BRAKE_CLEAR_MS);

        if (!brakes_clear) {
            if (millis() - s_lastLogTime > 400) {
                s_lastLogTime = millis();
                uint32_t rem  = BRAKE_CLEAR_MS - (millis() - s_idleEntryMs);
                Serial.printf("[IDLE] Brakes retracting... %lums\r\n",
                              (unsigned long)rem);
            }
            break;
        }

        // Brakes clear — log once
        static bool s_idle_ready_logged = false;
        if (!s_idle_ready_logged) {
            vcs_log("IDLE: brakes clear — throttle=MANUAL, hold DMS=AUTONOMOUS");
            s_idle_ready_logged = true;
        }

        // Throttle or brake button → MANUAL
        if (isThrottlePedalPressed() || isPhysicalBrakePressed()) {
            currentState        = MANUAL_STATE;
            dmsStartTime        = 0;
            s_idle_ready_logged = false;
            vcs_log("IDLE: driver input -> MANUAL");
            break;
        }

        // ── AUTONOMOUS entry: DMS hold ONLY ──────────────────
        // Removed jetsonWantsAuto — cmd_vel timeout was dropping Mode=1
        // causing VCS to never see a stable autonomous request.
        // VCS now enters AUTONOMOUS on DMS hold alone. Jetson follows
        // when it sees State=3 in the VCS TX packet.
        if (isDeadmanActive()) {
            if (dmsStartTime == 0) {
                dmsStartTime = millis();
                vcs_log("IDLE: both DMS held — AUTONOMOUS countdown 1s");
            }
            uint32_t held = millis() - dmsStartTime;
            if (millis() - s_lastLogTime > 200) {
                s_lastLogTime = millis();
                Serial.printf("[IDLE->AUTO] DMS held %lums / %ums\r\n",
                              (unsigned long)held, DMS_HOLD_REQUIRED_MS);
            }
            if (held >= DMS_HOLD_REQUIRED_MS) {
                currentState        = AUTONOMOUS_STATE;
                dmsStartTime        = 0;
                s_idle_ready_logged = false;
                vcs_log("FSM: IDLE -> AUTONOMOUS");
            }
        } else {
            if (dmsStartTime != 0) {
                dmsStartTime = 0;
                vcs_log("IDLE: DMS released — countdown reset");
            }
        }
        break;
    }

    // ── MANUAL ───────────────────────────────────────────────
    // Drive freely. DMS hold → AUTONOMOUS (no Jetson mode required).
    case MANUAL_STATE: {
        bool driverInputActive = isThrottlePedalPressed() ||
                                 isPhysicalBrakePressed();

        // Debug print every 2s
        {
            static uint32_t lastDbg = 0;
            if (millis() - lastDbg > 2000) {
                lastDbg = millis();
                Serial.printf(
                    "[MAN] dms=%d mode=%d thr_raw=%u thr=%d brk=%d\r\n",
                    (int)isDeadmanActive(),
                    (int)getANSCommandMode(),
                    (unsigned)current_throttle_adc,
                    (int)isThrottlePedalPressed(),
                    (int)isPhysicalBrakePressed()
                );
            }
        }

        // DMS held → AUTONOMOUS (DMS alone, no Jetson confirmation needed)
        if (isDeadmanActive() && !driverInputActive) {
            if (dmsStartTime == 0) {
                dmsStartTime = millis();
                vcs_log("MANUAL: both DMS held — AUTONOMOUS countdown 1s");
            }
            if (millis() - dmsStartTime >= DMS_HOLD_REQUIRED_MS) {
                currentState     = AUTONOMOUS_STATE;
                dmsStartTime     = 0;
                s_graceStartTime = 0;
                vcs_log("FSM: MANUAL -> AUTONOMOUS");
            }
        } else {
            if (dmsStartTime != 0) {
                dmsStartTime = 0;
            }
        }
        break;
    }

    // ── AUTONOMOUS ───────────────────────────────────────────
    case AUTONOMOUS_STATE: {

        // Debug print every 2s
        {
            static uint32_t lastDbg2 = 0;
            if (millis() - lastDbg2 > 2000) {
                lastDbg2 = millis();
                Serial.printf(
                    "[AUTO] thr=%d brk=%d dms=%d mode=%d grace=%lums\r\n",
                    (int)isThrottlePedalPressed(),
                    (int)isPhysicalBrakePressed(),
                    (int)isDeadmanActive(),
                    (int)getANSCommandMode(),
                    s_graceStartTime
                        ? (unsigned long)(millis() - s_graceStartTime) : 0UL
                );
            }
        }

        // 1. Throttle pedal → MANUAL immediately
        if (isThrottlePedalPressed()) {
            currentState     = MANUAL_STATE;
            s_graceStartTime = 0;
            vcs_log("SAFETY: throttle pedal -> MANUAL");
            break;
        }

        // 2. Physical brake switch → MANUAL immediately
        if (isPhysicalBrakePressed()) {
            currentState     = MANUAL_STATE;
            s_graceStartTime = 0;
            vcs_log("SAFETY: brake switch -> MANUAL");
            break;
        }

        // 3. DMS released → 2s grace → MANUAL
        if (!isDeadmanActive()) {
            if (s_graceStartTime == 0) {
                s_graceStartTime = millis();
                vcs_log("DMS released — 2s grace started");
            }
            if (millis() - s_graceStartTime > 2000u) {
                currentState     = MANUAL_STATE;
                s_graceStartTime = 0;
                vcs_log("SAFETY: DMS released -> MANUAL");
            }
        } else {
            s_graceStartTime = 0;
        }

        // 4. Jetson explicit manual command only (mode=2)
        // Do NOT exit on mode=0 (IDLE) — transient during FSM changes
        if (currentState == AUTONOMOUS_STATE && getANSCommandMode() == 2) {
            currentState     = MANUAL_STATE;
            s_graceStartTime = 0;
            vcs_log("JETSON: sent MANUAL (mode=2) -> exiting AUTONOMOUS");
        }
        break;
    }

    default: break;
    }

    // ── Entry actions (once per state change) ─────────────────
    
    if (currentState != lastState) {
        if (currentState == IDLE_STATE) {
            s_idleEntryMs = millis();
            forceBrakeEngagement(false);
            vcs_log("IDLE entered: brakes retracting");
            hall_interrupts_attach();
        } else if (!isDrivingState()) {
            forceBrakeEngagement(true);
            s_idleEntryMs = 0;
        }
        Serial.print(F("VCS_STATE_MACHINE: -> "));
        Serial.println(getStateName(currentState));
        lastState = currentState;
    }
}

// ── Fault management ──────────────────────────────────────────
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

// ── Helpers ───────────────────────────────────────────────────
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