#ifndef VCS_STATE_MACHINE_H
#define VCS_STATE_MACHINE_H

#include <Arduino.h>

/* ==============================================================================
 * MODULE:         VCS_StateMachine
 * RESPONSIBILITY: Core vehicle safety and state transition logic.
 *                 Revised for Shell Eco-marathon 2026 autonomous rules.
 *
 * ARCHITECTURE NOTES:
 *   - FAULT_STATE removed. Task-hang protection is handled by the FreeRTOS
 *     watchdog timer (see main.cpp). Recoverable soft faults (UART CRC,
 *     heartbeat loss, sensor dropout) now route the FSM back to MANUAL_STATE
 *     so the driver retakes control without an actuator glitch.
 *   - requestSoftwareEstop() removed. No physical E-stop is wired to the
 *     ESP32; the hardware E-stop bypasses the embedded system entirely.
 *   - Fault tracking (triggerFault / getSystemFaults / clearFault) RETAINED
 *     for telemetry visibility so the Jetson dashboard can show what failed.
 * ============================================================================== */

// Sidlak Safety Hierarchy
enum VcsState {
    INIT_STATE,        // Power-on self-test & sensor stabilization
    IDLE_STATE,        // Standby; waiting for ANS heartbeat
    MANUAL_STATE,      // Human-in-the-loop control (default drive state +
                       //  the safe fallback on any soft fault)
    AUTONOMOUS_STATE,  // ANS-controlled driving (requires both DMS held)
    STOPPING_STATE     // Transient 3 s pause on stop-line / brake assertion
};

// Global state. Single-writer (task_control) — see concurrency note in v1.5.
extern VcsState currentState;

// --- Initialization & main tick ---
void initState_Machine();
void updateStateMachine();

// --- Fault tracking (telemetry only — does not change state) ---
// Faults are now informational. Any non-zero fault while AUTONOMOUS demotes
// the FSM to MANUAL via the heartbeat/fault check in updateStateMachine().
void     triggerFault(uint16_t fault_code);
void     clearFault(uint16_t fault_code);
void     clearAllFaults();
uint32_t getSystemFaults();

// --- Telemetry / display helpers ---
uint32_t    getDMSHoldStartTime();
const char* getStateName(VcsState state);

// --- Logic helpers ---
bool isAutonomousMode();
bool isDrivingState();

#endif // VCS_STATE_MACHINE_H