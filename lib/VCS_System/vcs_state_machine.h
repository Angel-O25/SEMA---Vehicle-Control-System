#ifndef VCS_STATE_MACHINE_H
#define VCS_STATE_MACHINE_H

#include <Arduino.h>

/* ==============================================================================
 * MODULE:         VCS_StateMachine
 * RESPONSIBILITY: Core vehicle safety and state transition logic.
 *                 Revised for Shell Eco-marathon 2026 autonomous rules.
 * ============================================================================== */

// Sidlak Safety Hierarchy
enum VcsState {
    INIT_STATE,        // Power-on self-test & sensor stabilization
    IDLE_STATE,        // Standby; waiting for ANS heartbeat
    MANUAL_STATE,      // Human-in-the-loop control (default drive state)
    AUTONOMOUS_STATE,  // ANS-controlled driving (requires both DMS held)
    FAULT_STATE,       // Software / comms fail-safe (recoverable)
    STOPPING_STATE     // Transient 3 s pause on stop-line / brake assertion
                       // NOTE: a separate latched ESTOP_STATE is not modelled.
                       //       Treat FAULT as the highest unrecoverable state for now.
};

// Global state.
//
// NOTE on concurrency: currentState is currently written by task_control
// (the FSM owner) and read by other tasks. Word-aligned enum reads on
// ESP32 (Xtensa LX6) are single-instruction atomic in practice, so a
// plain extern is acceptable for now. Wrap in std::atomic<VcsState>
// during a future hardening pass if you start writing it from more
// than one task.
extern VcsState currentState;

// --- Initialization & main tick ---
void initState_Machine();
void updateStateMachine();
void clearFault(uint16_t fault_code);
void clearAllFaults();
void triggerFault(uint16_t fault_code);

// --- Fault injection ---
// Latch a software E-stop. Currently routes through FAULT_STATE; will
// route to a dedicated latched state once one is added.
void requestSoftwareEstop();
uint32_t getSystemFaults();

// --- Telemetry / display helpers ---
uint32_t    getDMSHoldStartTime();
const char* getStateName(VcsState state);

// --- Logic helpers ---
bool isAutonomousMode();
bool isDrivingState();
bool isEstopLatched();

#endif // VCS_STATE_MACHINE_H