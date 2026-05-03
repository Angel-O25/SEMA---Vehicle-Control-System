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

// --- Fault Bit Definitions ---
// Recoverable faults map to FAULT_STATE and clear when the underlying
// condition resolves.
#define VCS_FAULT_NONE               0x00000000u
#define VCS_FAULT_UART_CRC           0x00000001u   // CRC mismatch on command packet
#define VCS_FAULT_HEARTBEAT_LOST     0x00000002u   // ANS heartbeat timeout
#define VCS_FAULT_SENSOR_SPIKE       0x00000004u   // Hall / steering pot out-of-range
#define VCS_FAULT_OVERCURRENT        0x00000008u   // Motor controller fault line

// --- Safety Timing Constants ---
// (Candidate to relocate into vcs_constants.h.)
#define DMS_HOLD_REQUIRED_MS         1000u         // SEM spec: 1.0 s dual-grip hold

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

#endif // VCS_STATE_MACHINE_H