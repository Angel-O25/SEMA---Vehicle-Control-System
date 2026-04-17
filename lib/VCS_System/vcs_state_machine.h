#ifndef VCS_STATE_MACHINE_H
#define VCS_STATE_MACHINE_H

#include <Arduino.h>

/* ==============================================================================
 * MODULE:        VCS_StateMachine
 * RESPONSIBILITY: Core vehicle safety and state transition logic.
 * *REVISED FOR SEM AUTONOMOUS RULES (V1.5)*
 * ============================================================================== */

// SIDLAK Safety Hierarchy Enums
enum VcsState {
    INIT_STATE,        // Power-on self-test & sensor stabilization
    IDLE_STATE,        // Standby; waiting for ANS System heartbeat
    MANUAL_STATE,      // Human-in-the-loop control (Default Drive State)
    AUTONOMOUS_STATE,  // ANS System-controlled driving (Requires BOTH Dead-Man Switches held)
    FAULT_STATE,       // Software/Comms fail-safe triggered (e.g., ANS System disconnected)
    ESTOP_STATE        // Critical hardware lockout (Hard reset required)
};

// --- Fault Bit Definitions ---
// Recoverable faults map to FAULT_STATE and clear when the underlying
// condition resolves. ESTOP-class faults are latched and require reboot.
#define VCS_FAULT_NONE               0x00000000u

// Recoverable faults (low 16 bits):
#define VCS_FAULT_UART_CRC           0x00000001u   // CRC mismatch on command packet
#define VCS_FAULT_HEARTBEAT_LOST     0x00000002u   // ANS System heartbeat timeout
#define VCS_FAULT_SENSOR_SPIKE       0x00000004u   // Hall / steering pot out-of-range
#define VCS_FAULT_OVERCURRENT        0x00000008u   // Motor controller fault line

// ESTOP-class faults (high 16 bits, latched until power-cycle):
#define VCS_FAULT_ESTOP_MASK         0xFFFF0000u
#define VCS_FAULT_SOFTWARE_ESTOP     0x00010000u   // ANS System sent fatal kill command

// --- Safety Timing Constants ---
// (Candidate to relocate into vcs_constants.h if that's the convention.)
#define DMS_HOLD_REQUIRED_MS         1000u         // SEM spec: 1.0 s dual-grip hold

// Global State Variable
// NOTE: currentState is read from ControlTask (1 kHz) and written from
// CommTask (100 Hz). On nRF52840, word-aligned enum reads/writes are
// single-instruction atomic in practice, but consider wrapping in
// std::atomic<VcsState> for a future hardening pass.
extern VcsState currentState;

// --- Initialization and Main Logic ---
void initState_Machine();
void updateStateMachine(uint32_t externalFaults);

// --- Fault Injection ---
// Latch a software E-Stop (called by UART module when ANS System sends a
// fatal kill command, or by any other module that detects a non-recoverable
// hazard). Only clearable by power-cycle.
void requestSoftwareEstop();

// --- Telemetry & Display Helpers ---
// Returns the millis() timestamp at which the current DMS hold began,
// or 0 if no hold is in progress.
uint32_t getDMSHoldStartTime();
const char* getStateName(VcsState state);

// --- Logic Helpers for Actuators/UART ---
bool isAutonomousMode();
bool isDrivingState();

#endif // VCS_STATE_MACHINE_H