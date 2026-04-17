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
    IDLE_STATE,        // Standby; waiting for RPi heartbeat
    MANUAL_STATE,      // Human-in-the-loop control (Default Drive State)
    AUTONOMOUS_STATE,  // RPi-controlled driving (Requires BOTH Dead-Man Switches held)
    FAULT_STATE,       // Software/Comms fail-safe triggered (e.g., Pi disconnected)
    ESTOP_STATE        // Critical hardware lockout (Hard reset required)
};

// Global State Variable
extern VcsState currentState;

// --- Initialization and Main Logic ---
void initState_Machine();
void updateStateMachine(uint32_t externalFaults);

// --- Telemetry & Display Helpers ---
// Returns how long the dual-DMS AND gate has been held down
uint32_t getDMSHoldStartTime(); 
const char* getStateName(VcsState state);

// --- Logic Helpers for Actuators/UART ---
bool isAutonomousMode();

// Optional: Returns true if the car is actively allowed to move
bool isDrivingState(); 

#endif // VCS_STATE_MACHINE_H