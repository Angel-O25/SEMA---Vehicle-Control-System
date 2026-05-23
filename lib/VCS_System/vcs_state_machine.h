#ifndef VCS_STATE_MACHINE_H
#define VCS_STATE_MACHINE_H

#include <Arduino.h>

/* ==============================================================================
 * MODULE:         VCS_StateMachine
 * RESPONSIBILITY: Core vehicle safety and state transition logic.
 *
 * STATES: INIT → IDLE → MANUAL ↔ AUTONOMOUS
 *
 * STOPPING_STATE retained in enum for API compatibility but is never entered.
 * Stop-line timing is handled entirely by Jetson mission_fsm. VCS stays in
 * AUTONOMOUS and physically brakes when getTargetBrake() > 0.
 *
 * AUTONOMOUS exit conditions:
 *   - Throttle pedal pressed        → MANUAL immediately
 *   - Physical brake switch pressed → MANUAL immediately
 *   - Deadman released              → MANUAL after 2s grace
 *   - Jetson sends mode=0 or mode=2 → MANUAL
 *   - Link lost or fault            → MANUAL
 * ============================================================================== */

enum VcsState {
    INIT_STATE,
    IDLE_STATE,
    MANUAL_STATE,
    AUTONOMOUS_STATE,
    STOPPING_STATE    // retained for API compat — not entered
};

extern VcsState currentState;

void initState_Machine();
void updateStateMachine();

void     triggerFault(uint16_t fault_code);
void     clearFault(uint16_t fault_code);
void     clearAllFaults();
uint32_t getSystemFaults();

uint32_t    getDMSHoldStartTime();
const char* getStateName(VcsState state);

bool isAutonomousMode();
bool isDrivingState();

#endif // VCS_STATE_MACHINE_H