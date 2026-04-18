#include "vcs_state_machine.h"
#include "vcs_uart.h"
#include "vcs_pins.h"
#include "vcs_deadman.h"    // [ADDED] Replaces vcs_embutton
#include "vcs_lowbrake.h"
#include "vcs_throttle.h"   // [ADDED] For the pedal override check
#include "vcs_reverse.h"

VcsState currentState = INIT_STATE;
uint32_t dmsStartTime = 0;

// ESTOP latch: once set, only a power-cycle (which reinitializes this to false
// via initState_Machine) can clear it. Set either by an ESTOP-class bit in the
// fault word or by requestSoftwareEstop().
static bool estopLatched = false;

void initState_Machine() {
    currentState = INIT_STATE;
    dmsStartTime = 0;
    estopLatched = false;
}

void requestSoftwareEstop() {
    // Latch immediately so the next updateStateMachine() sees it and
    // drives the brake. We also pull the brake here so the reaction
    // doesn't have to wait for the next tick.
    estopLatched = true;
    currentState = ESTOP_STATE;
    forceBrakeEngagement(true);
}

void updateStateMachine(uint32_t faults) {
    static VcsState lastState = INIT_STATE;

    // --- 1. PRIORITY SAFETY OVERRIDES ---

    // ESTOP-class bits in the fault word latch forever (until power-cycle).
    // requestSoftwareEstop() sets the latch directly and bypasses this path.
    if (faults & VCS_FAULT_ESTOP_MASK) {
        estopLatched = true;
    }

    if (estopLatched) {
        // Highest priority: once latched, nothing else in this function
        // is allowed to transition us out of ESTOP_STATE.
        currentState = ESTOP_STATE;
    } else {
        // Recoverable faults (low 16 bits) and heartbeat loss during AUTONOMOUS.
        //
        // Note: heartbeat loss during MANUAL is intentionally NOT a fault here.
        // The human driver is in control and losing the ANS link should not
        // yank the car out from under them. VCS_FAULT_HEARTBEAT_LOST as defined
        // in the header is only surfaced as a state transition while autonomous.
        uint32_t recoverable = faults & ~VCS_FAULT_ESTOP_MASK;
        if (recoverable > 0 ||
            (!ansHeartbeatReceived() && currentState == AUTONOMOUS_STATE)) {
            currentState = FAULT_STATE;
        }
    }

    // --- 2. STATE TRANSITION LOGIC ---
    switch (currentState) {
        case INIT_STATE:
            // Boot delay for sensor stabilization
            if (millis() > 2000) currentState = IDLE_STATE;
            break;

        case IDLE_STATE:
            // Wait for ANS to be alive before allowing Manual mode
            if (ansHeartbeatReceived()) currentState = MANUAL_STATE;
            break;

        case MANUAL_STATE:
            // Transition to Auto: Requires physical DMS AND gate hold (1.0s), 
            // the ANS requesting Auto Mode (1), AND the car MUST NOT be in Reverse.
            if (isDeadmanActive() && getANSCommandMode() == 1 && !isReverseEngaged()) {
                if (dmsStartTime == 0) dmsStartTime = millis();
                if (millis() - dmsStartTime > DMS_HOLD_REQUIRED_MS) {
                    currentState = AUTONOMOUS_STATE;
                    // Clear so a subsequent AUTO->MANUAL->AUTO cycle requires
                    // a fresh 1-second hold instead of re-using the old timestamp.
                    dmsStartTime = 0;
                }
            } else {
                dmsStartTime = 0;
            }
            break;

        case AUTONOMOUS_STATE:
            // EXIT TO MANUAL IF:
            // 1. Physical Brake is pressed (Rule 412c Hard Override)
            // 2. Physical Throttle is pressed (Rule 412c Hard Override)
            // 3. Either DMS button is released (Rule 412f Safety requirement)
            // 4. ANS sends a Manual request (Soft override)
            if (isPhysicalBrakePressed() || 
                isThrottlePedalPressed() || 
                !isDeadmanActive() || 
                getANSCommandMode() == 0) {
                
                currentState = MANUAL_STATE;
            }
            break;
            
        case FAULT_STATE:
            // Reset only happens if ANS is re-linked and driver's hands are OFF the dead-man switches
            if (ansHeartbeatReceived() && !isDeadmanActive()) {
                currentState = IDLE_STATE;
            }
            break;

        case ESTOP_STATE:
            // Terminal state. Reached when an ESTOP-class fault bit is set or
            // requestSoftwareEstop() is called. estopLatched keeps us here
            // forever; only a power-cycle (which re-runs initState_Machine)
            // can clear it. Brake is asserted on entry by the transition
            // handler below.
            break;
    }

    // --- 3. ENTRY ACTIONS + STATE DEBUGGING ---
    if (currentState != lastState) {
        // On any transition into a non-driving state, assert the brake
        // immediately. This closes the hole where FAULT_STATE could be
        // entered while the pedal was released and vcs_lowbrake's else
        // branch would leave the optocoupler de-energized.
        if (!isDrivingState()) {
            forceBrakeEngagement(true);
        }

        Serial.print("VCS_STATE_MACHINE: Transitioned to ");
        Serial.println(getStateName(currentState));
        lastState = currentState;
    }
}

// Helper to get string names for the Serial Monitor
const char* getStateName(VcsState state) {
    switch (state) {
        case INIT_STATE:       return "INIT";
        case IDLE_STATE:       return "IDLE";
        case MANUAL_STATE:     return "MANUAL";
        case AUTONOMOUS_STATE: return "AUTONOMOUS";
        case FAULT_STATE:      return "FAULT";
        case ESTOP_STATE:      return "ESTOP";
        default:               return "UNKNOWN";
    }
}

// Getters for other modules
bool isAutonomousMode() { return currentState == AUTONOMOUS_STATE; }
uint32_t getDMSHoldStartTime() { return dmsStartTime; }


bool isDrivingState() { 
    return (currentState == MANUAL_STATE || currentState == AUTONOMOUS_STATE); 
}