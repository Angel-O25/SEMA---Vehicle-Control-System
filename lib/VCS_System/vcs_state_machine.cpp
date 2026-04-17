#include "vcs_state_machine.h"
#include "vcs_uart.h"
#include "vcs_pins.h"
#include "vcs_deadman.h"    // [ADDED] Replaces vcs_embutton
#include "vcs_lowbrake.h"
#include "vcs_throttle.h"   // [ADDED] For the pedal override check
#include "vcs_reverse.h"

VcsState currentState = INIT_STATE;
uint32_t dmsStartTime = 0;

void initState_Machine() {
    currentState = INIT_STATE;
    dmsStartTime = 0;
}

void updateStateMachine(uint32_t faults) {
    static VcsState lastState = INIT_STATE;

    // --- 1. PRIORITY SAFETY OVERRIDES ---
    
    // External faults (Over-current/Voltage) or Heartbeat Loss
    if (faults > 0 || (!ansHeartbeatReceived() && currentState == AUTONOMOUS_STATE)) {
        currentState = FAULT_STATE;
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
                if (millis() - dmsStartTime > 1000) currentState = AUTONOMOUS_STATE;
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
            // (Legacy state - Hardware E-stop moved to power circuit. 
            // Retained in case the ANS sends a fatal software kill command).
            break;
    }

    // --- 3. STATE DEBUGGING ---
    if (currentState != lastState) {
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