#include "vcs_reverse.h"
#include "vcs_pins.h"
#include "vcs_hallsensor.h" 
#include "vcs_state_machine.h" 
#include "vcs_uart.h"

// Track the actual state for telemetry and UI
static bool reverseEngaged = false;

void initReverse() {
    // Input: Driver's physical switch (LOW means they flipped it)
    pinMode(PIN_REVERSE_IN, INPUT_PULLUP);
    
    // Output: To Level Shifter -> Motor Controller Yellow Wire
    pinMode(PIN_REVERSE_OUT, OUTPUT);
    
    // Start in forward mode (HIGH = inactive/forward for the controller)
    digitalWrite(PIN_REVERSE_OUT, HIGH); 
}

void updateReverse() {
    // 1. Read the inputs
    bool manualReverseRequested = isReverseSwitchPressed();
    bool autoReverseRequested = getANSReverseCommand(); // Fetch from UART module
    
    // 2. Read what the car is actually doing
    int currentRPM = getMeasuredRPM(); 

    // --- 3. THE SECURITY GATES ---
    // Gate A: Is the car effectively stopped? (Crucial hardware protection)
    //
    // Hysteresis + debounce:
    //   - Enter "stopped" below 3 RPM.
    //   - Leave "stopped" above 8 RPM.
    //   - Require the condition to hold for STOPPED_CONFIRM_TICKS (50 ms @ 100 Hz)
    //     before committing, so RPM noise right at the threshold cannot flicker
    //     the reverse pin.
    static const int      STOPPED_ENTER_RPM    = 3;
    static const int      STOPPED_EXIT_RPM     = 8;
    static const uint8_t  STOPPED_CONFIRM_TICKS = 5;
    static bool           isStopped     = true; // safe default
    static uint8_t        stoppedCounter = 0;

    bool rawStopped = isStopped
                        ? (currentRPM <  STOPPED_EXIT_RPM)   // stay stopped until clearly moving
                        : (currentRPM <  STOPPED_ENTER_RPM); // stay moving until clearly stopped

    if (rawStopped != isStopped) {
        if (stoppedCounter < STOPPED_CONFIRM_TICKS) stoppedCounter++;
        if (stoppedCounter >= STOPPED_CONFIRM_TICKS) {
            isStopped = rawStopped;
            stoppedCounter = 0;
        }
    } else {
        stoppedCounter = 0;
    }
    
    // Gate B: Determine which mode holds authority
    bool isManual = (currentState == MANUAL_STATE || currentState == IDLE_STATE);
    bool isAuto = (currentState == AUTONOMOUS_STATE);

    // --- 4. THE DECISION MATRIX ---
    bool triggerReverse = false;

    // Only allow a shift if the car is stopped
    if (isStopped) {
        // If human is driving and human wants reverse
        if (isManual && manualReverseRequested) {
            triggerReverse = true;
        } 
        // If ANS is driving and ANS wants reverse
        else if (isAuto && autoReverseRequested) {
            triggerReverse = true;
        }
    }

    // --- 5. HARDWARE ACTUATION ---
    if (triggerReverse) {
        digitalWrite(PIN_REVERSE_OUT, LOW); // Pull Yellow wire to Ground
        reverseEngaged = true;
    } else {
        digitalWrite(PIN_REVERSE_OUT, HIGH); // Release Yellow wire (Forward)
        reverseEngaged = false;
    }
}

bool isReverseEngaged() {
    return reverseEngaged;
}

bool isReverseSwitchPressed() {
    return (digitalRead(PIN_REVERSE_IN) == LOW);
}