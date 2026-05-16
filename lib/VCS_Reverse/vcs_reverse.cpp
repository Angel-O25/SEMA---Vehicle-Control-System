#include "vcs_reverse.h"
#include "vcs_pins.h"
#include "vcs_hallsensor.h" 
#include "vcs_state_machine.h" 
#include "vcs_uart.h"

static bool reverseEngaged = false;

void initReverse() {
    // PCB has a physical 10k pull-UP. Configure as plain INPUT to prevent fighting the PCB.
    pinMode(PIN_REVERSE_IN, INPUT);
}

void updateReverse() {
    bool manualReverseRequested = isReverseSwitchPressed();
    bool autoReverseRequested = getANSReverseCommand(); 
    
    int currentRPM = getMeasuredRPM(); 

    static const int      STOPPED_ENTER_RPM    = 3;
    static const int      STOPPED_EXIT_RPM     = 8;
    static const uint8_t  STOPPED_CONFIRM_TICKS = 5;
    static bool           isStopped     = true; 
    static uint8_t        stoppedCounter = 0;

    bool rawStopped = isStopped
                        ? (currentRPM <  STOPPED_EXIT_RPM)   
                        : (currentRPM <  STOPPED_ENTER_RPM); 

    if (rawStopped != isStopped) {
        if (stoppedCounter < STOPPED_CONFIRM_TICKS) stoppedCounter++;
        if (stoppedCounter >= STOPPED_CONFIRM_TICKS) {
            isStopped = rawStopped;
            stoppedCounter = 0;
        }
    } else {
        stoppedCounter = 0;
    }
    
    bool isManual = (currentState == MANUAL_STATE || currentState == IDLE_STATE);
    bool isAuto = (currentState == AUTONOMOUS_STATE);

    bool triggerReverse = false;

    if (isStopped) {
        if (isManual && manualReverseRequested) {
            triggerReverse = true;
        } 
        else if (isAuto && autoReverseRequested) {
            triggerReverse = true;
        }
    }

    // On ESP32, the physical switch directly pulls the TXB0108 line LOW, 
    // so we only track the logical state here for telemetry.
    if (triggerReverse) {
        reverseEngaged = true;
    } else {
        reverseEngaged = false;
    }
}

bool isReverseEngaged() {
    return reverseEngaged;
}

bool isReverseSwitchPressed() {
    return (digitalRead(PIN_REVERSE_IN) == LOW);
}