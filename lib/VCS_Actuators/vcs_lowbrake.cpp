#include "vcs_lowbrake.h"


// Global Telemetry Variable
bool is_brake_pressed = false;

// Internal state tracking for the debounce logic
static uint32_t lastDebounceTime = 0;
static int lastButtonState = HIGH; 

void initLowBrake() {
    // Configure inputs and outputs
    pinMode(PIN_LOWBRAKE_IN, INPUT_PULLUP); 
    pinMode(PIN_LOWBRAKE_OUT, OUTPUT);
    
    // Engage brake fully on boot for hardware-level safety
    forceBrakeEngagement(true); 
}

void updateLowBrake() {
    // --- 1. DEBOUNCE LOGIC ---
    int reading = digitalRead(PIN_LOWBRAKE_IN);
    
    // Reset the debounce timer if the state changed (noise or physical press)
    if (reading != lastButtonState) {
        lastDebounceTime = millis();
    }
    
    // If the state has been stable longer than the debounce delay
    if ((millis() - lastDebounceTime) > DEBOUNCE_TIME_MS) {
        // Active LOW: If the pin is grounded, the brake is pressed
        is_brake_pressed = (reading == LOW);
    }
    
    lastButtonState = reading;
    
    // --- 2. INSTANT HARDWARE OVERRIDE & SECURE RELEASE ---
    if (is_brake_pressed) {
        // If human presses physical brake, fire the optocoupler immediately.
        // This bypasses the state machine entirely for zero-latency stopping.
        digitalWrite(PIN_LOWBRAKE_OUT, HIGH); 
    } else {
        // SECURITY FIX: The human let go of the pedal.
        // ONLY release the physical optocoupler if the vehicle is in a safe driving state.
        // If the system is in ESTOP, FAULT, or INIT, the brake remains physically locked.
        if (currentState == MANUAL_STATE || currentState == AUTONOMOUS_STATE) {
            digitalWrite(PIN_LOWBRAKE_OUT, LOW);
        }
    }
}

void forceBrakeEngagement(bool engage) {
    if (engage) {
        digitalWrite(PIN_LOWBRAKE_OUT, HIGH); // Force trigger optocoupler
    } else {
        // SECURITY CHECK: Only allow the state machine to release the software brake 
        // if the human driver is NOT currently holding down the physical brake pedal.
        if (!is_brake_pressed) {
            digitalWrite(PIN_LOWBRAKE_OUT, LOW);
        }
    }
}

bool isPhysicalBrakePressed() {
    return is_brake_pressed;
}