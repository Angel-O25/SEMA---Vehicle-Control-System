#include "vcs_lowbrake.h"


// Global Telemetry Variable
bool is_brake_pressed = false;

// Internal state tracking for the debounce logic
static uint32_t lastDebounceTime = 0;
static int lastButtonState = HIGH; 

void initLowBrake() {
    #if defined(ESP32_VCS)
        // Switch is active HIGH with no PCB resistor, requires internal pulldown
        pinMode(PIN_LOWBRAKE_IN, INPUT_PULLDOWN); 
        pinMode(PIN_LIMIT_SWITCH, INPUT_PULLDOWN);
        
        // TB6612 Actuator Pins
        pinMode(TB6612_IN1_PIN, OUTPUT);
        pinMode(TB6612_IN2_PIN, OUTPUT);
        pinMode(TB6612_PWM_PIN, OUTPUT);
        
        // Motor Controller Signal
        pinMode(BRAKE_MC_PIN, OUTPUT);
    #else
        pinMode(PIN_LOWBRAKE_IN, INPUT_PULLUP); 
        pinMode(PIN_LOWBRAKE_OUT, OUTPUT);
    #endif
    
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
        // If human presses physical brake, fire the brake immediately.
        // This bypasses the state machine entirely for zero-latency stopping.
        #if defined(ESP32_VCS)
            forceBrakeEngagement(true); // Route through centralized ESP32 dual-actuator logic
        #else
            digitalWrite(PIN_LOWBRAKE_OUT, HIGH); 
        #endif
    } else {
        // SECURITY FIX: The human let go of the pedal.
        // ONLY release the brake if the vehicle is in a safe driving state.
        if (currentState == MANUAL_STATE || currentState == AUTONOMOUS_STATE) {
            #if defined(ESP32_VCS)
                forceBrakeEngagement(false);
            #else
                digitalWrite(PIN_LOWBRAKE_OUT, LOW);
            #endif
        }
    }
}

void forceBrakeEngagement(bool engage) {
    if (engage) {
        #if defined(ESP32_VCS)
            digitalWrite(BRAKE_MC_PIN, LOW); // Active LOW to cut motor power
            
            // Extend linear actuator via TB6612
            if (digitalRead(PIN_LIMIT_SWITCH) == LOW) { // Stop if limit reached
                digitalWrite(TB6612_IN1_PIN, HIGH);
                digitalWrite(TB6612_IN2_PIN, LOW);
                analogWrite(TB6612_PWM_PIN, 200); 
            } else {
                digitalWrite(TB6612_IN1_PIN, LOW); // Hold position
                digitalWrite(TB6612_IN2_PIN, LOW);
            }
        #else
            digitalWrite(PIN_LOWBRAKE_OUT, HIGH);
        #endif
    } else {
        if (!is_brake_pressed) {
            #if defined(ESP32_VCS)
                digitalWrite(BRAKE_MC_PIN, HIGH); // Release MC brake
                
                // Retract linear actuator
                digitalWrite(TB6612_IN1_PIN, LOW);
                digitalWrite(TB6612_IN2_PIN, HIGH);
                analogWrite(TB6612_PWM_PIN, 200);
                // Note: The timed retract logic (BRAKE_RETRACT_MS) must be managed in the control loop
            #else
                digitalWrite(PIN_LOWBRAKE_OUT, LOW);
            #endif
        }
    }
}

bool isPhysicalBrakePressed() {
    return is_brake_pressed;
}