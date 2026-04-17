#include "vcs_throttle.h"
#include "vcs_threespeed.h"
#include "vcs_state_machine.h"
#include "vcs_constants.h"
#include "vcs_pins.h"

// Global Telemetry Variables
uint16_t current_throttle_adc = 0;
uint16_t current_pwm_duty = 0;

// EMA Smoothing for Throttle Input
float smoothedThrottle = 0.0;
const float emaAlphaThrottle = 0.15; // 0.15 is the smoothing weight

// PID Variables
float measured_rpm = 0.0f;
float target_rpm = 0.0f;
float throttle_pwm_out = 0.0f;

// Initialize QuickPID
QuickPID speedPID(&measured_rpm, &throttle_pwm_out, &target_rpm);

void initThrottle() {
    pinMode(PIN_THROTTLE_OUT, OUTPUT);
    pinMode(PIN_THROTTLE_IN, INPUT);

    // Sync Nano 33 BLE hardware to the 10-bit standard (0-1023)
    #ifdef NANO_33_BLE
    analogReadResolution(10);
    analogWriteResolution(10);
    #endif

    // Apply constants from vcs_constants.h
    speedPID.SetTunings(SPEED_KP, SPEED_KI, 0.0f);
    speedPID.SetOutputLimits(MIN_PWM_OUT, MAX_PWM_OUT);
    
    // CRITICAL: Sync QuickPID to our 1kHz Mbed ControlTask thread
    speedPID.SetSampleTimeUs(1000); 
    
    // Start in manual to prevent accidental windup on boot
    speedPID.SetMode(QuickPID::Control::manual);
}

void updateThrottle(float current_rpm_in, float target_rpm_in) {
    // --- EMA FILTER INJECTION ---
    int rawThrottle = analogRead(PIN_THROTTLE_IN);
    smoothedThrottle = (emaAlphaThrottle * rawThrottle) + ((1.0 - emaAlphaThrottle) * smoothedThrottle);
    current_throttle_adc = (uint16_t)smoothedThrottle;

    // --- NEW: FETCH HARDWARE SPEED LIMIT ---
    float speed_multiplier = getMaxThrottleMultiplier(); 
    
    // Calculate the dynamic max PWM based on the 3-position switch
    // E.g., If MAX_PWM is 1023 and multiplier is 0.6, dynamic max is ~613.
    int dynamic_max_pwm = MIN_PWM_OUT + (int)((MAX_PWM_OUT - MIN_PWM_OUT) * speed_multiplier);

    // Update PID limits dynamically so the integral windup respects the physical switch
    speedPID.SetOutputLimits(MIN_PWM_OUT, dynamic_max_pwm);

    // --- 1. HARDWARE SAFETY LOCKOUT ---
    bool isBrakePressed = (digitalRead(PIN_LOWBRAKE_IN) == LOW);

    // If the state machine says we shouldn't be driving OR driver hits the brake
    if ((currentState != AUTONOMOUS_STATE && currentState != MANUAL_STATE) || isBrakePressed) {
        current_pwm_duty = MIN_PWM_OUT;
        analogWrite(PIN_THROTTLE_OUT, current_pwm_duty);
        
        speedPID.SetMode(QuickPID::Control::manual);
        throttle_pwm_out = MIN_PWM_OUT;
        return; 
    }

    // --- 2. AUTONOMOUS CONTROL (PID) ---
    if (currentState == AUTONOMOUS_STATE) {
        if (speedPID.GetMode() == (uint8_t)QuickPID::Control::manual) {
            speedPID.SetMode(QuickPID::Control::automatic);
        }
        
        measured_rpm = current_rpm_in;
        target_rpm = target_rpm_in;
        
        if (speedPID.Compute()) {
            current_pwm_duty = (uint16_t)throttle_pwm_out;
            analogWrite(PIN_THROTTLE_OUT, current_pwm_duty);
        }
    } 
    // --- 3. MANUAL CONTROL (Pass-Through) ---
    else if (currentState == MANUAL_STATE) {
        int mapped_pwm = MIN_PWM_OUT;
        
        if (current_throttle_adc > THROTTLE_MIN_INPUT) {
            // Use the dynamic_max_pwm instead of the absolute MAX_PWM_OUT
            mapped_pwm = map(current_throttle_adc, THROTTLE_MIN_INPUT, THROTTLE_MAX_INPUT, MIN_PWM_OUT, dynamic_max_pwm);
            mapped_pwm = constrain(mapped_pwm, MIN_PWM_OUT, dynamic_max_pwm);
        }
        
        current_pwm_duty = mapped_pwm;
        analogWrite(PIN_THROTTLE_OUT, current_pwm_duty);
        
        // BUMPLESS TRANSFER: 
        throttle_pwm_out = mapped_pwm;
        speedPID.SetMode(QuickPID::Control::manual);
    }
}

// [ADDED] Helper function for the State Machine to check for manual override
bool isThrottlePedalPressed() {
    // Add a small safety margin above the minimum to prevent noise from dropping auto mode
    return (current_throttle_adc > (THROTTLE_MIN_INPUT + 15));
}