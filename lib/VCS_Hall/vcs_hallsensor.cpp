#include "vcs_hallsensor.h"
#include "vcs_pins.h" // Ensure pins are included for PIN_HALL_SPEED

// Volatile variables are required for data shared with Interrupt Service Routines (ISRs)
volatile uint32_t hall_pulse_count = 0;
uint32_t last_calc_time = 0;
float current_rpm = 0.0f;

// The ISR: Runs every time the Controller's Yellow Wire pulses HIGH
// REMOVED IRAM_ATTR: Not required/supported on Nano 33 BLE (Mbed OS)
void handleHallInterrupt() {
    hall_pulse_count++;
}

void initHallSensors() {
    // The signal now comes from the 5V Level Shifter to D10.
    // INPUT_PULLUP provides noise immunity if the wire ever vibrates loose.
    pinMode(PIN_HALL_SPEED, INPUT_PULLUP);
    
    // Attach interrupt to the new Speed Pin. 
    // Changed from CHANGE to RISING because the controller outputs a clean square wave.
    attachInterrupt(digitalPinToInterrupt(PIN_HALL_SPEED), handleHallInterrupt, RISING);
    
    last_calc_time = millis();
}

void updateHallCalculations() {
    uint32_t now = millis();
    uint32_t elapsed = now - last_calc_time;

    // Calculate RPM every 100ms for a balance of responsiveness and stability
    if (elapsed >= 100) {
        // 1. Atomically read and reset pulse count
        noInterrupts(); // Disable interrupts briefly to prevent data corruption
        uint32_t pulses = hall_pulse_count;
        hall_pulse_count = 0;
        interrupts();

        // 2. Math for RPM (Corrected for Controller Speed Output)
        // Since we are triggering on RISING, we get exactly 1 interrupt per pulse.
        // Note: Standard e-bike controllers output either 1 pulse per mechanical revolution 
        // OR a number of pulses equal to the MOTOR_POLE_PAIRS. 
        float pulses_per_rev = (float)MOTOR_POLE_PAIRS; // Change to 1.0f if your controller outputs 1 pulse/rev
        
        // Apply Gear Reduction if your e-bike hub has internal planetary gears
        pulses_per_rev *= GEAR_REDUCTION; 
        
        // RPM = (Pulses / Pulses_Per_Rev) * (60,000ms / Elapsed_ms)
        if (pulses > 0) {
            current_rpm = ((float)pulses / pulses_per_rev) * (60000.0f / (float)elapsed);
        } else {
            current_rpm = 0.0f;
        }

        last_calc_time = now;
    }
}

float getMeasuredRPM() {
    // Linked directly to the v1.3 constant we defined earlier
    #if SIMULATION_MODE
        return getSimulatedRPM();
    #else
        return current_rpm;
    #endif
}