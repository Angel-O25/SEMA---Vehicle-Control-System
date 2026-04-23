#include "vcs_hallsensor.h"
#include "vcs_pins.h"

volatile uint32_t hall_pulse_count = 0;
uint32_t last_calc_time = 0;
float current_rpm = 0.0f;
static bool s_new_rpm_sample = false;

// 1. MUST be declared before attachInterrupt uses it
void handleHallInterrupt() {
    hall_pulse_count++;
}

void initHallSensors() {
    #if defined(ESP32_VCS)
        // ESP32 supports internal pulldowns
        pinMode(PIN_HALL_SPEED, INPUT_PULLDOWN); 
    #else
        // ATmega328 ONLY supports internal pullups
        pinMode(PIN_HALL_SPEED, INPUT_PULLUP);
    #endif
    
    last_calc_time = millis();
}

void hall_interrupts_attach() {
    attachInterrupt(digitalPinToInterrupt(PIN_HALL_SPEED), handleHallInterrupt, RISING);
}

void hall_interrupts_detach() {
    detachInterrupt(digitalPinToInterrupt(PIN_HALL_SPEED));
}

// ... [KEEP YOUR updateHallCalculations() AND THE REST OF THE FILE INTACT BELOW] ...

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
        s_new_rpm_sample = true; // Signal downstream consumers (throttle PID)
    }
}

bool consumeNewRPMSample() {
    // Test-and-clear. Safe to call from the ControlTask thread since
    // updateHallCalculations() and consumeNewRPMSample() both run there
    // (not from the ISR).
    if (s_new_rpm_sample) {
        s_new_rpm_sample = false;
        return true;
    }
    return false;
}

float getMeasuredRPM() {
    // Linked directly to the v1.3 constant we defined earlier
    #if SIMULATION_MODE
        return getSimulatedRPM();
    #else
        return current_rpm;
    #endif
}