#include "vcs_hallsensor.h"
#include "vcs_pins.h"

#define WHEEL_CIRCUMFERENCE_M 1.2764f

volatile uint32_t hall_pulse_count = 0;
uint32_t last_calc_time = 0;
float current_rpm = 0.0f;
static bool s_new_rpm_sample = false;

// MUST be declared before attachInterrupt uses it
void IRAM_ATTR handleHallInterrupt() {
    hall_pulse_count++;
}

void initHallSensors() {
    // ESP32 supports internal pulldowns
    pinMode(PIN_HALL_SPEED, INPUT_PULLDOWN); 
    last_calc_time = millis();
}

void hall_interrupts_attach() {
    attachInterrupt(digitalPinToInterrupt(PIN_HALL_SPEED), handleHallInterrupt, RISING);
}

void hall_interrupts_detach() {
    detachInterrupt(digitalPinToInterrupt(PIN_HALL_SPEED));
}

void updateHallCalculations() {
    uint32_t now = millis();
    uint32_t elapsed = now - last_calc_time;

    if (elapsed >= 100) {
        // 1. Atomically read and reset pulse count
        noInterrupts(); 
        uint32_t pulses = hall_pulse_count;
        hall_pulse_count = 0;
        interrupts();

        // 2. Math for RPM
        float pulses_per_rev = (float)MOTOR_POLE_PAIRS; 
        pulses_per_rev *= GEAR_REDUCTION; 
        
        if (pulses > 0) {
            current_rpm = ((float)pulses / pulses_per_rev) * (60000.0f / (float)elapsed);
        } else {
            current_rpm = 0.0f;
        }

        last_calc_time = now;
        s_new_rpm_sample = true; 
    }
}

bool consumeNewRPMSample() {
    if (s_new_rpm_sample) {
        s_new_rpm_sample = false;
        return true;
    }
    return false;
}

float getMeasuredRPM() {
    #if SIMULATION_MODE
        return getSimulatedRPM();
    #else
        return current_rpm;
    #endif
}

float getMeasuredSpeedKmh() {
    float rpm = getMeasuredRPM();
    return (rpm * WHEEL_CIRCUMFERENCE_M * 60.0f) / 1000.0f;
}