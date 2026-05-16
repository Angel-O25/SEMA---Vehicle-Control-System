#include "vcs_hallsensor.h"
#include "vcs_pins.h"
#include "vcs_constants.h"

// ============================================================
//  HALL SENSOR IMPLEMENTATION
volatile uint32_t hall_pulse_count = 0;
uint32_t last_calc_time = 0;
float current_rpm = 0.0f;
static bool s_new_rpm_sample = false;

// For debugging: count false edges that are too close together
constexpr uint32_t MIN_PULSE_WIDTH_US = 800;
static volatile uint32_t s_lastEdge_us = 0;
static volatile uint32_t s_falseEdges  = 0;

void IRAM_ATTR handleHallInterrupt() {
    uint64_t now = esp_timer_get_time();
    if (now - s_lastEdge_us < HALL_DEBOUNCE_US) return;   // 2000µs from cal
    s_lastEdge_us = now;
    hall_pulse_count++;
}

uint32_t getHallFalseEdgeCount() { return s_falseEdges; }

void initHallSensors() {
    // ESP32 supports internal pulldowns
    pinMode(PIN_HALL_SPEED, INPUT_PULLDOWN); 
    last_calc_time = millis();
}

void hall_interrupts_attach() {
    attachInterrupt(digitalPinToInterrupt(PIN_HALL_SPEED), handleHallInterrupt, CHANGE);
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
        float pulses_per_rev = (float)HALL_TRANSITIONS_PER_MECH_REV; 
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