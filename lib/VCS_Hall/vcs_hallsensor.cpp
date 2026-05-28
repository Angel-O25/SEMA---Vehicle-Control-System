// ============================================================
//  vcs_hallsensor.cpp — SIDLAK 2 VCS
//  Team Wired PH0017003 | Shell Eco-marathon 2026
//
//  Hall ISR DISABLED — open-loop operation.
//  RPM is estimated from the current DAC output (throttle lookup).
//  Physical hall sensor wires may remain connected but are ignored.
// ============================================================

#include "vcs_hallsensor.h"
#include "vcs_throttle.h"
#include "vcs_constants.h"
#include "vcs_calibration.h"

// ── Stub: ISR does nothing ────────────────────────────────────
void hall_interrupts_attach()  { /* disabled — open-loop mode */ }
void hall_interrupts_detach()  { /* disabled — open-loop mode */ }

// ── Stub: updateHallCalculations does nothing ─────────────────
void initHallSensors() {
    // Pin setup kept so PCB remains valid — just no interrupt attached
    pinMode(PIN_HALL_SPEED, INPUT_PULLDOWN);
}

void updateHallCalculations() {
    // No-op. RPM is derived from DAC in vcs_throttle.cpp.
}

bool consumeNewRPMSample() {
    return true;   // always "new" so callers don't gate on it
}

// ── RPM / speed — from throttle estimate ─────────────────────
float getMeasuredRPM() {
    return getThrottleEstimatedRPM();
}

float getMeasuredSpeedKmh() {
    float rpm = getMeasuredRPM();
    // v (km/h) = RPM × circumference(m) / 60 × 3.6
    return (rpm * WHEEL_CIRCUMFERENCE_M / 60.0f) * 3.6f;
}

// ── Stub for debug display ───────────────────────────────────
uint32_t getHallFalseEdgeCount() { return 0; }