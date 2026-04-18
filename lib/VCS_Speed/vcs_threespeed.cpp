/* ==============================================================================
 * MODULE:        VCS_ThreeSpeed
 * RESPONSIBILITY: Read physical 3-position switch and apply software speed limits.
 * *REVISED FOR AUTONOMOUS SOFTWARE LIMITS*
 * ============================================================================== */

#include "vcs_threespeed.h"
#include "vcs_pins.h"

DriveMode current_drive_mode = DRIVE_MED; 
static float speed_limit_multiplier = 0.60f; // Default to 60% on boot

// Debounce: at 100 Hz, 3 ticks = 30 ms of stable reading before we
// commit a mode change. Prevents PWM surges on SP3T contact bounce.
static const uint8_t SPEED_DEBOUNCE_TICKS = 3;

void initThreeSpeed() {
    // Enable internal pull-ups so grounding the pins gives a clean LOW
    pinMode(PIN_SPEED_SW_LOW, INPUT_PULLUP);
    pinMode(PIN_SPEED_SW_HIGH, INPUT_PULLUP);
    
    // Note: pinMode OUTPUTS for physical speed wires were removed 
    // because D12 was repurposed for the Organizer Relay in V1.5.

    setDriveMode(DRIVE_MED); // Default on boot
}

void updateThreeSpeed() {

    bool swLow = (digitalRead(PIN_SPEED_SW_LOW) == LOW);
    bool swHigh = (digitalRead(PIN_SPEED_SW_HIGH) == LOW);

    // Decide the raw requested mode this tick. Center position
    // (neither pin grounded) defaults to MED.
    DriveMode rawMode;
    if (swLow) {
        rawMode = DRIVE_LOW;
    } else if (swHigh) {
        rawMode = DRIVE_HIGH;
    } else {
        rawMode = DRIVE_MED;
    }

    // Debounce: only commit the change once the same mode has been
    // seen for SPEED_DEBOUNCE_TICKS consecutive ticks.
    static DriveMode pendingMode = DRIVE_MED;
    static uint8_t   pendingCount = 0;

    if (rawMode == current_drive_mode) {
        // Already there — discard any in-progress pending change.
        pendingCount = 0;
    } else if (rawMode == pendingMode) {
        if (pendingCount < SPEED_DEBOUNCE_TICKS) pendingCount++;
        if (pendingCount >= SPEED_DEBOUNCE_TICKS) {
            setDriveMode(pendingMode);
            pendingCount = 0;
        }
    } else {
        // New candidate — restart the counter.
        pendingMode = rawMode;
        pendingCount = 1;
    }
}

void setDriveMode(DriveMode mode) {
    current_drive_mode = mode;
    
    // Instead of triggering hardware relays, we set a software throttle multiplier.
    // The vcs_throttle.cpp module will multiply its final PWM output by this number.
    switch (mode) {
        case DRIVE_LOW:
            speed_limit_multiplier = 0.30f; // 30% Max Throttle
            break;
        case DRIVE_MED:
            speed_limit_multiplier = 0.60f; // 60% Max Throttle
            break;
        case DRIVE_HIGH:
            speed_limit_multiplier = 1.00f; // 100% Max Throttle
            break;
    }
}

float getMaxThrottleMultiplier() {
    return speed_limit_multiplier;
}