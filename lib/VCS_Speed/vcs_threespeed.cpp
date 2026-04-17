/* ==============================================================================
 * MODULE:        VCS_ThreeSpeed
 * RESPONSIBILITY: Read physical 3-position switch and apply software speed limits.
 * *REVISED FOR AUTONOMOUS SOFTWARE LIMITS*
 * ============================================================================== */

#include "vcs_threespeed.h"
#include "vcs_pins.h"

DriveMode current_drive_mode = DRIVE_MED; 
static float speed_limit_multiplier = 0.60f; // Default to 60% on boot

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

    if (swLow) { 
        setDriveMode(DRIVE_LOW);
    } else if (swHigh) {
        setDriveMode(DRIVE_HIGH);
    } else {
        setDriveMode(DRIVE_MED); // Center position / No pins grounded
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