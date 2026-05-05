#include "vcs_threespeed.h"
#include "vcs_pins.h"

DriveMode current_drive_mode = DRIVE_MED; 
static float speed_limit_multiplier = 0.60f; // Default to 60% on boot

void initThreeSpeed() {
    setDriveMode(DRIVE_MED); // Default on boot
}

void updateThreeSpeed() {
    // FIX #10: For testing, cycle through modes every 5 seconds.
    // In a real implementation, this would be triggered by a button press or command.
    if (current_drive_mode != DRIVE_HIGH) {
        setDriveMode(DRIVE_HIGH);
    }
}

void setDriveMode(DriveMode mode) {
    current_drive_mode = mode;
    
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