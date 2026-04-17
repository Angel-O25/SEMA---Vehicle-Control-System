#ifndef VCS_THREESPEED_H
#define VCS_THREESPEED_H

#include <Arduino.h>

enum DriveMode {
    DRIVE_LOW,
    DRIVE_MED,
    DRIVE_HIGH
};

extern DriveMode current_drive_mode;

void initThreeSpeed();
void updateThreeSpeed();
void setDriveMode(DriveMode mode);
float getMaxThrottleMultiplier();

#endif // VCS_THREESPEED_H