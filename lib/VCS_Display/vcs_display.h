#ifndef VCS_DISPLAY_H
#define VCS_DISPLAY_H

#include <Arduino.h>
#include <Wire.h>
#include "vcs_threespeed.h"

void initDisplay();
void updateDisplay(float rpm, uint16_t steer, DriveMode speedMode);

#endif // VCS_DISPLAY_H