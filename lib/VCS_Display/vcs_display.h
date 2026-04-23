#ifndef VCS_DISPLAY_H
#define VCS_DISPLAY_H

#include <Arduino.h>
#include <Wire.h>
#include "vcs_threespeed.h"
#include "vcs_pins.h"

void initDisplay();
void updateDisplay(float rpm, uint16_t steer, DriveMode speedMode);

#endif // VCS_DISPLAY_H