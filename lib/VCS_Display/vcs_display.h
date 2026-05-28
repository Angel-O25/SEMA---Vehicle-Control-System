#ifndef VCS_DISPLAY_H
#define VCS_DISPLAY_H

#include <Arduino.h>
#include "vcs_pins.h"

void initDisplay();
void updateDisplay(float rpm, uint16_t steer_comm);  // normal production display
void updateDebugDisplay();                             // debug mode display

#endif // VCS_DISPLAY_H