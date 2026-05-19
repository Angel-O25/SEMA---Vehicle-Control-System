#ifndef VCS_DISPLAY_H
#define VCS_DISPLAY_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include "vcs_pins.h"

void initDisplay();
void updateDisplay(float rpm, uint16_t steer_comm);

#endif // VCS_DISPLAY_H