#ifndef VCS_LOWBRAKE_H
#define VCS_LOWBRAKE_H

#include <Arduino.h>
#include "vcs_pins.h"
#include "vcs_constants.h"
#include "vcs_state_machine.h"

// True while the physical brake switch is debounced-pressed.
extern bool is_brake_pressed;

void initLowBrake();
void updateLowBrake();

// Forces brake state from the FSM (engage on FAULT/INIT/STOPPING entry).
void forceBrakeEngagement(bool engage);

bool isPhysicalBrakePressed();

#endif // VCS_LOWBRAKE_H