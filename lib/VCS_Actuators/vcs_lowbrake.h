#ifndef VCS_LOWBRAKE_H
#define VCS_LOWBRAKE_H

#include <Arduino.h>
#include "vcs_pins.h"
#include "vcs_constants.h"
#include "vcs_state_machine.h"

// Global Telemetry Variable for State Machine & UART
extern bool is_brake_pressed;

void initLowBrake();
void updateLowBrake();

// Allows the State Machine to lock the brakes during FAULT/INIT
void forceBrakeEngagement(bool engage);

// Helper function
bool isPhysicalBrakePressed();

#endif // VCS_LOWBRAKE_H