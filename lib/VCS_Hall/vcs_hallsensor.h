#ifndef VCS_HALLSENSOR_H
#define VCS_HALLSENSOR_H

#include <Arduino.h>
#include "vcs_pins.h"
#include "vcs_constants.h"
#include "vcs_simulation.h"

// Volatile variables for ISR communication00
void initHallSensors();

// Hall sensor processing logic, called from the ControlTask thread
void updateHallCalculations();

// Functions for Telemetry
float getMeasuredRPM();

#endif // VCS_HALLSENSOR_H