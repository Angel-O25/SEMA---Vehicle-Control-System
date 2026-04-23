#ifndef VCS_HALLSENSOR_H
#define VCS_HALLSENSOR_H

#include <Arduino.h>
#include "vcs_pins.h"
#include "vcs_constants.h"
#include "vcs_simulation.h"

// Volatile variables for ISR communication
void initHallSensors();
void hall_interrupts_attach();
void hall_interrupts_detach();

// Hall sensor processing logic, called from the ControlTask thread
void updateHallCalculations();

// Functions for Telemetry
float getMeasuredRPM();

// Returns true exactly once per new RPM sample computed by
// updateHallCalculations(). Consumers (e.g. the throttle PID) use this to
// avoid running Ki on stale samples at the 1 kHz control rate while the
// Hall window is only 100 ms wide.
bool consumeNewRPMSample();

#endif // VCS_HALLSENSOR_H