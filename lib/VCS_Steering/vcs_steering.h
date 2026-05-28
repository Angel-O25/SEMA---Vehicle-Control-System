#ifndef VCS_STEERING_H
#define VCS_STEERING_H

#include <Arduino.h>
#include <QuickPID.h>
#include "vcs_pins.h"
#include "vcs_constants.h"
#include "vcs_state_machine.h"
#include "vcs_simulation.h"

void     initSteering();
uint16_t getMeasuredSteering();              // filtered, 0-1000 protocol units
void     updateSteeringPID(uint16_t target_position, bool is_automatic);

// Debug getter — raw unfiltered pot mV
uint32_t getSteeringRawMv();

#endif // VCS_STEERING_H
