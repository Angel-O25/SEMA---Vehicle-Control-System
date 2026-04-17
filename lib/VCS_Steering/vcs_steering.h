#ifndef VCS_STEERING_H
#define VCS_STEERING_H

#include <Arduino.h>
#include <QuickPID.h>
#include "vcs_pins.h"
#include "vcs_constants.h"
#include "vcs_state_machine.h"
#include "vcs_simulation.h"

void initSteering();
uint16_t getMeasuredSteering();
void updateSteeringPID(uint16_t target_position, bool is_automatic);

#endif // VCS_STEERING_H