#ifndef VCS_SIMULATION_H
#define VCS_SIMULATION_H

#include <Arduino.h>
#include "vcs_constants.h"

void updateSimulatedPhysics(int pulse_freq, bool direction);
float getSimulatedRPM();
float getSimulatedSteering();

#endif // VCS_SIMULATION_H