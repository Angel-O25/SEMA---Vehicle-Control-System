#ifndef VCS_HALLSENSOR_H
#define VCS_HALLSENSOR_H

#include <Arduino.h>
#include "vcs_pins.h"
#include "vcs_constants.h"
#include "vcs_simulation.h"

void initHallSensors();
void hall_interrupts_attach();
void hall_interrupts_detach();
void updateHallCalculations();

float getMeasuredRPM();
float getMeasuredSpeedKmh();
bool consumeNewRPMSample();

#endif // VCS_HALLSENSOR_H