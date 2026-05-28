#ifndef VCS_THROTTLE_H
#define VCS_THROTTLE_H

#include <Arduino.h>
#include "vcs_pins.h"
#include "vcs_constants.h"
#include "vcs_state_machine.h"

extern uint16_t current_throttle_adc;
extern uint16_t current_pwm_duty;

void     initThrottle();
void     updateThrottle(float measured_rpm, float target_rpm);
bool     isThrottlePedalPressed();
uint8_t  getThrottleDacOut();
uint32_t getThrottlePedalMv();
float    getThrottleEstimatedRPM();  // open-loop RPM estimate from DAC

#endif // VCS_THROTTLE_H