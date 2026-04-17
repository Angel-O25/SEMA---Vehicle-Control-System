#ifndef VCS_THROTTLE_H
#define VCS_THROTTLE_H

#include <Arduino.h>
#include "vcs_pins.h"
#include "vcs_constants.h"
#include "vcs_state_machine.h"
#include <QuickPID.h>


// Variables exposed for telemetry and state machine.
// Changed current_pwm_duty to uint16_t to support 10-bit (0-1023) resolution without overflowing.
extern uint16_t current_throttle_adc;
extern uint16_t current_pwm_duty;

void initThrottle();
void updateThrottle(float measured_rpm, float target_rpm);
bool isThrottlePedalPressed();

#endif // VCS_THROTTLE_H