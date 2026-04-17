#ifndef VCS_SIMULATION_H
#define VCS_SIMULATION_H

#include <Arduino.h>
#include "vcs_constants.h"

// Global Simulated Variables exposed for getters
extern float sim_steer_pos; 
extern float sim_motor_rpm;

// Legacy signature preserved, but throttle_pwm defaults to 0 as it now reads globally
void updateSimulatedPhysics(int pulse_freq, bool direction, uint16_t throttle_pwm = 0);

float getSimulatedRPM();
float getSimulatedSteering();

#endif // VCS_SIMULATION_H