#ifndef VCS_SIMULATION_H
#define VCS_SIMULATION_H

#include <Arduino.h>
#include "vcs_constants.h"

// Note: simulated state is file-static inside vcs_simulation.cpp and is only
// reachable through the getters below. Do NOT re-add extern globals - they
// let any translation unit stomp the digital twin's state unnoticed.

/**
 * @brief Advance the physics "digital twin" one control tick.
 * @param pulse_freq  Current frequency commanded to the stepper (Hz).
 * @param direction   true = Right, false = Left.
 *
 * Call rate is assumed to be FREQ_STEER_CTRL_HZ (100 Hz / 10 ms). If you
 * invoke this from the 1 kHz core loop instead, the integration step and the
 * motor-inertia filter must be re-derived - see vcs_simulation.cpp.
 *
 * Throttle state is read from the global current_pwm_duty inside the .cpp
 * rather than passed in, to keep the sim aligned with whatever vcs_throttle
 * actually wrote to the PWM pin that tick.
 */
void updateSimulatedPhysics(int pulse_freq, bool direction);

float getSimulatedRPM();
float getSimulatedSteering();

#endif // VCS_SIMULATION_H