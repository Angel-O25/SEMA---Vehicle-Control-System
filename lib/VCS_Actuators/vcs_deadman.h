#ifndef VCS_DEADMAN_H
#define VCS_DEADMAN_H

#include <Arduino.h>

// Initializes the hardware pins with internal pull-ups
void initDeadman();

// Polled at 100Hz to read the hardware switches and apply debounce filtering
void updateDeadman();

// Returns true ONLY if BOTH switches are actively held down
bool isDeadmanActive();

#endif // VCS_DEADMAN_H