#ifndef VCS_DEADMAN_H
#define VCS_DEADMAN_H

#include <Arduino.h>

// Configures GPIO 33 + 27 as INPUT_PULLDOWN (active-HIGH switches, no PCB pull).
void initDeadman();

// Polled at 100Hz to debounce both DMS switches.
void updateDeadman();

// Returns true ONLY if BOTH grips have been actively held (debounced).
bool isDeadmanActive();          // BOTH switches held (AUTONOMOUS)
bool isEitherDeadmanHeld();      // EITHER switch held (INIT→IDLE)

#endif // VCS_DEADMAN_H