#ifndef VCS_REVERSE_H
#define VCS_REVERSE_H

#include <Arduino.h>

void initReverse();
void updateReverse();

// Returns true if the hardware is actively pulling the controller into reverse
bool isReverseEngaged();

// Returns true if the physical switch is flipped (used for Auto override)
bool isReverseSwitchPressed();

#endif // VCS_REVERSE_H