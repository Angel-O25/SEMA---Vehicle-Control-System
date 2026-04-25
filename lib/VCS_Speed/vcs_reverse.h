#ifndef VCS_REVERSE_H
#define VCS_REVERSE_H

#include <Arduino.h>

void initReverse();
void updateReverse();
bool isReverseEngaged();
bool isReverseSwitchPressed();

#endif // VCS_REVERSE_H