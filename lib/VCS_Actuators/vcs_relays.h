#ifndef VCS_RELAYS_H
#define VCS_RELAYS_H

#include <Arduino.h>

// Initialize the relay pins and set them to default Manual state
void initRelays();

// Polled in the CommTask to trigger the physical relays based on system state
void updateRelays(bool isAutonomous);

#endif // VCS_RELAYS_H