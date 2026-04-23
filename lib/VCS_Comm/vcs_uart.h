#ifndef VCS_UART_H
#define VCS_UART_H

#include <Arduino.h>
#include "vcs_pins.h"
#include "vcs_constants.h"
#include "vcs_threespeed.h"



extern DriveMode current_drive_mode; // Explicitly pull the variable from vcs_threespeed.cpp

// Core UART functions
void initUART();
void updateUART();
void sendTelemetry(float rpm, uint16_t steer, float volt, uint8_t state);
void broadcastVehicleTelemetry();

// Data Retrieval Functions
uint8_t getANSCommandMode(); 
float getTargetRPM(); 
uint16_t getTargetSteering(); 
uint8_t getTargetBrake(); 
bool getANSReverseCommand();

// Security checks
bool ansHeartbeatReceived();

#endif // VCS_UART_H