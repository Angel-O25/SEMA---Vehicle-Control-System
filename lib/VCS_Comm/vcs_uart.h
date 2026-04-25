#ifndef VCS_UART_H
#define VCS_UART_H

#include <Arduino.h>
#include "vcs_pins.h"
#include "vcs_constants.h"
#include "vcs_threespeed.h"

// Pulled in for the gear byte in broadcastVehicleTelemetry()
extern DriveMode current_drive_mode;

// =========================================================
// Core UART functions
// =========================================================
// Single Serial2 reader. Call from EXACTLY ONE task (task_control)
// to avoid a hardware-FIFO race between cores.
void initUART();
void handleIncomingUART();
void updateUART();

// Telemetry TX. Safe to call from any single task.
void broadcastVehicleTelemetry();

// One-time setup
void printHexDebug(const char* prefix, const uint8_t* data, uint8_t length);



// =========================================================
// Thread-safe getters (mux-protected reads of last RX command)
// =========================================================
uint8_t  getANSCommandMode();
float    getTargetRPM();
uint16_t getTargetSteering();
uint8_t  getTargetBrake();
bool     getANSReverseCommand();
bool     ansHeartbeatReceived();
bool     isJetsonStopLineActive();

void printHexDebug();// =========================================================
// Legacy unprotected globals — kept for source-compat with
// modules that read them directly. New code should use the
// getters above.
// =========================================================
extern uint8_t  current_target_brake;
extern int16_t  current_target_rpm;
extern uint16_t current_target_steer;
extern uint8_t  current_target_mode;
extern uint32_t last_uart_time;

#endif // VCS_UART_H