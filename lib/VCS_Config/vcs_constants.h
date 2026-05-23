#ifndef VCS_CONSTANTS_H
#define VCS_CONSTANTS_H

#include <Arduino.h>

// ============================================================
//  vcs_constants.h — SIDLAK 2 VCS
//  Team Wired PH0017003 | Shell Eco-marathon 2026
//
//  Fixed system constants only — build flags, logic level,
//  task frequencies, protocol ranges, fault codes, and
//  legacy bridging macros.
//
//  CALIBRATABLE VALUES (PID gains, ADC endpoints, brake timing,
//  pot measurements, hall transitions) have been moved to:
//      vcs_calibration.h
//
//  This file includes vcs_calibration.h at the bottom, so all
//  existing #include "vcs_constants.h" in the codebase continue
//  to work without modification.
// ============================================================


// ============================================================
//  SYSTEM ARCHITECTURE & BUILD FLAGS
// ============================================================
#define SIMULATION_MODE 0   // 1 = Digital Twin, 0 = LIVE 1500W motor control
#define V_LOGIC         3.3f  // ESP32 logic level — used by ADC scaling helpers

#if SIMULATION_MODE
  #pragma message ("VCS BUILD >>> SIMULATION_MODE = 1  (Digital Twin, no live motor output)")
#else
  #pragma message ("VCS BUILD >>> SIMULATION_MODE = 0  (LIVE 1500W MOTOR CONTROL - verify E-stop)")
#endif

// ============================================================
//  PHYSICAL CONSTANTS
#define WHEEL_CIRCUMFERENCE_M 1.2764f


// ============================================================
//  TASK FREQUENCIES & TIMING
//  These are system architecture decisions, not tunable values.
//  Changing them affects FreeRTOS task scheduling — do not
//  adjust without reviewing all vTaskDelay() calls.
// ============================================================
#define FREQ_CONTROL_HZ    1000   // Core control loop    (1ms tick)
#define FREQ_STEER_CTRL_HZ  100   // Steering inner loop  (10ms tick)
#define FREQ_COMM_HZ        100   // Comms / inputs sweep (10ms tick)
#define FREQ_UI_HZ           20   // OLED + telemetry     (50ms tick)
#define DEBOUNCE_TIME_MS     50   // Physical brake switch debounce window


// ============================================================
//  COMMUNICATION PROTOCOL RANGES (Jetson → VCS)
//  These are the agreed protocol values between ESP32 and
//  Jetson. Do not change without updating both ends.
// ============================================================
#define COMM_SPEED_MIN      0
#define COMM_SPEED_MAX      3000    // Maximum target RPM from Jetson
#define COMM_STEER_LEFT     0
#define COMM_STEER_CENTER   500
#define COMM_STEER_RIGHT    1000
#define COMM_BRAKE_MIN      0
#define COMM_BRAKE_MAX      100     // 0-100% matches Jetson protocol


// ============================================================
//  FAULT CODES
// ============================================================
// --- Fault Bit Definitions ---
// Recoverable faults map to FAULT_STATE and clear when the underlying
// condition resolves.
#define VCS_FAULT_NONE               0x00000000u
#define VCS_FAULT_UART_CRC           0x00000001u   // CRC mismatch on command packet
#define VCS_FAULT_HEARTBEAT_LOST     0x00000002u   // ANS heartbeat timeout
#define VCS_FAULT_SENSOR_SPIKE       0x00000004u   // Hall / steering pot out-of-range
#define VCS_FAULT_OVERCURRENT        0x00000008u   // Motor controller fault line

// --- Safety Timing Constants ---
// (Candidate to relocate into vcs_constants.h.)
#define DMS_HOLD_REQUIRED_MS         1000u         // 1s confirm for AUTONOMOUS entry

// Jetson command timeout in autonomous mode.
// Triggered when no valid UART packet arrives within
// SIGNAL_TIMEOUT_MS (defined in main.cpp).
constexpr uint16_t FAULT_SIGNAL_TIMEOUT = 0x0004;


// ============================================================
//  LEGACY BRIDGING MACROS
//  Speed PI was originally written for a 0–1023 PWM output.
//  These let legacy throttle PID code compile unchanged while
//  the actual output path uses the 0–255 DAC range.
//  Do not use in new code — reference THROTTLE_MIN/MAX_INPUT_MV
//  from vcs_calibration.h directly.
// ============================================================
#define MIN_PWM_OUT       0
#define MAX_PWM_OUT       1023
#define THROTTLE_MIN_INPUT 201    // = map(650mV, 0, 3300mV, 0, 1023)
#define THROTTLE_MAX_INPUT 930   // = map(3000mV, 0, 3300mV, 0, 1023)


// ============================================================
//  CALIBRATABLE VALUES
//  Moved to vcs_calibration.h. Included here so all existing
//  #include "vcs_constants.h" continue to compile unchanged.
// ============================================================
#include "vcs_calibration.h"

#endif // VCS_CONSTANTS_H