#ifndef VCS_CONSTANTS_H
#define VCS_CONSTANTS_H

#include <Arduino.h>

// ============================================================
//  vcs_constants.h — SIDLAK 2 VCS
//  Team Wired PH0017003 | Shell Eco-marathon 2026
// ============================================================

#define SIMULATION_MODE 0
#define V_LOGIC         3.3f

#if SIMULATION_MODE
  #pragma message ("VCS BUILD >>> SIMULATION_MODE = 1  (Digital Twin)")
#else
  #pragma message ("VCS BUILD >>> SIMULATION_MODE = 0  (LIVE 1500W MOTOR CONTROL)")
#endif

#define WHEEL_CIRCUMFERENCE_M 1.2764f

#define FREQ_CONTROL_HZ    1000
#define FREQ_STEER_CTRL_HZ  100
#define FREQ_COMM_HZ        100
#define FREQ_UI_HZ           20
#define DEBOUNCE_TIME_MS     50

#define COMM_SPEED_MIN      0
#define COMM_SPEED_MAX      3000
#define COMM_STEER_LEFT     0
#define COMM_STEER_CENTER   500
#define COMM_STEER_RIGHT    1000
#define COMM_BRAKE_MIN      0
#define COMM_BRAKE_MAX      100
#define STEER_PUL_HIGH_US   5000

#define VCS_FAULT_NONE               0x00000000u
#define VCS_FAULT_UART_CRC           0x00000001u
#define VCS_FAULT_HEARTBEAT_LOST     0x00000002u
#define VCS_FAULT_SENSOR_SPIKE       0x00000004u
#define VCS_FAULT_OVERCURRENT        0x00000008u
#define DMS_HOLD_REQUIRED_MS         1000u
constexpr uint16_t FAULT_SIGNAL_TIMEOUT = 0x0004;

// ============================================================
//  LEGACY BRIDGING MACROS (0-1023 PWM -> 0-255 DAC pipeline)
//
//  Motor start threshold (no-load tachometer):
//    throttle 29% -> DAC 74  -> motor just starts
//    throttle 40% -> DAC 102 -> 173 RPM
//    throttle 60% -> DAC 153 -> 417 RPM
//    throttle 70% -> DAC 178 -> 537 RPM (capped)
//
//  DAC = map(pwm_duty, 0, 1023, 0, 255)
//  MIN_PWM_OUT to get DAC>=74: 74/255 * 1023 = 297
//
//  DAC=0 is output EXPLICITLY when target=0 or motor should stop.
//  MIN_PWM_OUT only applies when motor IS commanded to move.
// ============================================================
#define MIN_PWM_OUT        297    // FIXED: was 0 (motor unresponsive at low commands)
#define MAX_PWM_OUT       1023
#define THROTTLE_MIN_INPUT 310    // = map(650mV, 0, 3300, 0, 1023)
#define THROTTLE_MAX_INPUT 930    // = map(3000mV, 0, 3300, 0, 1023)

#include "vcs_calibration.h"

#endif // VCS_CONSTANTS_H