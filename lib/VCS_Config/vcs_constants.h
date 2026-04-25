#ifndef VCS_CONSTANTS_H
#define VCS_CONSTANTS_H

#include <Arduino.h>

// ==========================================
// System Architecture & Simulation
// ==========================================
#define SIMULATION_MODE 0      // 1 = Digital Twin Mode, 0 = LIVE 1500W BLDC Control
#define V_LOGIC         3.3f   // ESP32 logic level (used by ADC scaling helpers)

#if SIMULATION_MODE
  #pragma message ("VCS BUILD >>> SIMULATION_MODE = 1  (Digital Twin, no live motor output)")
#else
  #pragma message ("VCS BUILD >>> SIMULATION_MODE = 0  (LIVE 1500W MOTOR CONTROL - verify E-stop)")
#endif

// ==========================================
// System Frequencies & Timing
// ==========================================
#define FREQ_CONTROL_HZ     1000  // Core control loop (1 ms)
#define FREQ_STEER_CTRL_HZ  100   // Steering inner loop (10 ms)
#define FREQ_COMM_HZ        100   // Comms / inputs sweep (10 ms)
#define FREQ_UI_HZ          20    // OLED + telemetry (50 ms)
#define DEBOUNCE_TIME_MS    50    // Debounce window for physical brake

// ==========================================
// Motor & Powertrain
// ==========================================
// IMPORTANT: pole-pair count MUST match the actual motor on the car.
// Verify by spinning the wheel one mechanical revolution by hand,
// counting Hall transitions, and dividing by 6.
//
// VCS_TECHNICAL_SPECIFICATION lists POLE_PAIRS = 23 with
// HALL_TRANSITIONS_PER_MECH_REV = 138 (= 6 × 23). The value below
// (16) is from an earlier bench measurement. RECONCILE before first
// live run — these cannot both be right.
#define MOTOR_POLE_PAIRS      16
#define HALL_TRANSITIONS_REV  6     // 6 transitions per electrical cycle
#define GEAR_REDUCTION        1.0f

// ==========================================
// Throttle Output (ESP32 DAC, 8-bit native)
// ==========================================
// GPIO25 -> LM358 -> motor controller.  dacWrite() takes 0–255.
// --- Legacy Bridging Macros for Throttle PID (10-bit scale) ---
#define MIN_PWM_OUT 0
#define MAX_PWM_OUT 1023
#define THROTTLE_MIN_INPUT 180
#define THROTTLE_MAX_INPUT 850
// ==========================================
// Speed PI (target RPM -> DAC value)
// ==========================================
// Gains scaled from the legacy 0–1023 PWM values (KP=0.8, KI=0.15) by
// 1/4 to match the 0–255 DAC output range. These are STARTING POINTS
// only — re-tune on the bench against the real motor controller.
#define SPEED_KP              0.20f
#define SPEED_KI              0.0375f

// ==========================================
// Steering PID (target -> stepper effort)
// ==========================================
// PID setpoint and input both run in COMM units (0..1000 across full
// steering travel), so the deadzone is in the same space.
#define STEER_KP              1.2f
#define STEER_KI              0.05f
#define STEER_KD              0.01f
#define STEER_DEADZONE        5     // COMM units (~0.5% of full travel)

// ==========================================
// Brake Actuator (TB6612 + 12V linear)
// ==========================================
#define BRAKE_PWM             200   // 0–255, applied to TB6612 PWMA/PWMB
#define BRAKE_RETRACT_MS      900   // Time-limited retract (no lower limit switch).
                                    // Calibrate physically: power the actuator at
                                    // BRAKE_PWM and time a full retract from
                                    // engaged to fully released.

// ==========================================
// Communication Protocol Ranges (ANS -> VCS)
// ==========================================
#define COMM_SPEED_MIN        0
#define COMM_SPEED_MAX        3000  // Maximum target RPM
#define COMM_STEER_LEFT       0
#define COMM_STEER_CENTER     500
#define COMM_STEER_RIGHT      1000
#define COMM_BRAKE_MIN        0
#define COMM_BRAKE_MAX        1     // Binary (0=Off, 1=On)

// ==========================================
// Physical Interface Mapping (ESP32 ADC -> mV)
// ==========================================
// Thresholds below are in MILLIVOLTS as returned by
// esp_adc_cal_raw_to_voltage().  Always go through the calibrated
// reader — never compare these against raw analogRead() values.

// --- Throttle pedal (via 10k/18k divider, ~0–3210 mV at full press) ---
#define THROTTLE_DEADBAND_MV   50
#define THROTTLE_MIN_INPUT_MV  150
#define THROTTLE_MAX_INPUT_MV  3000

// --- Steering pot (3590S, 3.3V powered) ---
#define STEER_POT_MIN_MV       200
#define STEER_POT_CENTER_MV    1650
#define STEER_POT_MAX_MV       3100

#endif // VCS_CONSTANTS_H