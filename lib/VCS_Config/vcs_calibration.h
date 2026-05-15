// VCS_Config\vcs_calibration.h
#ifndef VCS_CALIBRATION_H
#define VCS_CALIBRATION_H

// ============================================================
//  vcs_calibration.h — SIDLAK 2 VCS
//  Team Wired PH0017003 | Shell Eco-marathon 2026
//
//  All values bench-calibrated using REV 4.4 test firmware.
//  Source of truth for measurements: VCS Hardware Test Firmware
//  on the actual SIDLAK 2 vehicle.
// ============================================================


// ============================================================
//  SECTION 1 — MOTOR & HALL SENSOR
//  Single Hall C (GPIO32). 46 transitions/rev = 2 × 23 pole pairs.
//  Calibration factor and debounce empirically tuned via test
//  firmware against motor controller PWM noise.
// ============================================================
#define MOTOR_POLE_PAIRS              23
#define HALL_TRANSITIONS_PER_MECH_REV 46
#define GEAR_REDUCTION                2.0f       // direct drive
#define RPM_CALIBRATION_FACTOR        0.9587f    // measured correction
#define HALL_DEBOUNCE_US              2000       // motor PWM noise filter
#define RPM_SAMPLE_WINDOW_MS          500
#define RPM_TIMEOUT_MS                1000
#define MAX_SPEED_KMPH                60.0f      // sanity reject above this
#define WHEEL_CIRCUMFERENCE_M         1.2764f


// ============================================================
//  SECTION 2 — THROTTLE ADC CALIBRATION
//  Pedal via 5V→3.3V resistor divider into ADC1_CH6 (GPIO34).
//  Output: DAC1 (GPIO25) → LM358 on 12V supply → 0–4.72V (gain 1.43).
// ============================================================
#define THROTTLE_DEADBAND_MV         600
#define THROTTLE_MIN_INPUT_MV        650
#define THROTTLE_MAX_INPUT_MV        3000
#define THROTTLE_R1_OHMS             10000.0f
#define THROTTLE_R2_OHMS             18000.0f
#define THROTTLE_LM358_GAIN          1.43f


// ============================================================
//  SECTION 3 — THROTTLE MODULE TUNING
// ============================================================
#define THROTTLE_EMA_ALPHA           0.15f
#define SPEED_KP                     0.20f
#define SPEED_KI                     0.0375f


// ============================================================
//  SECTION 4 — STEERING POT ADC CALIBRATION
//  3590S pot powered at 3.3V, ADC1_CH7 (GPIO35).
//  Values bench-measured via 'set full_l/full_r/center' commands.
// ============================================================
#define STEER_POT_MIN_MV             200.0f      // full-left
#define STEER_POT_CENTER_MV          1666.0f     // mechanical center
#define STEER_POT_MAX_MV             2776.0f     // full-right
#define STEPS_FULL_L                -2000.0f    // stepper position at full-left
#define STEPS_FULL_R                 2000.0f    // stepper position at full-right


// ============================================================
//  SECTION 5 — STEERING MODULE TUNING
//  Stepper PID gains and DM542 driver timing.
// ============================================================
#define STEER_EMA_ALPHA              0.15f
#define STEER_KP                     1.2f
#define STEER_KI                     0.05f
#define STEER_KD                     0.01f
#define STEER_DEADZONE               5         // COMM units
#define STEER_DM542_DIR_SETUP_US     5         // DM542 DIR-before-PUL spec
#define STEPPER_DEFAULT_HZ           1000
#define STEPPER_MAX_HZ               5000
#define STEPPER_PULSES_PER_REV       400


// ============================================================
//  SECTION 6 — BRAKE ACTUATOR CALIBRATION
//  TB6612FNG driving 12V linear actuator.
//  PWM via LEDC channel 1 @10kHz (channel 0 reserved for stepper).
// ============================================================
#define BRAKE_PWM                    200      // bench-verified
#define BRAKE_RETRACT_MS             900
#define BRAKE_EXTEND_TIMEOUT_MS      3000
#define BRAKE_LEDC_CH                1
#define BRAKE_LEDC_FREQ              10000
#define BRAKE_LEDC_RES               8


#endif // VCS_CALIBRATION_H