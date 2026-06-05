// ============================================================
//  vcs_calibration.h — SIDLAK 2 VCS
//  Team Wired PH0017003 | Shell Eco-marathon 2026
// ============================================================

#ifndef VCS_CALIBRATION_H
#define VCS_CALIBRATION_H

// ============================================================
//  SECTION 1 — MOTOR & HALL SENSOR
// ============================================================
#define MOTOR_POLE_PAIRS              23
#define HALL_TRANSITIONS_PER_MECH_REV 138
#define GEAR_REDUCTION                2.0f
#define RPM_CALIBRATION_FACTOR        0.9587f
#define HALL_DEBOUNCE_US              800
#define RPM_SAMPLE_WINDOW_MS          500
#define RPM_TIMEOUT_MS                1000
#define MAX_SPEED_KMPH                60.0f
#define WHEEL_CIRCUMFERENCE_M         1.2764f

// ============================================================
//  SECTION 2 — THROTTLE ADC CALIBRATION
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
//
//  PHYSICAL MAPPING (confirmed on hardware):
//    Low  mV (~142)  = physical RIGHT end  ← pot wired this way
//    High mV (~3145) = physical LEFT  end
//    Center (~1502)  = steering center
//
//  NOTE: Comments on MIN/MAX refer to physical position labels
//  in the original firmware which used the opposite convention.
//  The mapping swap in getMeasuredSteering() corrects this:
//    map(mV, MIN_MV, MAX_MV, COMM_RIGHT, COMM_LEFT)
//    → low mV = COMM 1000 (right), high mV = COMM 0 (left)
// ============================================================
#define STEER_POT_MIN_MV             142.0f    // physical RIGHT end (low  mV)
#define STEER_POT_CENTER_MV          1502.0f   // mechanical center
#define STEER_POT_MAX_MV             3145.0f   // physical LEFT  end (high mV)
#define STEPS_FULL_L                -2000.0f
#define STEPS_FULL_R                 2000.0f

// ============================================================
//  SECTION 5 — STEERING MOTOR TUNING
// ============================================================

// ── Direction flip ────────────────────────────────────────────
// If steering goes the wrong physical direction, set this true.
// true  = HIGH pin → right, LOW pin → left
// false = HIGH pin → left,  LOW pin → right  (current wiring)
#define STEER_DIR_FLIP               false

// ── Safe software travel limits (COMM units 0-1000) ──────────
// Prevent commanding past the physical steering stops.
// Widen after confirming no mechanical binding at these limits.
#define STEER_COMM_LEFT_SAFE         200
#define STEER_COMM_RIGHT_SAFE        800

// ── mV deadband ───────────────────────────────────────────────
// Motor holds when within this many mV of target.
// Larger = less hunting between gear teeth.
// 100mV ≈ 3-4 COMM units ≈ ~1 gear tooth step.
#define STEER_MV_DEADBAND            100.0f

// ── EMA filter for pot ADC reading ───────────────────────────
#define STEER_EMA_ALPHA              0.15f

// ── Stepper motor speed ───────────────────────────────────────
#define STEPPER_MAX_HZ               800    // max step frequency
#define STEER_MIN_HZ                 80     // min (stall-safe)
#define STEPPER_DEFAULT_HZ           800
#define STEPPER_PULSES_PER_REV       400    // DQ860MA SW5-SW8=ON

// ── LEDC glitch suppression ───────────────────────────────────
// ledcSetup() causes one spurious pulse. Only reconfigure when
// frequency changes by at least this many Hz.
#define STEER_FREQ_UPDATE_HZ         15

// ── DM542 / DQ860 DIR setup time ─────────────────────────────
#define STEER_DM542_DIR_SETUP_US     5

// ============================================================
//  SECTION 6 — BRAKE ACTUATOR CALIBRATION
// ============================================================
#define BRAKE_PWM                    200
#define BRAKE_RETRACT_MS             2900
#define BRAKE_EXTEND_TIMEOUT_MS      3000
#define BRAKE_LEDC_CH                1
#define BRAKE_LEDC_FREQ              10000
#define BRAKE_LEDC_RES               8

// ============================================================
//  SECTION 7 — THROTTLE OPEN-LOOP FEEDFORWARD
// ============================================================
#define THROTTLE_MAX_DAC             178
#define THROTTLE_OL_START_DAC         74
#define THROTTLE_OL_MIN_DAC          102
#define THROTTLE_OL_MIN_RPM          173
#define THROTTLE_OL_MAX_RPM          537

// ============================================================
//  SECTION 8 — MOTION SMOOTHING & FILTER TUNING
// ============================================================
#undef  STEER_EMA_ALPHA
#define STEER_ADC_EMA_ALPHA          0.15f
#define STEER_EMA_ALPHA              0.15f

// Deadband in COMM units — kept for any code still referencing it
// Actual control now uses STEER_MV_DEADBAND (Section 5)
#define STEER_DEADZONE               8.0f

#define STEER_RAMP_RATE              40
#define DMS_HOLD_REQUIRED_MS         1000

#endif // VCS_CALIBRATION_H