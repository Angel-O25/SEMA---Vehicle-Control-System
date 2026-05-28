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
#define HALL_TRANSITIONS_PER_MECH_REV 138
#define GEAR_REDUCTION                2.0f       // direct drive
#define RPM_CALIBRATION_FACTOR        0.9587f    // measured correction
#define HALL_DEBOUNCE_US              800      // motor PWM noise filter
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
#define STEER_POT_MIN_MV             142.0f      // full-left
#define STEER_POT_CENTER_MV          1502.0f     // mechanical center
#define STEER_POT_MAX_MV             3145.0f     // full-right
#define STEPS_FULL_L                -2000.0f    // stepper position at full-left
#define STEPS_FULL_R                 2000.0f    // stepper position at full-right


// ============================================================
//  SECTION 5 — STEERING MODULE TUNING
//  Stepper PID gains and DM542 driver timing.
// ============================================================
#define STEER_EMA_ALPHA              0.15f
#define STEER_KP                     0.4f
#define STEER_KI                     0.02f
#define STEER_KD                     0.01f
#define STEER_DEADZONE               5         // COMM units
#define STEPPER_DEFAULT_HZ           1000
#define STEER_DM542_DIR_SETUP_US     5         // DM542 DIR-before-PUL spec
#define STEPPER_MAX_HZ               800
#define STEPPER_PULSES_PER_REV       400
#define STEER_COMM_LEFT_SAFE    200   // don't steer past here (left)
#define STEER_COMM_RIGHT_SAFE   800   // don't steer past here (right)

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

// ============================================================
//  SECTION 7 — THROTTLE OPEN-LOOP FEEDFORWARD
//  No-load bench-calibrated DAC vs RPM values.
//  Re-calibrate under load after first driving test.
// ============================================================

// DAC ceiling — never exceed regardless of Jetson command
// 178 = 70% throttle = ~537 RPM no-load.
// Raise to 200 only after confirming motor handles it under load.
#define THROTTLE_MAX_DAC         178

// Minimum DAC for reliable motor start (bench: 29% = first movement)
// Below this the motor doesn't start spinning. Never go lower.
#define THROTTLE_OL_START_DAC     74

// Minimum DAC for continuous reliable running (bench: 40%)
// Motor can start at 74 but runs reliably from this value.
#define THROTTLE_OL_MIN_DAC      102

// RPM anchors matching the DAC values above (no-load measurements)
// After loaded test, adjust these to match real loaded RPM.
#define THROTTLE_OL_MIN_RPM      173    // RPM at THROTTLE_OL_MIN_DAC
#define THROTTLE_OL_MAX_RPM      537    // RPM at THROTTLE_MAX_DAC


// ============================================================
//  SECTION 8 — MOTION SMOOTHING & FILTER TUNING
//  *** THIS IS THE SECTION TO ADJUST FOR SMOOTHNESS ***
//
//  All values have been chosen as safe starting points.
//  Lower values = smoother/slower. Higher = more responsive/jittery.
//  After hardware changes, re-tune Section 5 PID gains first,
//  then fine-tune smoothing here.
// ============================================================

// ── Steering pot EMA filter (applied to raw ADC mV) ──────────
// Exponential Moving Average: output = alpha*new + (1-alpha)*prev
// Reduces ADC noise before it reaches the PID controller.
//   1.00 = no filter (raw, very noisy — never use)
//   0.30 = light smoothing (fast response, some jitter)
//   0.15 = moderate smoothing (recommended for competition)
//   0.08 = heavy smoothing (very smooth, ~80ms lag)
// If steering feels sluggish to human input: increase toward 0.30.
// If steering jitters/hunts at rest: decrease toward 0.08.
#define STEER_ADC_EMA_ALPHA      0.15f

// ── Steering PID deadband ─────────────────────────────────────
// Error in COMM units (0–1000 scale) below which motor holds.
// 1 COMM unit ≈ 3mV ≈ 0.036° of steering angle.
// Higher = less micro-corrections but less precise final position.
//   3  = tight (competition — precise but may hunt slightly)
//   8  = moderate (recommended starting point)
//   15 = relaxed (smooth, ±0.54° tolerance — good for rough surfaces)
// If motor keeps twitching around center: increase this value.
#undef  STEER_DEADZONE
#define STEER_DEADZONE           8.0f

// ── Stepper ramp rate (Hz per PID tick) ──────────────────────
// Controls acceleration/deceleration of the stepper motor.
// PID runs at 100Hz so: RATE × 100 = Hz per second of acceleration.
//   80 = 8000 Hz/s → reaches 800Hz in ~100ms (responsive)
//   40 = 4000 Hz/s → reaches 800Hz in ~200ms (smooth — recommended)
//   20 = 2000 Hz/s → reaches 800Hz in ~400ms (very gentle, slow corners)
// If motor sounds rough on direction change: decrease this value.
#define STEER_RAMP_RATE          40

// ── Stepper minimum run frequency ────────────────────────────
// Motor will never step slower than this (below minimum it stalls).
// DQ860MA reliable minimum is ~80Hz at 400 steps/rev.
// Increasing this makes slow approach to target more aggressive.
//   50 = very slow creep (may stall under load)
//   80 = reliable minimum (recommended)
//  120 = no slow creep — snaps to position
#define STEER_MIN_HZ             80

// ── Stepper frequency update threshold ───────────────────────
// Only reconfigure LEDC when frequency changes by this many Hz.
// Each ledcSetup() call causes one spurious pulse (glitch).
// Higher threshold = fewer glitches = smoother but less granular speed.
//   5  = very granular (glitches on almost every tick — avoid)
//   15 = recommended (smooth, rarely glitches)
//   30 = coarse speed steps (very smooth, slight speed quantization)
#define STEER_FREQ_UPDATE_HZ     15

// ── DMS hold time before AUTONOMOUS entry ─────────────────────
// How long driver must hold DMS + Jetson must be in AUTO mode
// before FSM transitions to AUTONOMOUS.
// Prevents accidental autonomous entry.
//   500ms = quick (good for testing)
//  1000ms = one second hold (recommended for competition)
//  2000ms = extra safe (for first outdoor tests)
#define DMS_HOLD_REQUIRED_MS     1000

#endif // VCS_CALIBRATION_H