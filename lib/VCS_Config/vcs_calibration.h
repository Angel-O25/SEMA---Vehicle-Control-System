#ifndef VCS_CALIBRATION_H
#define VCS_CALIBRATION_H

// ============================================================
//  vcs_calibration.h — SIDLAK 2 VCS
//  Team Wired PH0017003 | Shell Eco-marathon 2026
//
//  All values in this file require physical measurement or
//  bench/track tuning. Nothing here is a fixed system constant.
//
//  WORKFLOW:
//    1. Measure physical values (ADC endpoints, Hall count).
//    2. Update defines here.
//    3. Re-run wizard to confirm on car.
//    4. Tune PID/PI/EMA gains last, after mechanical cal is done.
//
//  DO NOT put protocol ranges, task frequencies, or fault codes
//  here — those belong in vcs_constants.h.
//
//  SECTIONS:
//    1. Motor & Hall sensor
//    2. Throttle ADC calibration
//    3. Throttle module tuning (EMA, PI)
//    4. Steering pot ADC calibration
//    5. Steering module tuning (EMA, PID)
//    6. Brake actuator calibration
// ============================================================


// ============================================================
//  SECTION 1 — MOTOR & HALL SENSOR
//
//  *** CRITICAL DISCREPANCY — RESOLVE BEFORE FIRST LIVE RUN ***
//
//  MOTOR_POLE_PAIRS = 23 with HALL_TRANSITIONS_PER_MECH_REV = 46
//  assumes a SINGLE Hall sensor (Hall C, GPIO32 only) with a
//  CHANGE interrupt: transitions/rev = pole_pairs × 2 = 23 × 2 = 46.
//
//  An earlier spec listed 138 transitions (= 6 × 23), which assumes
//  THREE Hall sensors. GPIO36/39 are removed from this design due
//  to ESP32 ECO 3.11 silicon bug. 138 is wrong for this hardware.
//
//  FIELD MEASUREMENT PROCEDURE (wizard Step 10):
//    1. 'hall reset'
//    2. Spin wheel exactly ONE full mechanical revolution.
//    3. 'hall status' → read Pulses field.
//    4. That count IS HALL_TRANSITIONS_PER_MECH_REV.
//    5. pole_pairs = measured_transitions / 2
//    6. Update both constants below and reflash.
// ============================================================
#define MOTOR_POLE_PAIRS              23   // *** VERIFY via wizard Step 10 ***
#define HALL_TRANSITIONS_PER_MECH_REV 46   // *** VERIFY — see note above ***
#define GEAR_REDUCTION                1.0f // Direct drive; update if gearbox fitted


// ============================================================
//  SECTION 2 — THROTTLE ADC CALIBRATION
//
//  All values are millivolts from esp_adc_cal.
//  Never compare against raw analogRead() counts.
//
//  HOW TO MEASURE:
//    'monitor throttle' in Serial Monitor.
//    Record mV at: zero-pedal (deadband), pedal-just-active,
//    and full-press. Update the three values below.
// ============================================================
#define THROTTLE_DEADBAND_MV         600   // Zero-pedal reads ~551mV; add margin
#define THROTTLE_MIN_INPUT_MV        650   // Pedal-just-active threshold
#define THROTTLE_MAX_INPUT_MV       3000   // Full-press mV — verify with 'monitor throttle'


// ============================================================
//  SECTION 3 — THROTTLE MODULE TUNING
//
//  THROTTLE_EMA_ALPHA:
//    Controls how aggressively the throttle ADC reading is
//    smoothed. Range 0.0–1.0.
//    Lower = smoother but slower to respond (more lag).
//    Higher = faster response but more ADC noise passes through.
//    0.15 is a typical starting point for a foot pedal.
//
//  SPEED_KP / SPEED_KI:
//    PI gains for the RPM → DAC control loop.
//    Output maps to 0–255 (8-bit DAC → LM358 → motor controller).
//
//  TUNING SEQUENCE (bench, no load first):
//    1. KI = 0. Raise KP until motor tracks step RPM changes
//       without large overshoot.
//    2. Add KI to eliminate steady-state RPM offset.
//       Keep KI small — integrator windup causes runaway.
//    3. Re-verify at full target RPM (COMM_SPEED_MAX = 3000).
// ============================================================
#define THROTTLE_EMA_ALPHA           0.15f  // ADC smoothing — see note above

#define SPEED_KP                     0.20f  // Starting point — retune on bench
#define SPEED_KI                     0.0375f


// ============================================================
//  SECTION 4 — STEERING POT ADC CALIBRATION
//
//  3590S pot powered at 3.3V. Values are calibrated ADC mV.
//  All three must be measured on car with wizard Steps 5–7.
//
//  HOW TO MEASURE:
//    Turn wheel to full LEFT  → 'set full_l' → note mV printed.
//    Turn wheel to full RIGHT → 'set full_r' → note mV printed.
//    Center steering          → 'set center' → note mV printed.
//    Update below and reflash.
// ============================================================
#define STEER_POT_MIN_MV            200    // PLACEHOLDER — run wizard Step 5
#define STEER_POT_CENTER_MV        1666    // PLACEHOLDER — run wizard Step 7
#define STEER_POT_MAX_MV           2766    // PLACEHOLDER — run wizard Step 6


// ============================================================
//  SECTION 5 — STEERING MODULE TUNING
//
//  STEER_EMA_ALPHA:
//    EMA smoothing for the steering pot ADC read.
//    Lower = smoother but adds lag to steering response.
//    Higher = more responsive but more noise reaches the PID.
//    0.15 matches the throttle alpha — adjust independently
//    if steering feels laggy or jittery.
//
//  STEER_KP / STEER_KI / STEER_KD:
//    PID gains for the stepper steering loop.
//    Error is in COMM units (0–1000 across full steering travel).
//    Setpoint and input are both in COMM units so the deadzone
//    is in the same space.
//
//  TUNING SEQUENCE (car, after pot calibration is done):
//    1. KI = 0, KD = 0. Raise KP until wheel reaches target.
//    2. If oscillating: lower KP slightly.
//    3. Add KD to dampen overshoot. Raise until smooth.
//    4. Add small KI only if steady-state offset remains.
//       Keep near 0 — integrator windup causes continuous drift.
//
//  STEER_DEADZONE:
//    Minimum error in COMM units before PID issues a correction.
//    Increase if stepper hunts (oscillates around target).
//    Decrease for tighter tracking at cost of more motor activity.
// ============================================================
#define STEER_EMA_ALPHA              0.15f  // ADC smoothing — see note above
#define STEER_KP                     1.2f
#define STEER_KI                     0.05f
#define STEER_KD                     0.01f
#define STEER_DEADZONE               5      // COMM units — increase if hunting


// ============================================================
//  SECTION 6 — BRAKE ACTUATOR CALIBRATION
//
//  TB6612FNG driving a 12V linear actuator.
//  Both values require physical timing on the car.
//
//  HOW TO CALIBRATE BRAKE_RETRACT_MS (wizard Step 3):
//    1. 'actuator extend' until limit switch stops it.
//    2. 'set retract_ms 600' then 'actuator retract'.
//    3. Observe — did it fully retract?
//       NO  → increase by 50ms and retry.
//       YES → decrease by 50ms until barely fully retracts.
//    4. Add 50ms safety margin to that minimum.
//    5. Update BRAKE_RETRACT_MS below.
// ============================================================
#define BRAKE_PWM                    255   // 0–255 to TB6612; 255 = full 12V (bench confirmed)
#define BRAKE_RETRACT_MS             2400   // PLACEHOLDER — calibrate via wizard Step 3


#endif // VCS_CALIBRATION_H