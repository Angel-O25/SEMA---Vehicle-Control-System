// ============================================================
//  vcs_steering.cpp — SIDLAK 2 VCS
//  Team Wired PH0017003 | Shell Eco-marathon 2026
//
//  Fixes in this revision:
//  Bug 1: mapped_pos was computed but never assigned to
//         current_pos → getMeasuredSteering() returned garbage,
//         PID always saw input=0, motor ran in one direction forever
//  Bug 2: debug target computed as debug_tgt but constrain()
//         was applied to target_position — debug mode ignored its
//         own commanded target
//  Bug 3: at_left_end / at_right_end labels were inverted after
//         the pot mapping swap — hard limits blocked correct motion
//  New:   mV-based deadband (±STEER_MV_DEADBAND) instead of COMM
//         units — more reliable for geared steering where position
//         advances in discrete tooth steps, not continuously
//  New:   Proportional speed in mV space — motor slows as it
//         approaches target instead of running at fixed max speed
//
//  2026-06 fixes:
//  Fix 1: All ledcWrite(0,0) stop calls changed to ledcWrite(0,255)
//         so PUL- idles HIGH (opto OFF) instead of LOW (opto ON).
//         With common-anode wiring, PUL- LOW = opto permanently ON
//         = driver counts spurious steps and wastes power.
//         Exception: direction-change pause at line ~255 stays as
//         ledcWrite(0,0) — it is momentary (5µs) before 128 fires.
//  Fix 2: commToMv() now respects debug calibration overrides
//         (dbg_pot_min_mv / dbg_pot_max_mv) when in debug mode.
//         Previously always used compile-time constants, so
//         steer_mv commands during calibration used wrong mapping.
//  Fix 3: Effort denominator reduced 1500→400 to match actual
//         pot range (~720mV total). With 1500mV denominator the
//         motor ran at minimum speed (80Hz) for the entire travel.
// ============================================================

#include "vcs_steering.h"
#include "vcs_debug.h"
#include "esp_adc_cal.h"
#include "driver/adc.h"
#include "vcs_calibration.h"
#include "vcs_constants.h"

extern esp_adc_cal_characteristics_t adc_chars;

float smoothedSteering = 0.0f;

static int  s_last_freq_hz = -1;
static bool s_last_dir     = false;

#if SIMULATION_MODE
static constexpr float STEPS_TO_COMM_SCALE = 0.05f;
static float    s_sim_steer_pos_f = (float)COMM_STEER_CENTER;
static uint16_t sim_steer_pos     = COMM_STEER_CENTER;
#endif

// ── Raw mV getter (no filter — used by debug display + calibration) ──
uint32_t getSteeringRawMv() {
    uint32_t sum = 0;
    for (int i = 0; i < 16; i++) sum += adc1_get_raw(ADC1_CHANNEL_7);
    return esp_adc_cal_raw_to_voltage(sum / 16, &adc_chars);
}

// ─────────────────────────────────────────────────────────────
void initSteering() {
    pinMode(PIN_STEER_POT, INPUT);
    pinMode(PIN_STEER_DIR, OUTPUT);
    pinMode(PIN_STEER_ENA, OUTPUT);

    // Configure LEDC channel 0 for stepper pulses.
    // ledcSetup() must be called before ledcAttachPin() — order matters.
    // ledcAttachPin() connects the hardware timer output to GPIO 18.
    // After this pair, GPIO 18 is driven by the LEDC peripheral.
    ledcSetup(0, STEPPER_MAX_HZ, 8);
    ledcAttachPin(PIN_STEER_PUL, 0);

    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);

    // Boot safe: stepper disabled
    // 2N2222 inverter: HIGH = transistor ON = opto fires = motor DISABLED
    digitalWrite(PIN_STEER_ENA, HIGH);
    // Idle HIGH: PUL- stays HIGH = opto OFF = no spurious steps at boot
    ledcWrite(0, 255);

    // Seed smoothed value from actual pot to avoid startup lurch
    uint32_t boot_mv = getSteeringRawMv();
    if (boot_mv >= 50 && boot_mv <= 3200)
        smoothedSteering = (float)boot_mv;
    else
        smoothedSteering = STEER_POT_CENTER_MV;
}

// ─────────────────────────────────────────────────────────────
// getMeasuredSteering — returns current position as COMM (0-1000).
// Used by OLED display, odometry, and debug screens.
// Physical mapping (confirmed on hardware):
//   Low  mV (~1240) = physical RIGHT → COMM 1000
//   High mV (~1960) = physical LEFT  → COMM 0
// ─────────────────────────────────────────────────────────────
uint16_t getMeasuredSteering() {
    uint16_t current_pos;

#if SIMULATION_MODE
    current_pos = (uint16_t)constrain(sim_steer_pos, COMM_STEER_LEFT, COMM_STEER_RIGHT);
#else
    uint32_t pot_mv = getSteeringRawMv();

    // Disconnection / wiring fault check
    if (pot_mv < 50 || pot_mv > 3200) {
        smoothedSteering = STEER_POT_CENTER_MV;
        return COMM_STEER_CENTER;
    }

    // Calibration range — use debug overrides if in debug mode
    uint32_t pot_raw_min = isDebugMode() ? dbg_pot_min_mv : (uint32_t)STEER_POT_MIN_MV;
    uint32_t pot_raw_max = isDebugMode() ? dbg_pot_max_mv : (uint32_t)STEER_POT_MAX_MV;
    uint32_t pot_lo = min(pot_raw_min, pot_raw_max);
    uint32_t pot_hi = max(pot_raw_min, pot_raw_max);

    pot_mv = constrain(pot_mv, pot_lo, pot_hi);

    smoothedSteering = (STEER_EMA_ALPHA * (float)pot_mv)
                     + ((1.0f - STEER_EMA_ALPHA) * smoothedSteering);

    // Map mV → COMM.
    // Low mV (right end)  → COMM_STEER_RIGHT (1000)
    // High mV (left end)  → COMM_STEER_LEFT  (0)
    int mapped_pos = map((long)smoothedSteering,
                         (long)pot_lo, (long)pot_hi,
                         (long)COMM_STEER_RIGHT,
                         (long)COMM_STEER_LEFT);

    current_pos = (uint16_t)constrain(mapped_pos, COMM_STEER_LEFT, COMM_STEER_RIGHT);
#endif

    // Slew rate limit — prevents large jumps from noisy ADC reads
    static uint16_t last_pos = COMM_STEER_CENTER;
    int32_t delta = (int32_t)current_pos - (int32_t)last_pos;
    if      (delta >  50) current_pos = last_pos + 50;
    else if (delta < -50) current_pos = last_pos - 50;
    last_pos = current_pos;
    return current_pos;
}

// ─────────────────────────────────────────────────────────────
// commToMv — convert COMM (0-1000) back to pot mV.
// Used to work in mV space for deadband and direction decisions.
// Inverse of the map in getMeasuredSteering().
//
// FIX 2: now uses debug calibration overrides when in debug mode
// so steer_mv commands use the correct mV range during calibration.
// ─────────────────────────────────────────────────────────────
static float commToMv(uint16_t comm) {
    float pot_lo = isDebugMode()
        ? (float)min(dbg_pot_min_mv, dbg_pot_max_mv)
        : min(STEER_POT_MIN_MV, STEER_POT_MAX_MV);
    float pot_hi = isDebugMode()
        ? (float)max(dbg_pot_min_mv, dbg_pot_max_mv)
        : max(STEER_POT_MIN_MV, STEER_POT_MAX_MV);
    return (float)map((long)comm,
                      (long)COMM_STEER_RIGHT, (long)COMM_STEER_LEFT,
                      (long)pot_lo, (long)pot_hi);
}

// ─────────────────────────────────────────────────────────────
void updateSteeringPID(uint16_t target_position, bool is_automatic) {

    static uint32_t s_dbg_ms = 0;
    bool dbg = (millis() - s_dbg_ms > 800);
    if (dbg) s_dbg_ms = millis();

    uint32_t raw = getSteeringRawMv();

    // ── Disconnection guard ───────────────────────────────────
    if (raw < 50 || raw > 3250) {
        // FIX 1: idle HIGH (opto OFF) not LOW (opto ON)
        ledcWrite(0, 255);
        digitalWrite(PIN_STEER_ENA, HIGH);
        if (dbg) Serial.println("[STEER] pot disconnected");
        return;
    }

    // ── Debug steer control override ─────────────────────────
    if (isDebugSteerControl()) {
        uint32_t pot_lo = min(dbg_pot_min_mv, dbg_pot_max_mv);
        uint32_t pot_hi = max(dbg_pot_min_mv, dbg_pot_max_mv);
        // Fall back to compile-time constants if debug pot not calibrated
        if (pot_lo >= pot_hi) {
            pot_lo = (uint32_t)min(STEER_POT_MIN_MV, STEER_POT_MAX_MV);
            pot_hi = (uint32_t)max(STEER_POT_MIN_MV, STEER_POT_MAX_MV);
        }
        int debug_tgt = map((long)dbg_steer_target_mv,
                            (long)pot_hi, (long)pot_lo,
                            (long)COMM_STEER_LEFT, (long)COMM_STEER_RIGHT);
        target_position = (uint16_t)constrain(debug_tgt,
                                               STEER_COMM_LEFT_SAFE,
                                               STEER_COMM_RIGHT_SAFE);
        is_automatic = true;
    }

    // ── Manual mode — shaft free ──────────────────────────────
    if (!is_automatic) {
        if (dbg) Serial.println("[STEER] MANUAL -> ENA=HIGH (shaft free)");
        digitalWrite(PIN_STEER_ENA, HIGH);
        // FIX 1: idle HIGH (opto OFF) not LOW (opto ON)
        ledcWrite(0, 255);
        s_last_freq_hz = -1;
        return;
    }

    // ── Hard physical limits — only reached in autonomous mode ──
    // Use phy_lo/phy_hi so logic is correct regardless of MIN > MAX or MIN < MAX.
    // Your cal: MIN_MV=2000 (right end), MAX_MV=1200 (left end) → inverted range.
    // Old code: at_left = (raw >= MAX_MV - 80) = (raw >= 1120) → always true. Fixed.
    const int32_t MARGIN_MV = 80;
    const int32_t phy_lo = (int32_t)min(STEER_POT_MIN_MV, STEER_POT_MAX_MV);  // 1200
    const int32_t phy_hi = (int32_t)max(STEER_POT_MIN_MV, STEER_POT_MAX_MV);  // 2000
    bool at_lo_end = ((int32_t)raw <= phy_lo + MARGIN_MV);  // raw <= 1280
    bool at_hi_end = ((int32_t)raw >= phy_hi - MARGIN_MV);  // raw >= 1920
    float target_mv_raw = commToMv(target_position);
    bool heading_lo = (target_mv_raw < (float)raw);   // moving toward lower mV
    bool heading_hi = (target_mv_raw > (float)raw);   // moving toward higher mV
    if ((at_lo_end && heading_lo) || (at_hi_end && heading_hi)) {
        ledcWrite(STEER_LEDC_CH, 0);
        digitalWrite(PIN_STEER_ENA, LOW);
        if (dbg) Serial.printf("[STEER] HARD LIMIT raw=%lu end=%s\n",
                               (unsigned long)raw, at_lo_end ? "LO" : "HI");
        return;
    }

    // ── Clamp target to safe physical travel range ────────────
    target_position = (uint16_t)constrain((int)target_position,
                                           STEER_COMM_LEFT_SAFE,
                                           STEER_COMM_RIGHT_SAFE);

    // ── mV-based control ─────────────────────────────────────
    float target_mv = commToMv(target_position);
    float mv_error  = target_mv - (float)raw;

    if (dbg) {
        Serial.printf("[STEER] raw=%lumV tgt=%lumV err=%+.0fmV comm_tgt=%u comm_meas=%u\n",
                      (unsigned long)raw,
                      (unsigned long)(uint32_t)target_mv,
                      mv_error,
                      (unsigned)target_position,
                      (unsigned)getMeasuredSteering());
    }

    // ── Deadband — hold if within ±STEER_MV_DEADBAND ─────────
    if (fabsf(mv_error) < STEER_MV_DEADBAND) {
        if (dbg) Serial.printf("[STEER] DEADBAND %.0fmV < %.0fmV -> hold\n",
                               fabsf(mv_error), (float)STEER_MV_DEADBAND);
        // FIX 1: idle HIGH (opto OFF) not LOW (opto ON)
        ledcWrite(0, 255);
        digitalWrite(PIN_STEER_ENA, LOW);   // hold position (2N2222: LOW=enabled)
        s_last_freq_hz = -1;
        return;
    }

    // ── Enable motor ──────────────────────────────────────────
    digitalWrite(PIN_STEER_ENA, LOW);   // 2N2222: LOW=transistor OFF=opto off=enabled

    // ── Direction ─────────────────────────────────────────────
    // mv_error > 0 → need more mV → go LEFT  (high mV = left)
    // mv_error < 0 → need less mV → go RIGHT (low  mV = right)
    bool go_left = (mv_error > 0);
    bool new_dir = go_left ^ STEER_DIR_FLIP;

    if (new_dir != s_last_dir) {
        // Momentary pulse suppression during direction change only.
        // This stays as 0 — it lasts only STEER_DM542_DIR_SETUP_US (5µs)
        // before ledcWrite(0,128) fires immediately after.
        ledcWrite(0, 0);
        delayMicroseconds(STEER_DM542_DIR_SETUP_US);
        s_last_dir = new_dir;
    }
    digitalWrite(PIN_STEER_DIR, new_dir ? HIGH : LOW);

    // ── Proportional speed — slow down as error shrinks ───────
    // FIX 3: denominator reduced 1500→400 to match actual pot range
    // (~720mV total). With 1500mV denominator the motor ran at
    // minimum speed (80Hz) for the entire steering travel.
    // 400mV = just over half the full range = full speed at half deflection.
    // Effort denominator 200mV — motor reaches full speed only when error
    // exceeds 200mV. For short moves (e.g. 100mV) effort=0.5 → ~760Hz,
    // much slower than before (was 435Hz at 400 denom, now half speed).
    // Reduces overshoot on small corrections without affecting large moves.
    float effort = constrain(fabsf(mv_error) / 200.0f, 0.0f, 1.0f);
    int step_hz  = (int)(STEER_MIN_HZ + effort * (float)(STEPPER_MAX_HZ - STEER_MIN_HZ));

    // Update LEDC frequency without detaching the pin.
    // ledcSetup() + ledcAttachPin() temporarily floats GPIO 18 during
    // reconfiguration — causes the 1.85V floating measurement and lost
    // pulses. ledcChangeFrequency() (Arduino ESP32 v2.0+) updates the
    // timer in-place without touching the pin mux attachment.
    if (abs(step_hz - s_last_freq_hz) >= STEER_FREQ_UPDATE_HZ) {
        ledcChangeFrequency(0, step_hz, 8);
        s_last_freq_hz = step_hz;
    }
    ledcWrite(0, 128);   // 50% duty — run

#if SIMULATION_MODE
    updateSimulatedPhysics(step_hz, new_dir);
    const float delta = (float)step_hz * (1.0f / FREQ_STEER_CTRL_HZ) * STEPS_TO_COMM_SCALE;
    s_sim_steer_pos_f += new_dir ? +delta : -delta;
    s_sim_steer_pos_f  = constrain(s_sim_steer_pos_f,
                                    (float)COMM_STEER_LEFT, (float)COMM_STEER_RIGHT);
    sim_steer_pos = (uint16_t)s_sim_steer_pos_f;
#endif
}