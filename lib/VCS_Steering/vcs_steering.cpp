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

    ledcSetup(0, STEPPER_MAX_HZ, 8);
    ledcAttachPin(PIN_STEER_PUL, 0);

    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);

    // Boot safe: stepper disabled
    // 2N2222 inverter: HIGH = transistor ON = opto fires = motor DISABLED
    digitalWrite(PIN_STEER_ENA, HIGH);
    ledcWrite(0, 0);

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
//   Low  mV (~142)  = physical RIGHT → COMM 1000
//   High mV (~3145) = physical LEFT  → COMM 0
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
    uint32_t pot_lo = min(pot_raw_min, pot_raw_max);   // 142  (physical right, low mV)
    uint32_t pot_hi = max(pot_raw_min, pot_raw_max);   // 3145 (physical left,  high mV)

    pot_mv = constrain(pot_mv, pot_lo, pot_hi);

    smoothedSteering = (STEER_EMA_ALPHA * (float)pot_mv)
                     + ((1.0f - STEER_EMA_ALPHA) * smoothedSteering);

    // Map mV → COMM.
    // Low mV (right end)  → COMM_STEER_RIGHT (1000)
    // High mV (left end)  → COMM_STEER_LEFT  (0)
    int mapped_pos = map((long)smoothedSteering,
                         (long)pot_lo, (long)pot_hi,
                         (long)COMM_STEER_RIGHT,   // low  mV = right = COMM 1000
                         (long)COMM_STEER_LEFT);    // high mV = left  = COMM 0

    // BUG 1 FIX: assign mapped_pos to current_pos (was missing — caused garbage return)
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
// ─────────────────────────────────────────────────────────────
static float commToMv(uint16_t comm) {
    // map(COMM, COMM_RIGHT, COMM_LEFT, pot_lo, pot_hi)
    // i.e., map(comm, 1000, 0, 142, 3145)
    return (float)map((long)comm,
                      (long)COMM_STEER_RIGHT, (long)COMM_STEER_LEFT,
                      (long)STEER_POT_MIN_MV, (long)STEER_POT_MAX_MV);
}

// ─────────────────────────────────────────────────────────────
void updateSteeringPID(uint16_t target_position, bool is_automatic) {

    static uint32_t s_dbg_ms = 0;
    bool dbg = (millis() - s_dbg_ms > 800);
    if (dbg) s_dbg_ms = millis();

    uint32_t raw = getSteeringRawMv();

    // ── Disconnection guard ───────────────────────────────────
    if (raw < 50 || raw > 3250) {
        ledcWrite(0, 0);
        digitalWrite(PIN_STEER_ENA, HIGH);
        if (dbg) Serial.println("[STEER] pot disconnected");
        return;
    }

    // ── Hard physical limits — stop BEFORE mechanical end stops ──
    // Physical RIGHT = low  mV (~142) = STEER_POT_MIN_MV
    // Physical LEFT  = high mV (~3145) = STEER_POT_MAX_MV
    const int32_t MARGIN_MV    = 80;
    bool at_right_physical = ((int32_t)raw <= (int32_t)STEER_POT_MIN_MV + MARGIN_MV);
    bool at_left_physical  = ((int32_t)raw >= (int32_t)STEER_POT_MAX_MV - MARGIN_MV);

    // BUG 3 FIX: direction check in mV space (not COMM) to avoid
    // the label inversion that existed when using COMM comparisons.
    // going_right = want to decrease mV (toward right end)
    // going_left  = want to increase mV (toward left  end)
    float target_mv_raw = commToMv(target_position);
    bool going_right = (target_mv_raw < (float)raw);
    bool going_left  = (target_mv_raw > (float)raw);

    if ((at_right_physical && going_right) ||
        (at_left_physical  && going_left)) {
        ledcWrite(0, 0);
        digitalWrite(PIN_STEER_ENA, LOW);   // hold (2N2222: LOW=enabled)
        if (dbg) Serial.printf("[STEER] HARD LIMIT raw=%lu going=%s\n",
                               (unsigned long)raw,
                               going_right ? "RIGHT" : "LEFT");
        return;
    }

    // ── Debug steer control override ─────────────────────────
    // BUG 2 FIX: use debug_tgt (not target_position) after mapping.
    if (isDebugSteerControl()) {
        uint32_t pot_lo = min(dbg_pot_min_mv, dbg_pot_max_mv);
        uint32_t pot_hi = max(dbg_pot_min_mv, dbg_pot_max_mv);
        if (pot_lo < pot_hi) {
            int debug_tgt = map((long)dbg_steer_target_mv,
                                (long)pot_hi, (long)pot_lo,
                                (long)COMM_STEER_LEFT, (long)COMM_STEER_RIGHT);
            // BUG 2 FIX was here: constrain debug_tgt, not target_position
            target_position = (uint16_t)constrain(debug_tgt,
                                                   STEER_COMM_LEFT_SAFE,
                                                   STEER_COMM_RIGHT_SAFE);
        }
        is_automatic = true;
    }

    // ── Manual mode — shaft free ──────────────────────────────
    if (!is_automatic) {
        if (dbg) Serial.println("[STEER] MANUAL -> ENA=HIGH (shaft free)");
        digitalWrite(PIN_STEER_ENA, HIGH);
        ledcWrite(0, 0);
        s_last_freq_hz = -1;
        return;
    }

    // ── Clamp target to safe physical travel range ────────────
    target_position = (uint16_t)constrain((int)target_position,
                                           STEER_COMM_LEFT_SAFE,
                                           STEER_COMM_RIGHT_SAFE);

    // ── mV-based control ─────────────────────────────────────
    // Work in mV space so geared stepping (incremental, not continuous)
    // is handled correctly. ±STEER_MV_DEADBAND tolerance avoids
    // hunting between gear teeth.
    float target_mv = commToMv(target_position);
    float mv_error  = target_mv - (float)raw;   // positive = need more mV = go LEFT
                                                  // negative = need less mV = go RIGHT

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
        ledcWrite(0, 0);
        digitalWrite(PIN_STEER_ENA, LOW);   // hold (2N2222: LOW=enabled)
        s_last_freq_hz = -1;
        return;
    }

    // ── Enable motor ──────────────────────────────────────────
    digitalWrite(PIN_STEER_ENA, LOW);   // 2N2222: LOW=transistor OFF=opto off=enabled

    // ── Direction ─────────────────────────────────────────────
    // mv_error > 0 → need more mV → go LEFT  (high mV = left)
    // mv_error < 0 → need less mV → go RIGHT (low  mV = right)
    //
    // Physical direction for DIR pin (verified on hardware):
    //   DIR=HIGH → motor goes LEFT  (increases mV)
    //   DIR=LOW  → motor goes RIGHT (decreases mV)
    //
    // If this is inverted on your car, set STEER_DIR_FLIP true
    // in vcs_calibration.h.
    bool go_left = (mv_error > 0);
    bool new_dir = go_left ^ STEER_DIR_FLIP;   // flip if wired opposite

    if (new_dir != s_last_dir) {
        ledcWrite(0, 0);
        delayMicroseconds(STEER_DM542_DIR_SETUP_US);
        s_last_dir = new_dir;
    }
    digitalWrite(PIN_STEER_DIR, new_dir ? HIGH : LOW);

    // ── Proportional speed — slow down as error shrinks ───────
    // Full speed at 1500mV error, minimum speed at deadband edge.
    float effort = constrain(fabsf(mv_error) / 1500.0f, 0.0f, 1.0f);
    int step_hz  = (int)(STEER_MIN_HZ + effort * (float)(STEPPER_MAX_HZ - STEER_MIN_HZ));

    // Only reconfigure LEDC if frequency changed enough (each
    // ledcSetup() causes one spurious pulse)
    if (abs(step_hz - s_last_freq_hz) >= STEER_FREQ_UPDATE_HZ) {
        ledcSetup(0, step_hz, 8);
        ledcAttachPin(PIN_STEER_PUL, 0);
        s_last_freq_hz = step_hz;
    }
    ledcWrite(0, 128);   // 50% duty

#if SIMULATION_MODE
    updateSimulatedPhysics(step_hz, new_dir);
    const float delta = (float)step_hz * (1.0f / FREQ_STEER_CTRL_HZ) * STEPS_TO_COMM_SCALE;
    s_sim_steer_pos_f += new_dir ? +delta : -delta;
    s_sim_steer_pos_f  = constrain(s_sim_steer_pos_f,
                                    (float)COMM_STEER_LEFT, (float)COMM_STEER_RIGHT);
    sim_steer_pos = (uint16_t)s_sim_steer_pos_f;
#endif
}