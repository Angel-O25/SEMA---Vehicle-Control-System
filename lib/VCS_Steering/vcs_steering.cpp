// ============================================================
//  vcs_steering.cpp — SIDLAK 2 VCS
//  Team Wired PH0017003 | Shell Eco-marathon 2026
//
//  Changes from previous revision:
//  • Added getSteeringRawMv() — raw unfiltered pot mV for debug
//  • Added debug steer control: when isDebugSteerControl() is
//    true, steer target comes from dbg_steer_target_mv (mV)
//    instead of the Jetson UART command
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

// FIX 9: file-scope statics to lock down PID I/O variables
static float setpoint = 0.0f;
static float input    = 0.0f;
static float output   = 0.0f;

QuickPID steeringPID(&input, &output, &setpoint);

const float Ts_s = 1.0f / FREQ_STEER_CTRL_HZ;

// ── Raw mV getter (no filter, used by debug display + calibration) ──
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

    digitalWrite(PIN_STEER_ENA, HIGH);   // boot safe: disabled (2N2222: HIGH=transistor ON=opto fires=disabled)
    ledcWrite(0, 0);

    steeringPID.SetTunings(STEER_KP, STEER_KI, STEER_KD);
    steeringPID.SetSampleTimeUs((uint32_t)(Ts_s * 1000000.0f));
    steeringPID.SetOutputLimits(-255.0f, 255.0f);
    steeringPID.SetMode(QuickPID::Control::automatic);
}

// ─────────────────────────────────────────────────────────────
uint16_t getMeasuredSteering() {
    uint16_t current_pos;

#if SIMULATION_MODE
    current_pos = (uint16_t)constrain(sim_steer_pos, COMM_STEER_LEFT, COMM_STEER_RIGHT);
#else
    uint32_t pot_mv = getSteeringRawMv();

    // Disconnection check
    if (pot_mv < 50 || pot_mv > 3200) {
        smoothedSteering = (float)STEER_POT_CENTER_MV;
        return COMM_STEER_CENTER;
    }

    // Clamp to calibrated range.
    // NOTE: STEER_POT_MIN_MV (3145) is numerically LARGER than STEER_POT_MAX_MV (142)
    // because the pot is inverted (high mV = full-left, low mV = full-right).
    // constrain() requires lo < hi, so always pass the numerically smaller value first.
    uint32_t pot_raw_min = isDebugMode() ? dbg_pot_min_mv : (uint32_t)STEER_POT_MIN_MV;
    uint32_t pot_raw_max = isDebugMode() ? dbg_pot_max_mv : (uint32_t)STEER_POT_MAX_MV;
    uint32_t pot_lo = min(pot_raw_min, pot_raw_max);  // numerically smaller  (142)
    uint32_t pot_hi = max(pot_raw_min, pot_raw_max);  // numerically larger  (3145)

    pot_mv = constrain(pot_mv, pot_lo, pot_hi);

    smoothedSteering = (STEER_EMA_ALPHA * (float)pot_mv)
                     + ((1.0f - STEER_EMA_ALPHA) * smoothedSteering);

    // Swap map endpoints so that high mV (full-left) → COMM_STEER_LEFT (0)
    // and low mV (full-right) → COMM_STEER_RIGHT (1000).
    int mapped_pos = map((long)smoothedSteering,
                 (long)pot_lo, (long)pot_hi,
                 (long)COMM_STEER_RIGHT, (long)COMM_STEER_LEFT);
#endif

    // Slew rate limit
    static uint16_t last_pos = COMM_STEER_CENTER;
    int32_t delta = (int32_t)current_pos - (int32_t)last_pos;
    if      (delta >  50) current_pos = last_pos + 50;
    else if (delta < -50) current_pos = last_pos - 50;
    last_pos = current_pos;
    return current_pos;
}

// ─────────────────────────────────────────────────────────────
void updateSteeringPID(uint16_t target_position, bool is_automatic) {
    static uint32_t s_dbg_ms = 0;
    bool dbg = (millis() - s_dbg_ms > 800);
    if (dbg) s_dbg_ms = millis();
    
        uint32_t raw = getSteeringRawMv();

    // ── Hard limits: stop motor before physical end of pot ────
    const int32_t MARGIN_MV   = 80;
    bool at_left_end  = ((int32_t)raw <= (int32_t)STEER_POT_MIN_MV + MARGIN_MV);
    bool at_right_end = ((int32_t)raw >= (int32_t)STEER_POT_MAX_MV - MARGIN_MV);

    // Disconnected check
    if (raw < 50 || raw > 3250) {
        ledcWrite(0, 0); digitalWrite(PIN_STEER_ENA, HIGH); return;
    }

    // At limit: only allow motion in the escaping direction
    if (at_left_end || at_right_end) {
        uint16_t comm = getMeasuredSteering();
        bool trying_to_go_left  = (target_position < comm);
        bool trying_to_go_right = (target_position > comm);

        if ((at_left_end  && trying_to_go_left) ||
            (at_right_end && trying_to_go_right)) {
            ledcWrite(0, 0);   // block motion toward the end
            digitalWrite(PIN_STEER_ENA, LOW);   // hold position (2N2222: LOW=enabled=holds)
            return;
        }
    }

    // ── Debug steer control override ──────────────────────────
    // When in DBG_STEER_CTRL: enable motor, use dbg_steer_target_mv
    // as the target (mapped to 0-1000 using debug calibration values)
    if (isDebugSteerControl()) {
        uint32_t pot_lo = min(dbg_pot_min_mv, dbg_pot_max_mv);
        uint32_t pot_hi = max(dbg_pot_min_mv, dbg_pot_max_mv);
        if (pot_lo < pot_hi) {
            // Same endpoint swap as getMeasuredSteering()
            int debug_tgt = map((long)dbg_steer_target_mv,
                    (long)pot_lo, (long)pot_hi,   // was (pot_hi, pot_lo)
                    COMM_STEER_LEFT, COMM_STEER_RIGHT);
            target_position = (uint16_t)constrain((int)target_position,
                                           STEER_COMM_LEFT_SAFE,
                                           STEER_COMM_RIGHT_SAFE);
        }
        is_automatic = true;   // force PID on
    }

    input    = (float)getMeasuredSteering();
    setpoint = (float)target_position;

    // ── Manual mode / debug steer_read: motor free ────────────
    if (!is_automatic) {
        if (dbg) Serial.println("[STEER_DBG] MANUAL mode -> ENA=HIGH (shaft free)");
        digitalWrite(PIN_STEER_ENA, HIGH);  // 2N2222: HIGH=transistor ON=disabled=shaft free
        ledcWrite(0, 0);
        s_last_freq_hz = -1;
        if (steeringPID.GetMode() != (uint8_t)QuickPID::Control::manual) {
            steeringPID.SetMode(QuickPID::Control::manual);
            steeringPID.Initialize();
        }
        return;
    }

    steeringPID.Compute();

    if (steeringPID.GetMode() == (uint8_t)QuickPID::Control::manual) {
        steeringPID.SetMode(QuickPID::Control::automatic);
    }

    // Deadband
    if (fabsf(setpoint - input) < STEER_DEADZONE) {
        if (dbg) Serial.printf("[STEER_DBG] DEADBAND: |%.1f-%.1f|=%.1f < zone=%.1f -> holding\n",setpoint,input,fabsf(setpoint-input),(float)STEER_DEADZONE);
        ledcWrite(0, 0);
        digitalWrite(PIN_STEER_ENA, LOW);   // hold position (2N2222: LOW=enabled)
        s_last_freq_hz = -1;
        return;
    }

    digitalWrite(PIN_STEER_ENA, LOW);   // enabled (2N2222: LOW=transistor OFF=opto off=enabled)

    bool new_dir;
    if (fabsf(output) < 5.0f) {
        new_dir = s_last_dir;
    } else {
        new_dir = (output > 0);
    }
    digitalWrite(PIN_STEER_DIR, new_dir ? LOW : HIGH);

    float effort = fabsf(output);
    if (effort < 1.0f) effort = 1.0f;
    int step_hz = (int)map((long)effort, 0, 255, 50, STEPPER_MAX_HZ);

#if SIMULATION_MODE
    updateSimulatedPhysics(step_hz, new_dir);
    const float delta = (float)step_hz * Ts_s * STEPS_TO_COMM_SCALE;
    s_sim_steer_pos_f += new_dir ? +delta : -delta;
    s_sim_steer_pos_f  = constrain(s_sim_steer_pos_f,
                                    (float)COMM_STEER_LEFT, (float)COMM_STEER_RIGHT);
    sim_steer_pos = (uint16_t)s_sim_steer_pos_f;
#else
    // Fixed frequency — direction and run/stop only. No ledcChangeFrequency() in loop.
    if (new_dir != s_last_dir) {
        ledcWrite(0, 0);                            // stop before direction change
        delayMicroseconds(10);
        digitalWrite(PIN_STEER_DIR, new_dir ? LOW : HIGH);
        s_last_dir = new_dir;
    }
    ledcWrite(0, 128);                              // run at fixed freq
    s_last_freq_hz = STEPPER_MAX_HZ;               // mark as "set once"
#endif
}