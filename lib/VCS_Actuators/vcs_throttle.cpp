// ============================================================
//  vcs_throttle.cpp — SIDLAK 2 VCS
//  Team Wired PH0017003 | Shell Eco-marathon 2026
//
//  AUTONOMOUS: open-loop DAC from lookup table (no PID, no hall sensor)
//  MANUAL:     pedal ADC pass-through
//
//  Calibration data (tachometer, no-load):
//    throttle 29% (DAC  74) →  12.5 RPM
//    throttle 40% (DAC 102) → 173   RPM  ← reliable minimum
//    throttle 43% (DAC 110) → 208   RPM
//    throttle 44% (DAC 112) → 219   RPM
//    throttle 45% (DAC 115) → 229   RPM
//    throttle 60% (DAC 153) → 417   RPM
//    throttle 70% (DAC 178) → 537   RPM  ← motor cap
//    throttle 70%+          → 537   RPM  (capped)
//
//  Competition (15–20 km/h):
//    15 km/h → 196 RPM → DAC 107 (42%)
//    17.5 km/h → 229 RPM → DAC 115 (45%)
//    20 km/h → 261 RPM → DAC 121 (47.5%)
// ============================================================

#include "vcs_throttle.h"
#include "vcs_state_machine.h"
#include "vcs_constants.h"
#include "vcs_pins.h"
#include "vcs_calibration.h"
#include "vcs_debug.h"
#include "esp_adc_cal.h"
#include "driver/adc.h"

esp_adc_cal_characteristics_t adc_chars;

uint16_t current_throttle_adc = 0;
uint16_t current_pwm_duty     = 0;

static uint8_t s_last_dac         = 0;
static float   s_estimated_rpm    = 0.0f;

// ─────────────────────────────────────────────────────────────
//  Lookup table — measured tachometer data (no-load)
//  Sorted ascending by RPM.  Linear interpolation between points.
// ─────────────────────────────────────────────────────────────
struct ThrottlePoint { float rpm; uint8_t dac; };

static const ThrottlePoint OL_TABLE[] = {
    {   0.0f,   0 },   // stopped
    {  12.5f,  74 },   // 29% — approximate, motor barely turning
    { 173.0f, 102 },   // 40% — reliable minimum start
    { 208.0f, 110 },   // 43%
    { 219.0f, 112 },   // 44%
    { 229.0f, 115 },   // 45%
    { 417.0f, 153 },   // 60%
    { 537.0f, 178 },   // 70% — motor cap
};
static constexpr int OL_TABLE_LEN = (int)(sizeof(OL_TABLE) / sizeof(OL_TABLE[0]));

// ── Forward: RPM → DAC ───────────────────────────────────────
static uint8_t rpmToDac(float rpm) {
    if (rpm <= 0.0f) return 0;
    if (rpm >= OL_TABLE[OL_TABLE_LEN-1].rpm) return OL_TABLE[OL_TABLE_LEN-1].dac;

    for (int i = 0; i < OL_TABLE_LEN - 1; i++) {
        float r0 = OL_TABLE[i].rpm,  r1 = OL_TABLE[i+1].rpm;
        float d0 = OL_TABLE[i].dac,  d1 = OL_TABLE[i+1].dac;
        if (rpm >= r0 && rpm <= r1) {
            float t   = (rpm - r0) / (r1 - r0);
            uint8_t dac = (uint8_t)constrain((int)(d0 + t*(d1-d0) + 0.5f), 0, 255);
            return max(dac, (uint8_t)THROTTLE_OL_START_DAC);   // ← ADD: floor at throttle 29% for any non-zero request
        }
    }
    return 74;   // fallback
}

// ── Reverse: DAC → estimated RPM (for reporting back to Jetson) ──
static float dacToRpm(uint8_t dac) {
    if (dac <= 0)                         return 0.0f;
    if (dac >= OL_TABLE[OL_TABLE_LEN-1].dac) return OL_TABLE[OL_TABLE_LEN-1].rpm;
    for (int i = 0; i < OL_TABLE_LEN - 1; i++) {
        float d0 = OL_TABLE[i].dac,   d1 = OL_TABLE[i+1].dac;
        float r0 = OL_TABLE[i].rpm,   r1 = OL_TABLE[i+1].rpm;
        if ((float)dac >= d0 && (float)dac <= d1) {
            if (d1 <= d0) return r0;
            float t = ((float)dac - d0) / (d1 - d0);
            return r0 + t * (r1 - r0);
        }
    }
    return 0.0f;
}

// ── Safe DAC write ────────────────────────────────────────────
static void writeDac(uint8_t val) {
    val = constrain(val, 0, (uint8_t)THROTTLE_MAX_DAC);
    dacWrite(PIN_THROTTLE_OUT, val);
    s_last_dac      = val;
    s_estimated_rpm = dacToRpm(val);
    current_pwm_duty = (uint16_t)map(val, 0, 255, 0, 1023);
}

// ─────────────────────────────────────────────────────────────
void initThrottle() {
    pinMode(PIN_THROTTLE_OUT, OUTPUT);
    pinMode(PIN_THROTTLE_IN,  INPUT);

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11); // steering pot shares chars
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11,
                             ADC_WIDTH_BIT_12, 1100, &adc_chars);
}

// ─────────────────────────────────────────────────────────────
void updateThrottle(float /*unused*/, float target_rpm_in) {

    // ── Read pedal ADC (manual + debug) ──────────────────────
    uint32_t sum = 0;
    for (int i = 0; i < 16; i++) sum += adc1_get_raw(ADC1_CHANNEL_6);
    uint32_t pedal_mv = esp_adc_cal_raw_to_voltage(sum / 16, &adc_chars);
    current_throttle_adc = (uint16_t)map(pedal_mv, 0, 3300, 0, 1023);

    // ── Safety lockout ────────────────────────────────────────
    bool brakePressed = (digitalRead(PIN_LOWBRAKE_IN) == LOW);
    bool validState   = (currentState == AUTONOMOUS_STATE ||
                         currentState == MANUAL_STATE);

    if (!validState || brakePressed) {
        writeDac(0);
        return;
    }

    // ── 1. AUTONOMOUS — open-loop lookup ─────────────────────
    if (currentState == AUTONOMOUS_STATE) {
        float target = (dbg_throttle_override_dac >= 0)
            ? dacToRpm((uint8_t)constrain(dbg_throttle_override_dac, 0, 255))
            : target_rpm_in;

        writeDac(rpmToDac(target));
        return;
    }

    // ── 2. MANUAL — pedal pass-through ───────────────────────
    if (currentState == MANUAL_STATE) {
        if (dbg_throttle_override_dac >= 0) {
            writeDac((uint8_t)constrain(dbg_throttle_override_dac, 0, 255));
            return;
        }

        uint8_t dac = 0;
        if (current_throttle_adc > THROTTLE_MIN_INPUT) {
            // Map pedal range to motor start→cap (DAC 74 to THROTTLE_MAX_DAC)
            // Below THROTTLE_MIN_INPUT: output 0 (deadband, motor won't run)
            int mapped = map(current_throttle_adc,
                             THROTTLE_MIN_INPUT,  THROTTLE_MAX_INPUT,
                             THROTTLE_OL_MIN_DAC, THROTTLE_MAX_DAC);
            dac = (uint8_t)constrain(mapped, 0, THROTTLE_MAX_DAC);
        }
        writeDac(dac);
        return;
    }
}

// ── Getters ──────────────────────────────────────────────────
uint8_t  getThrottleDacOut()       { return s_last_dac; }
float    getThrottleEstimatedRPM() { return s_estimated_rpm; }
bool     isThrottlePedalPressed()  { return current_throttle_adc > THROTTLE_MIN_INPUT; }

uint32_t getThrottlePedalMv() {
    uint32_t sum = 0;
    for (int i = 0; i < 16; i++) sum += adc1_get_raw(ADC1_CHANNEL_6);
    return esp_adc_cal_raw_to_voltage(sum / 16, &adc_chars);
}