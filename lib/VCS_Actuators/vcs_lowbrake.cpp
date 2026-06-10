#include "vcs_lowbrake.h"

// =========================================================
// Brake subsystem — SIDLAK 2 VCS
// Team Wired PH0017003 | Shell Eco-marathon 2026
//
// TB6612FNG motor controller modes used:
//   EXTEND  — IN1=HIGH IN2=LOW  PWM=BRAKE_PWM  (push pedal)
//   RETRACT — IN1=LOW  IN2=HIGH PWM=BRAKE_PWM  (release pedal)
//   HOLD    — IN1=HIGH IN2=HIGH PWM=255         (TB6612 brake mode)
//   COAST   — IN1=LOW  IN2=LOW  PWM=0           (free)
//
// TB6612 brake mode shorts motor terminals via internal FETs.
// Back-EMF from any external force creates opposing current →
// shaft resists movement without drawing continuous supply current.
// No pulsing needed. Cleaner and cooler than PWM hold.
//
// Switches: active LOW, INPUT_PULLUP
//   PIN_LOWBRAKE_IN  GPIO14  brake pedal pressed = LOW
//   PIN_LIMIT_SWITCH GPIO13  actuator fully extended = LOW
//
// Sequence:
//   BOOT:
//     retract BRAKE_RETRACT_MS → extend until limit → HOLD
//
//   INIT→IDLE (s_want_engage goes false):
//     retract BRAKE_RETRACT_MS → extend BRAKE_POSITION_MS → HOLD
//
//   IDLE / AUTONOMOUS (no brake):
//     HOLD at ready position
//
//   BRAKE COMMANDED:
//     extend until limit switch fires → HOLD (fully engaged)
//
//   BRAKE RELEASED:
//     retract BRAKE_RETRACT_MS → extend BRAKE_POSITION_MS → HOLD
// =========================================================

// ── Timing constants (tune these in vcs_calibration.h) ───
#ifndef BRAKE_RETRACT_MS
  #define BRAKE_RETRACT_MS   5000u
#endif
#ifndef BRAKE_POSITION_MS
  #define BRAKE_POSITION_MS  2500u
#endif

typedef enum {
    BS_BOOT_RETRACTING,   // boot: clear any stuck position
    BS_INIT_EXTENDING,    // boot: extend to limit switch
    BS_ENGAGED_HOLD,      // fully engaged, TB6612 brake hold
    BS_RETRACTING,        // full retract (timed)
    BS_POSITIONING,       // extend to ready position (timed)
    BS_READY_HOLD,        // at ready position, TB6612 brake hold
    BS_BRAKE_EXTENDING,   // brake commanded, extending to limit
} BrakeState;

static BrakeState s_state    = BS_BOOT_RETRACTING;
static uint32_t   s_timer_ms = 0;

// Desired engagement — set by forceBrakeEngagement(), read here
static volatile bool s_want_engage = false;

// Debounce
bool            is_brake_pressed = false;
static uint32_t s_debounce_ms    = 0;
static int      s_last_reading   = HIGH;

// ── TB6612 helpers ────────────────────────────────────────
static void actuator_extend() {
    digitalWrite(TB6612_IN1_PIN, HIGH);
    digitalWrite(TB6612_IN2_PIN, LOW);
    ledcWrite(BRAKE_LEDC_CH, BRAKE_PWM);
}

static void actuator_retract() {
    digitalWrite(TB6612_IN1_PIN, LOW);
    digitalWrite(TB6612_IN2_PIN, HIGH);
    ledcWrite(BRAKE_LEDC_CH, BRAKE_PWM);
}

static void actuator_hold() {
    // TB6612 brake mode: both IN pins HIGH, PWM=255
    // Shorts motor terminals → resists back-drive without current draw
    digitalWrite(TB6612_IN1_PIN, HIGH);
    digitalWrite(TB6612_IN2_PIN, HIGH);
    ledcWrite(BRAKE_LEDC_CH, 255);
}

static void actuator_coast() {
    digitalWrite(TB6612_IN1_PIN, LOW);
    digitalWrite(TB6612_IN2_PIN, LOW);
    ledcWrite(BRAKE_LEDC_CH, 0);
}

static bool limit_hit() {
    return digitalRead(PIN_LIMIT_SWITCH) == LOW;  // active LOW
}

static uint32_t retract_limit_ms() {
    // Allow runtime tuning via debug command "brake_ms N"
    #ifdef dbg_retract_ms
    return (uint32_t)dbg_retract_ms;
    #else
    return (uint32_t)BRAKE_RETRACT_MS;
    #endif
}

// ── State transition ──────────────────────────────────────
static void enter_state(BrakeState next) {
    s_state    = next;
    s_timer_ms = millis();

    switch (next) {
        case BS_BOOT_RETRACTING:
            actuator_retract();
            Serial.printf("[BRAKE] BOOT retract %lums\n",
                          (unsigned long)retract_limit_ms());
            break;
        case BS_INIT_EXTENDING:
            actuator_extend();
            Serial.println(F("[BRAKE] BOOT extend → waiting for limit switch"));
            break;
        case BS_ENGAGED_HOLD:
            actuator_hold();
            digitalWrite(BRAKE_MC_PIN, LOW);
            Serial.println(F("[BRAKE] ENGAGED HOLD"));
            break;
        case BS_RETRACTING:
            actuator_retract();
            Serial.printf("[BRAKE] RETRACT %lums\n",
                          (unsigned long)retract_limit_ms());
            break;
        case BS_POSITIONING:
            actuator_extend();
            Serial.printf("[BRAKE] POSITION %lums\n",
                          (unsigned long)BRAKE_POSITION_MS);
            break;
        case BS_READY_HOLD:
            actuator_hold();
            digitalWrite(BRAKE_MC_PIN, HIGH);
            Serial.println(F("[BRAKE] READY HOLD"));
            break;
        case BS_BRAKE_EXTENDING:
            actuator_extend();
            digitalWrite(BRAKE_MC_PIN, LOW);
            Serial.println(F("[BRAKE] BRAKE extend → waiting for limit switch"));
            break;
    }
}

// ─────────────────────────────────────────────────────────
void initLowBrake() {
    // Each pinMode on its own line — no formatting ambiguity
    pinMode(PIN_LOWBRAKE_IN,  INPUT_PULLUP);
    pinMode(PIN_LIMIT_SWITCH, INPUT_PULLUP);
    pinMode(TB6612_IN1_PIN,   OUTPUT);
    pinMode(TB6612_IN2_PIN,   OUTPUT);
    pinMode(TB6612_PWM_PIN,   OUTPUT);
    pinMode(BRAKE_MC_PIN,     OUTPUT);

    ledcSetup(BRAKE_LEDC_CH, BRAKE_LEDC_FREQ, BRAKE_LEDC_RES);
    ledcAttachPin(TB6612_PWM_PIN, BRAKE_LEDC_CH);
    ledcWrite(BRAKE_LEDC_CH, 0);

    digitalWrite(BRAKE_MC_PIN, LOW);  // MC brake ON at boot

    // Print limit switch state immediately so we can verify wiring
    Serial.printf("[BRAKE] init — limit switch raw=%d (expect 1=idle)\n",
                  digitalRead(PIN_LIMIT_SWITCH));

    enter_state(BS_BOOT_RETRACTING);
}

// ─────────────────────────────────────────────────────────
void forceBrakeEngagement(bool engage) {
    // Flag only — hardware is owned by updateLowBrake()
    s_want_engage = engage;
}

// ─────────────────────────────────────────────────────────
void updateLowBrake() {

    // ── 1. Limit switch diagnostic (throttled) ────────────
    static uint32_t s_lim_print_ms = 0;
    if (millis() - s_lim_print_ms >= 500) {
        s_lim_print_ms = millis();
        Serial.printf("[LIM] raw=%d hit=%d state=%d want=%d pedal=%d\n",
                      digitalRead(PIN_LIMIT_SWITCH),
                      (int)limit_hit(),
                      (int)s_state,
                      (int)s_want_engage,
                      (int)is_brake_pressed);
    }

    // ── 2. Debounced pedal read (active LOW) ──────────────
    int reading = digitalRead(PIN_LOWBRAKE_IN);
    if (reading != s_last_reading) s_debounce_ms = millis();
    if ((millis() - s_debounce_ms) > DEBOUNCE_TIME_MS) {
        is_brake_pressed = (reading == LOW);
    }
    s_last_reading = reading;

    if (is_brake_pressed && currentState != MANUAL_STATE) s_want_engage = true;

    uint32_t elapsed = millis() - s_timer_ms;

    // ── 3. State machine ──────────────────────────────────
    switch (s_state) {

        case BS_BOOT_RETRACTING:
            if (elapsed >= retract_limit_ms()) {
                enter_state(BS_INIT_EXTENDING);
            }
            break;

        case BS_INIT_EXTENDING:
            if (limit_hit()) {
                enter_state(BS_ENGAGED_HOLD);
            }
            if (elapsed >= 8000) {
                Serial.println(F("[BRAKE] WARN: init extend timeout — check limit switch"));
                enter_state(BS_ENGAGED_HOLD);
            }
            break;

        case BS_ENGAGED_HOLD:
            if (!s_want_engage && !is_brake_pressed &&
                (currentState == IDLE_STATE      ||
                 currentState == MANUAL_STATE    ||
                 currentState == AUTONOMOUS_STATE)) {
                enter_state(BS_RETRACTING);
            }
            break;

        case BS_RETRACTING:
            if (elapsed >= retract_limit_ms()) {
                // MANUAL: stay fully retracted — skip positioning extension
                enter_state(currentState == MANUAL_STATE ? BS_READY_HOLD : BS_POSITIONING);
            }
            break;

        case BS_POSITIONING:
            if (currentState == MANUAL_STATE) {
                enter_state(BS_READY_HOLD);  // abort positioning in MANUAL — stay retracted
                break;
            }
            if (elapsed >= (uint32_t)BRAKE_POSITION_MS) {
                enter_state(BS_READY_HOLD);
            }
            // Limit fires during positioning = mounted too close
            if (limit_hit()) {
                Serial.println(F("[BRAKE] WARN: limit hit during positioning"));
                enter_state(BS_ENGAGED_HOLD);
            }
            break;

        case BS_READY_HOLD:
            if ((s_want_engage || is_brake_pressed) && currentState != MANUAL_STATE) {
                enter_state(BS_BRAKE_EXTENDING);
            }
            break;

        case BS_BRAKE_EXTENDING:
            if (limit_hit()) {
                enter_state(BS_ENGAGED_HOLD);
            }
            if (elapsed >= 8000) {
                Serial.println(F("[BRAKE] WARN: brake extend timeout"));
                enter_state(BS_ENGAGED_HOLD);
            }
            if (!s_want_engage && !is_brake_pressed) {
                enter_state(BS_RETRACTING);
            }
            break;
    }
}

bool isPhysicalBrakePressed() {
    return is_brake_pressed;
}