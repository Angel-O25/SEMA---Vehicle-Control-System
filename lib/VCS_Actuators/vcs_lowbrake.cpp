#include "vcs_lowbrake.h"

// =========================================================
// Brake subsystem
//   - Physical brake switch (GPIO 14, active HIGH)
//   - Brake limit switch  (GPIO 13, active HIGH = at limit)
//   - Linear actuator     (TB6612 H-bridge)
//   - Brake-to-MC signal  (GPIO 12, active LOW = brake on)
// =========================================================

bool is_brake_pressed = false; //  Reflects the physical pedal state, debounced and updated in updateLowBrake().   

// Debounce state for the pedal switch
static uint32_t lastDebounceTime = 0;
static int      lastButtonState  = HIGH;   // INPUT_PULLUP -> idle reads HIGH

// Time-bounded retract state (no lower limit switch on the actuator)
static bool     retracting           = false;
static uint32_t retract_started_ms   = 0;

// Forward decl
static void brake_actuator_extend();
static void brake_actuator_retract();
static void brake_actuator_coast();

void initLowBrake() {
    // Active LOW: pressed connects GPIO14 to GND (MC brake signal).
    // INPUT_PULLUP idle = HIGH = brake appears released → fail-safe if wire disconnected.    pinMode(PIN_LOWBRAKE_IN,  INPUT_PULLUP);
    pinMode(PIN_LIMIT_SWITCH, INPUT_PULLUP);

    // TB6612 actuator pins
    pinMode(TB6612_IN1_PIN, OUTPUT);
    pinMode(TB6612_IN2_PIN, OUTPUT);
    pinMode(TB6612_PWM_PIN, OUTPUT);
    ledcSetup(BRAKE_LEDC_CH, BRAKE_LEDC_FREQ, BRAKE_LEDC_RES);
    ledcAttachPin(TB6612_PWM_PIN, BRAKE_LEDC_CH);
    ledcWrite(BRAKE_LEDC_CH, 0);


    // Brake signal to motor controller (active LOW)
    pinMode(BRAKE_MC_PIN, OUTPUT);

    // Boot in fully-engaged brake (safe default)
    forceBrakeEngagement(true);
}

void updateLowBrake() {
    // ------------------------------------------------------
    // 1. Debounced read of the physical brake switch
    //    Pin is active LOW with internal pull-down:
    //    HIGH == pressed, LOW == released.
    // ------------------------------------------------------
    int reading = digitalRead(PIN_LOWBRAKE_IN);
    if (reading != lastButtonState) {
        lastDebounceTime = millis();
    }
    if ((millis() - lastDebounceTime) > DEBOUNCE_TIME_MS) {
        // Active LOW: switch pressed connects GPIO14 to GND (or MC brake signal goes LOW).
        // INPUT_PULLUP idle = HIGH = brake appears released → fail-safe if wire disconnected.
        // MC brake signal LOW = brake active → is_brake_pressed = true.
        is_brake_pressed = (reading == LOW);
    }
    lastButtonState = reading;

    // ------------------------------------------------------
    // 2. Instant override: human pushing the pedal beats the FSM.
    // ------------------------------------------------------
    if (is_brake_pressed) {
        forceBrakeEngagement(true);
    } else if (currentState == MANUAL_STATE  ||
               currentState == AUTONOMOUS_STATE ||
               currentState == IDLE_STATE) {
        // MANUAL / AUTONOMOUS: normal driving — brake follows pedal only.
        // IDLE: retraction was started on entry; this prevents
        //       updateLowBrake from immediately re-engaging during retract.
        forceBrakeEngagement(false);
    }

    // ------------------------------------------------------
    // 3. Time-bounded retract — there is no end-stop on the
    //    retract direction, so we cut power after BRAKE_RETRACT_MS
    //    to avoid slamming the actuator into its mechanical limit.
    // ------------------------------------------------------
    if (retracting && (millis() - retract_started_ms >= BRAKE_RETRACT_MS)) {
        brake_actuator_coast();
        retracting = false;
    }

}

void forceBrakeEngagement(bool engage) {
    if (engage) {
        // MC brake signal: active LOW = motor power cut
        digitalWrite(BRAKE_MC_PIN, LOW);

        // Extend actuator until limit switch trips
        if (digitalRead(PIN_LIMIT_SWITCH) == LOW) {
            // Already at full extension — coast & hold
            brake_actuator_coast();
        } else {
            brake_actuator_extend();
        }
        retracting = false;
    } else {
        // Refuse to release while the human still has the pedal down
        if (is_brake_pressed) return;

        // Release MC brake first
        digitalWrite(BRAKE_MC_PIN, HIGH);

        // Start a timed retract; updateLowBrake() will cut power
        // after BRAKE_RETRACT_MS so we don't overdrive the actuator.
        if (!retracting) {
            brake_actuator_retract();
            retract_started_ms = millis();
            retracting         = true;
        }
    }
}

bool isPhysicalBrakePressed() {
    return is_brake_pressed;
}

// =========================================================
// TB6612 helpers (channels paralleled, single direction pair)
// =========================================================
static void brake_actuator_extend() {
    digitalWrite(TB6612_IN1_PIN, HIGH);
    digitalWrite(TB6612_IN2_PIN, LOW);
    ledcWrite(BRAKE_LEDC_CH, BRAKE_PWM);   // was analogWrite
}

static void brake_actuator_retract() {
    digitalWrite(TB6612_IN1_PIN, LOW);
    digitalWrite(TB6612_IN2_PIN, HIGH);
    ledcWrite(BRAKE_LEDC_CH, BRAKE_PWM);
}

static void brake_actuator_coast() {
    digitalWrite(TB6612_IN1_PIN, LOW);
    digitalWrite(TB6612_IN2_PIN, LOW);
    ledcWrite(BRAKE_LEDC_CH, 0);
}