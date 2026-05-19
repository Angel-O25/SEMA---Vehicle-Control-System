#include "vcs_deadman.h"
#include "vcs_pins.h"

// =========================================================
// Two grip switches feed an AND gate. Both must be held
// for DEBOUNCE_TICKS consecutive samples before the FSM
// will let us promote into AUTONOMOUS_STATE.
// =========================================================

// Internal state
static bool leftGripPressed   = false;
static bool rightGripPressed  = false;
static bool autoStateRequested = false;

// Debounce: 3 ticks × 10 ms = 30 ms required to flip state
static const uint8_t DEBOUNCE_TICKS = 3;

void initDeadman() {
    // Active HIGH, no PCB pull resistor — internal pull-down keeps the
    // line at LOW when the switch is open.
    pinMode(PIN_DMS_LEFT,  INPUT_PULLUP);
    pinMode(PIN_DMS_RIGHT, INPUT_PULLUP);
}

void updateDeadman() {
    // HIGH = pressed (switch ties GPIO to 3.3V)
    bool rawLeft  = (digitalRead(PIN_DMS_LEFT)  == LOW);
    bool rawRight = (digitalRead(PIN_DMS_RIGHT) == LOW);

    // Saturating up/down counters per channel
    static uint8_t leftCounter  = 0;
    static uint8_t rightCounter = 0;

    if (rawLeft  && leftCounter  < DEBOUNCE_TICKS) leftCounter++;
    if (!rawLeft && leftCounter  > 0)              leftCounter--;
    leftGripPressed  = (leftCounter  >= DEBOUNCE_TICKS);

    if (rawRight  && rightCounter < DEBOUNCE_TICKS) rightCounter++;
    if (!rawRight && rightCounter > 0)              rightCounter--;
    rightGripPressed = (rightCounter >= DEBOUNCE_TICKS);

    // Strict AND
    autoStateRequested = (leftGripPressed && rightGripPressed);
}

bool isDeadmanActive() {
    return autoStateRequested;
}