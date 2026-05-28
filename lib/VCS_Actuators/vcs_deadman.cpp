#include "vcs_deadman.h"
#include "vcs_pins.h"

// =========================================================
// Two grip switches on steering wheel.
// Active LOW with INPUT_PULLUP:
//   switch OPEN    → GPIO = HIGH (internal pull-up)
//   switch PRESSED → GPIO = LOW  (switch ties to GND)
//
// isDeadmanActive()     — BOTH switches held (AUTONOMOUS)
// isEitherDeadmanHeld() — EITHER switch held (INIT → IDLE)
// =========================================================

static bool leftGripPressed  = false;
static bool rightGripPressed = false;

static const uint8_t DEBOUNCE_TICKS = 3;   // 3 × 10ms = 30ms

void initDeadman() {
    pinMode(PIN_DMS_LEFT,  INPUT_PULLUP);   // active LOW — pull-up to 3.3V
    pinMode(PIN_DMS_RIGHT, INPUT_PULLUP);
}

void updateDeadman() {
    // Active LOW: pressed = LOW
    bool rawLeft  = (digitalRead(PIN_DMS_LEFT)  == LOW);
    bool rawRight = (digitalRead(PIN_DMS_RIGHT) == LOW);

    static uint8_t leftCounter  = 0;
    static uint8_t rightCounter = 0;

    if ( rawLeft  && leftCounter  < DEBOUNCE_TICKS) leftCounter++;
    if (!rawLeft  && leftCounter  > 0)              leftCounter--;
    leftGripPressed  = (leftCounter  >= DEBOUNCE_TICKS);

    if ( rawRight && rightCounter < DEBOUNCE_TICKS) rightCounter++;
    if (!rawRight && rightCounter > 0)              rightCounter--;
    rightGripPressed = (rightCounter >= DEBOUNCE_TICKS);
}

// BOTH switches held — required for AUTONOMOUS entry
bool isDeadmanActive() {
    return leftGripPressed && rightGripPressed;
}

// EITHER switch held — used by INIT to confirm driver is present
bool isEitherDeadmanHeld() {
    return leftGripPressed || rightGripPressed;
}