#include "vcs_deadman.h"
#include "vcs_pins.h"
#include "vcs_web.h" // [CRITICAL ADDITION] Grants access to global simulation flags

// Internal State Variables
static bool leftGripPressed = false;
static bool rightGripPressed = false;
static bool autoStateRequested = false;

// Debounce Configuration (Assumes updateDeadman() is called at 100Hz / 10ms)
// 3 ticks * 10ms = 30ms required to change state
const uint8_t DEBOUNCE_TICKS = 3; 

void initDeadman() {
    #if defined(ESP32_VCS)
        // ESP32 Hardware Design: Active HIGH, firmware pull-down
        // Unpressed = LOW, Pressed = HIGH to 3.3V
        pinMode(PIN_DMS_LEFT, INPUT_PULLDOWN);
        pinMode(PIN_DMS_RIGHT, INPUT_PULLDOWN);
    #else
        pinMode(PIN_DMS_LEFT, INPUT_PULLUP);
        pinMode(PIN_DMS_RIGHT, INPUT_PULLUP);
    #endif
}

void updateDeadman() {
    #if defined(ESP32_VCS)
        // 1. Read Raw Hardware States (HIGH means the button is pressed)
        bool rawLeft  = (digitalRead(PIN_DMS_LEFT) == HIGH);
        bool rawRight = (digitalRead(PIN_DMS_RIGHT) == HIGH);
    #else
        // 1. Read Raw Hardware States (LOW means the button is pressed)
        bool rawLeft  = (digitalRead(PIN_DMS_LEFT) == LOW);
        bool rawRight = (digitalRead(PIN_DMS_RIGHT) == LOW);
    #endif

    // Static counters for debouncing memory
    static uint8_t leftCounter = 0;
    static uint8_t rightCounter = 0;

    // 2. Filter Left Grip
    if (rawLeft) {
        if (leftCounter < DEBOUNCE_TICKS) leftCounter++;
    } else {
        if (leftCounter > 0) leftCounter--;
    }
    // Lock in the clean state
    leftGripPressed = (leftCounter >= DEBOUNCE_TICKS);

    // 3. Filter Right Grip
    if (rawRight) {
        if (rightCounter < DEBOUNCE_TICKS) rightCounter++;
    } else {
        if (rightCounter > 0) rightCounter--;
    }
    // Lock in the clean state
    rightGripPressed = (rightCounter >= DEBOUNCE_TICKS);

    // 4. The Strict AND Gate Logic
    // Both must be explicitly pressed to request Autonomous State
    if (leftGripPressed && rightGripPressed) {
        autoStateRequested = true;
    } else {
        autoStateRequested = false;
    }
}

bool isDeadmanActive() {    
    // --- LIVE HARDWARE MODE ---
    return autoStateRequested;
}