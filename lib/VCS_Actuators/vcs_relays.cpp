#include "vcs_relays.h"
#include "vcs_pins.h"

// --- HARDWARE CONFIGURATION ---
// Most standard opto-isolated Arduino relay modules are Active Low.
// If your specific relay module turns ON when it receives 5V, swap these!
#define RELAY_ENERGIZED   LOW   // Coil gets power, NO closes, NC opens
#define RELAY_DEENERGIZED HIGH  // Coil loses power, NO opens, NC closes

void initRelays() {
    pinMode(PIN_RELAY_STROBE, OUTPUT);
    pinMode(PIN_RELAY_STATE, OUTPUT);

    // Default to Manual Mode on boot (Safe State)
    digitalWrite(PIN_RELAY_STROBE, RELAY_DEENERGIZED);
    digitalWrite(PIN_RELAY_STATE, RELAY_DEENERGIZED);
}

void updateRelays(bool isAutonomous) {
    if (isAutonomous) {
        // --- AUTONOMOUS MODE ---
        // 1. Strobe: Relay energized -> NO contact closes -> 12V flows to Orange Strobe
        digitalWrite(PIN_RELAY_STROBE, RELAY_ENERGIZED);
        
        // 2. Organizer State: Relay energized -> NO contact closes (Signals Auto to Telemetry)
        digitalWrite(PIN_RELAY_STATE, RELAY_ENERGIZED);
        
    } else {
        // --- MANUAL MODE ---
        // 1. Strobe: Relay de-energized -> NO contact opens -> Strobe turns OFF
        digitalWrite(PIN_RELAY_STROBE, RELAY_DEENERGIZED);
        
        // 2. Organizer State: Relay de-energized -> NC contact closes (Signals Manual to Telemetry)
        digitalWrite(PIN_RELAY_STATE, RELAY_DEENERGIZED);
    }
}