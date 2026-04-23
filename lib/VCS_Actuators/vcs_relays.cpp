#include "vcs_relays.h"
#include "vcs_pins.h"

// --- HARDWARE CONFIGURATION ---
// Most standard opto-isolated Arduino relay modules are Active Low.
// If your specific relay module turns ON when it receives 5V, swap these!
#define RELAY_ENERGIZED   LOW   // Coil gets power, NO closes, NC opens
#define RELAY_DEENERGIZED HIGH  // Coil loses power, NO opens, NC closes

void initRelays() {
    pinMode(PIN_RELAY_STROBE, OUTPUT);
    
    #if !defined(ESP32_VCS)
        pinMode(PIN_RELAY_STATE, OUTPUT);
    #endif

    // Default to Manual Mode on boot (Safe State)
    digitalWrite(PIN_RELAY_STROBE, RELAY_DEENERGIZED);
    
    #if !defined(ESP32_VCS)
        digitalWrite(PIN_RELAY_STATE, RELAY_DEENERGIZED);
    #endif
}

void updateRelays(bool isAutonomous) {
    if (isAutonomous) {
        // --- AUTONOMOUS MODE ---
        // 1. Strobe: Relay energized -> NO contact closes -> 12V flows to Orange Strobe
        digitalWrite(PIN_RELAY_STROBE, RELAY_ENERGIZED);
        
        #if !defined(ESP32_VCS)
            // 2. Organizer State: Relay energized
            digitalWrite(PIN_RELAY_STATE, RELAY_ENERGIZED);
        #endif
        
    } else {
        // --- MANUAL MODE ---
        // 1. Strobe: Relay de-energized -> NO contact opens -> Strobe turns OFF
        digitalWrite(PIN_RELAY_STROBE, RELAY_DEENERGIZED);
        
        #if !defined(ESP32_VCS)
            // 2. Organizer State: Relay de-energized 
            digitalWrite(PIN_RELAY_STATE, RELAY_DEENERGIZED);
        #endif
    }
}