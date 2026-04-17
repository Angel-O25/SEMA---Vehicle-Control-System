#include "vcs_display.h"
#include "vcs_state_machine.h"
#include "vcs_reverse.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

Adafruit_SSD1306 display(128, 64, &Wire, -1);

void initDisplay() {
    // [REMOVED] pinMode(PIN_LED_FAULT, OUTPUT); -> Handled by vcs_relays.cpp now

    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        // Log I2C failure if necessary
    }
    display.clearDisplay();
    // Use the explicit SSD1306_WHITE to prevent macro collisions on ARM
    display.setTextColor(SSD1306_WHITE); 
}

void updateDisplay(float rpm, uint16_t steer, DriveMode speedMode) {
    // --- 1. OLED REFRESH ---
    display.clearDisplay();
    display.setCursor(0,0);
    display.setTextSize(2);
    
    // Header based on State
    if(currentState == FAULT_STATE || currentState == ESTOP_STATE) {
        display.println("!! CRITICAL !!");
    } else {
        display.println(currentState == AUTONOMOUS_STATE ? "AUTO MODE" : "MANUAL");
    }

    display.setTextSize(1);
    
    // Telemetry Data
    display.print("\nRPM:   ");
    display.println(rpm, 1);
    
    display.print("STEER: ");
    display.println(steer);
    
    // --- 2. SECURE GEAR DISPLAY ---
    display.print("GEAR:  ");
    // Check the hardware reverse state first
    if (isReverseEngaged()) {
        display.println("REVERSE"); 
    } else {
        // If not in reverse, show the forward 3-speed state using the new Enum
        display.println(speedMode == DRIVE_LOW ? "LOW" : (speedMode == DRIVE_HIGH ? "HIGH" : "MED"));
    }
    
    // --- 3. DMS AUTO-ENGAGE PROGRESS BAR ---
    if (currentState == MANUAL_STATE && getDMSHoldStartTime() > 0) {
        display.setCursor(0, 45);
        display.print("ENGAGING AUTO..."); 
        
        // Draw a small progress bar based on the 1000ms timer
        int progress = (millis() - getDMSHoldStartTime()) / 10; // 0 to 100
        
        // Integer math used instead of float (* 1.28) to prevent visual artifacts
        display.drawRect(0, 55, 128, 5, SSD1306_WHITE);
        display.fillRect(0, 55, (progress * 128) / 100, 5, SSD1306_WHITE);
    }
    
    // Moved to the bottom so everything actually renders
    display.display(); 
}