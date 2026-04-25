#include "vcs_display.h"
#include "vcs_state_machine.h"
#include "vcs_reverse.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

Adafruit_SSD1306 display(128, 64, &Wire, -1);

void initDisplay() {
    // Explicitly assign I2C pins for ESP32 to prevent defaults routing elsewhere
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        // Log I2C failure if necessary
    }

    Wire.setClock(400000);
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE); 
}

void updateDisplay(float rpm, uint16_t steer, DriveMode speedMode) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.setTextSize(2);
    
    if(currentState == FAULT_STATE) {
        display.println("!! CRITICAL !!");
    } else {
        display.println(currentState == AUTONOMOUS_STATE ? "AUTO MODE" : "MANUAL");
    }

    display.setTextSize(1);
    
    display.print("\nRPM:   ");
    display.println(rpm, 1);
    
    display.print("STEER: ");
    display.println(steer);
    
    display.print("GEAR:  ");
    if (isReverseEngaged()) {
        display.println("REVERSE"); 
    } else {
        display.println(speedMode == DRIVE_LOW ? "LOW" : (speedMode == DRIVE_HIGH ? "HIGH" : "MED"));
    }
    
    if (currentState == MANUAL_STATE && getDMSHoldStartTime() > 0) {
        display.setCursor(0, 45);
        display.print("ENGAGING AUTO..."); 
        
        int progress = (millis() - getDMSHoldStartTime()) / 10; 
        
        display.drawRect(0, 55, 128, 5, SSD1306_WHITE);
        display.fillRect(0, 55, (progress * 128) / 100, 5, SSD1306_WHITE);
    }
    
    display.display(); 
}