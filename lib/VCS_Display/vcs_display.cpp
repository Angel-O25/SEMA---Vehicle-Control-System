#include "vcs_display.h"
#include "vcs_state_machine.h"
#include "vcs_lowbrake.h"
#include "vcs_reverse.h"
#include "vcs_constants.h"
extern uint8_t getANSCommandMode();   // from vcs_communication / vcs_uart
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

// SH1106 chip (128x64, I2C addr 0x3C).
Adafruit_SH1106G display(128, 64, &Wire, -1);

static bool s_display_ok = false;

void initDisplay() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000);

    s_display_ok = display.begin(0x3C, true);
    if (!s_display_ok) {
        Serial.println(F("[DISPLAY] SH1106 not found at 0x3C"));
        return;
    }

    display.clearDisplay();
    display.setTextColor(SH110X_WHITE);
    display.setTextSize(1);

    // Splash screen
    display.setCursor(20, 10);
    display.setTextSize(2);
    display.println(F("SIDLAK 2"));
    display.setTextSize(1);
    display.setCursor(8, 36);
    display.println(F("Shell Eco-marathon 2026"));
    display.setCursor(16, 48);
    display.println(F("Team Wired PH0017003"));
    display.display();
}

// VCS state → short label
static const char* vcsStateLabel(VcsState s) {
    switch (s) {
        case INIT_STATE:       return "INIT";
        case IDLE_STATE:       return "IDLE";
        case MANUAL_STATE:     return "MANUAL";
        case AUTONOMOUS_STATE: return "AUTO";
        case STOPPING_STATE:   return "STOPPING";
        default:               return "?";
    }
}

// Jetson MODE byte → short label  (see UART protocol doc)
// 0=IDLE 1=AUTO 2=MANUAL 3=STOPPING 4=REV_RCVR 5=FAULT 6=ESTOP
static const char* jetsonModeLabel(uint8_t mode) {
    switch (mode) {
        case 0: return "IDLE";
        case 1: return "AUTO";
        case 2: return "MANUAL";
        case 3: return "STOPPING";
        case 4: return "REV_RCVR";
        case 5: return "FAULT";
        case 6: return "ESTOP";
        default: return "?";
    }
}

// ─── OLED Layout (128 x 64 px, textSize=1 → 6×8 px per char) ──────────
//  y= 0   VCS: <state>    JETSON: <mode>
//  y=13   RPM: <rpm>
//  y=26   STR: <steer> / 1000
//  y=39   BRK: ON/OFF   REV: ON/OFF
//  ─────────────────────── divider y=51
//  y=55   SEM2026  PH0017003
// ────────────────────────────────────────────────────────────────────────
void updateDisplay(float rpm, uint16_t steer_comm) {
    if (!s_display_ok) return;

    display.clearDisplay();
    display.setTextColor(SH110X_WHITE);
    display.setTextSize(1);

    // ── Row 0: VCS state + Jetson mode ──────────────────────────────
    display.setCursor(0, 0);
    display.print(F("VCS:"));
    display.print(vcsStateLabel(currentState));

    // Right-align Jetson mode label
    // "JET:" + up to 8 chars — starts at x=68 to stay within 128px
    display.setCursor(68, 0);
    display.print(F("JET:"));
    display.print(jetsonModeLabel(getANSCommandMode()));

    // ── Row 1: RPM ──────────────────────────────────────────────────
    display.setCursor(0, 13);
    display.print(F("RPM:  "));
    display.println((int)rpm);

    // ── Row 2: Steering ─────────────────────────────────────────────
    display.setCursor(0, 26);
    display.print(F("STR:  "));
    display.print(steer_comm);
    display.print(F(" / 1000"));

    // ── Row 3: Brake + Reverse ──────────────────────────────────────
    display.setCursor(0, 39);
    display.print(F("BRK:"));
    display.print(isPhysicalBrakePressed() ? F("ON ") : F("OFF"));
    display.print(F("   REV:"));
    display.print(isReverseEngaged()       ? F("ON")  : F("OFF"));

    // ── Divider ─────────────────────────────────────────────────────
    display.drawFastHLine(0, 51, 128, SH110X_WHITE);

    // ── Footer ──────────────────────────────────────────────────────
    display.setCursor(0, 55);
    display.print(F("SEM2026  PH0017003"));

    display.display();
}