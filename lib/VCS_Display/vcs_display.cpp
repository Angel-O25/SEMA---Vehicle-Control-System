#include "vcs_display.h"
#include "vcs_state_machine.h"
#include "vcs_lowbrake.h"
#include "vcs_reverse.h"
#include "vcs_constants.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

// SH1106 chip (128x64, I2C addr 0x3C).
// NOTE: The display physically looks like an SSD1306 but uses a different
// internal RAM offset. Using SSD1306 driver causes grainy static output.
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

    // Splash — Jetson takes ~30s to boot, fill that time.
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

// Maps VcsState to a short status string for the OLED.
static const char* stateLabel(VcsState s) {
    switch (s) {
        case INIT_STATE:       return "INIT";
        case IDLE_STATE:       return "IDLE";
        case MANUAL_STATE:     return "MANUAL";
        case AUTONOMOUS_STATE: return "AUTONOMOUS";
        case STOPPING_STATE:   return "STOPPING";
        default:               return "UNKNOWN";
    }
}

// ─── Layout (128 x 64 px, 6px per char at textSize=1) ─────────────────
//  Row 0  y= 0  STATE   label + value
//  Row 1  y=14  RPM     label + value
//  Row 2  y=28  STEER   label + value (raw mV)
//  Row 3  y=42  BRAKE   label + state + "  REV:" + state
//  Divider line at y=52
//  Row 4  y=55  small   "SEM 2026  PH0017003"
// ───────────────────────────────────────────────────────────────────────
void updateDisplay(float rpm, uint16_t steer_comm) {
    if (!s_display_ok) return;

    display.clearDisplay();
    display.setTextColor(SH110X_WHITE);
    display.setTextSize(1);

    // ── Row 0: State ────────────────────────────────────────────────
    display.setCursor(0, 0);
    display.print(F("STATE: "));
    display.println(stateLabel(currentState));

    // ── Row 1: RPM ──────────────────────────────────────────────────
    display.setCursor(0, 14);
    display.print(F("RPM:   "));
    display.println((int)rpm);

    // ── Row 2: Steering position (COMM units 0-1000) ────────────────
    display.setCursor(0, 28);
    display.print(F("STEER: "));
    display.print(steer_comm);
    display.print(F(" / 1000"));

    // ── Row 3: Brake + Reverse ──────────────────────────────────────
    display.setCursor(0, 42);
    display.print(F("BRK:"));
    display.print(isPhysicalBrakePressed() ? F("ON ") : F("OFF"));
    display.print(F("  REV:"));
    display.print(isReverseEngaged()       ? F("ON")  : F("OFF"));

    // ── Divider ─────────────────────────────────────────────────────
    display.drawFastHLine(0, 53, 128, SH110X_WHITE);

    // ── Footer ──────────────────────────────────────────────────────
    display.setCursor(0, 56);
    display.print(F("SEM2026 PH0017003"));

    display.display();
}