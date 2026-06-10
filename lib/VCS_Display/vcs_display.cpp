// ============================================================
//  vcs_display.cpp — SIDLAK 2 VCS
//  Team Wired PH0017003 | Shell Eco-marathon 2026
// ============================================================

#include "vcs_display.h"
#include "vcs_state_machine.h"
#include "vcs_lowbrake.h"
#include "vcs_reverse.h"
#include "vcs_constants.h"
#include "vcs_throttle.h"
#include "vcs_steering.h"
#include "vcs_hallsensor.h"
#include "vcs_debug.h"
#include "vcs_calibration.h"
#include "vcs_uart.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Wire.h>

extern uint8_t getANSCommandMode();

Adafruit_SH1106G display(128, 64, &Wire, -1);

static bool     s_display_ok   = false;
static uint8_t  s_display_addr = 0;
static uint32_t s_last_ping_ms = 0;
static uint8_t  s_fail_streak  = 0;

// ── I2C alive ping ────────────────────────────────────────────
static bool pingDisplay() {
    if (!s_display_addr) return false;
    Wire.beginTransmission(s_display_addr);
    return (Wire.endTransmission() == 0);
}

// ── Full re-init ──────────────────────────────────────────────
static bool doBegin(uint8_t addr) {
    Wire.end();
    delay(10);
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(100000);
    delay(10);
    bool ok = display.begin(addr, false);
    if (ok) {
        display.setTextColor(SH110X_WHITE);
        display.setTextSize(1);
        display.clearDisplay();
        display.display();
        Serial.printf("[DISPLAY] init OK @ 0x%02X\r\n", addr);
    }
    return ok;
}

// ── Watchdog ──────────────────────────────────────────────────
static void watchdogDisplay() {
    uint32_t now = millis();
    if (now - s_last_ping_ms < 2000) return;
    s_last_ping_ms = now;

    if (!pingDisplay()) {
        s_fail_streak++;
        s_display_ok = false;
        Serial.printf("[DISPLAY] ping FAIL streak=%u addr=0x%02X\r\n",
                      (unsigned)s_fail_streak, (unsigned)s_display_addr);
        if (s_fail_streak >= 2) {
            // Try last known address first, then scan
            if (s_display_addr && doBegin(s_display_addr)) {
                s_display_ok  = true;
                s_fail_streak = 0;
            } else {
                // Scan for display
                const uint8_t addrs[2] = {0x3C, 0x3D};
                for (int i = 0; i < 2 && !s_display_ok; i++) {
                    Wire.beginTransmission(addrs[i]);
                    if (Wire.endTransmission() == 0) {
                        s_display_ok  = doBegin(addrs[i]);
                        if (s_display_ok) {
                            s_display_addr = addrs[i];
                            s_fail_streak  = 0;
                        }
                    }
                }
            }
            if (!s_display_ok)
                Serial.println(F("[DISPLAY] recovery FAILED"));
        }
    } else {
        if (!s_display_ok) {
            s_display_ok  = true;
            s_fail_streak = 0;
            Serial.println(F("[DISPLAY] recovered OK"));
        } else {
            s_fail_streak = 0;
        }
    }
}

// ── Force reset (from debug command oled_reset) ───────────────
void forceResetDisplay() {
    Serial.println(F("[DISPLAY] force reset"));
    s_display_ok  = false;
    s_fail_streak = 3;
    s_last_ping_ms = 0;
    watchdogDisplay();
}

// ── Bar graph helper ──────────────────────────────────────────
static void drawBarGraph(int16_t x, int16_t y, int16_t w, int16_t h,
                         uint16_t value) {
    display.drawRect(x, y, w, h, SH110X_WHITE);
    const float POT_MIN_MV = STEER_POT_MIN_MV;
    const float POT_MAX_MV = STEER_POT_MAX_MV;
    float norm = 1.0f - ((float)value - POT_MIN_MV) / (POT_MAX_MV - POT_MIN_MV);
    norm = constrain(norm, 0.0f, 1.0f);
    int16_t fill = (int16_t)(norm * (w - 2));
    if (fill > 0) display.fillRect(x + 1, y + 1, fill, h - 2, SH110X_WHITE);
    display.drawFastVLine(x,         y + h, 3, SH110X_WHITE);
    display.drawFastVLine(x + w / 2, y + h, 3, SH110X_WHITE);
    display.drawFastVLine(x + w - 1, y + h, 3, SH110X_WHITE);
}

// ─────────────────────────────────────────────────────────────
//  initDisplay — called from setup()
//  Scans I2C bus and reports everything to serial.
// ─────────────────────────────────────────────────────────────
void initDisplay() {
    Serial.println(F("[DISPLAY] initialising..."));

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(100000);
    delay(50);

    // ── Full I2C scan (shows in serial even if display fails) ─
    Serial.println(F("[DISPLAY] I2C scan:"));
    bool any_found = false;
    for (uint8_t a = 1; a < 127; a++) {
        Wire.beginTransmission(a);
        uint8_t err = Wire.endTransmission();
        if (err == 0) {
            Serial.printf("  found device @ 0x%02X\r\n", (unsigned)a);
            any_found = true;
        }
    }
    if (!any_found) {
        Serial.println(F("  [DISPLAY] NO I2C DEVICES FOUND"));
        Serial.println(F("  Check SDA/SCL wiring and power to OLED"));
        return;  // s_display_ok stays false
    }

    // ── Try 0x3C first, then 0x3D ─────────────────────────────
    const uint8_t addrs[2] = {0x3C, 0x3D};
    for (int i = 0; i < 2 && !s_display_ok; i++) {
        Wire.beginTransmission(addrs[i]);
        if (Wire.endTransmission() != 0) continue;

        Serial.printf("[DISPLAY] trying begin() @ 0x%02X...\r\n",
                      (unsigned)addrs[i]);

        for (int attempt = 0; attempt < 3 && !s_display_ok; attempt++) {
            delay(attempt * 30);
            s_display_ok = display.begin(addrs[i], false);
        }

        if (s_display_ok) {
            s_display_addr = addrs[i];
            display.setTextColor(SH110X_WHITE);
            display.setTextSize(2);
            display.clearDisplay();
            display.setCursor(20, 10); display.println(F("SIDLAK 2"));
            display.setTextSize(1);
            display.setCursor(8,  36); display.println(F("Shell Eco-marathon 2026"));
            display.setCursor(16, 48); display.println(F("Team Wired PH0017003"));
            display.display();
            Serial.printf("[DISPLAY] OK @ 0x%02X — showing splash\r\n",
                          (unsigned)addrs[i]);
        } else {
            Serial.printf("[DISPLAY] begin() FAILED @ 0x%02X\r\n",
                          (unsigned)addrs[i]);
        }
    }

    if (!s_display_ok) {
        Serial.println(F("[DISPLAY] Could not init. DisplayLoop will keep retrying."));
        Serial.println(F("[DISPLAY] Hint: check if SH1106 or SSD1306 library matches your module"));
    }
}

// ─────────────────────────────────────────────────────────────
//  updateDisplay — called from DisplayLoop every 50ms
// ─────────────────────────────────────────────────────────────
static const char *vcsStateLabel(VcsState s) {
    switch (s) {
        case INIT_STATE:       return "INIT";
        case IDLE_STATE:       return "IDLE";
        case MANUAL_STATE:     return "MANUAL";
        case AUTONOMOUS_STATE: return "AUTO";
        case STOPPING_STATE:   return "STOP";
        default:               return "?";
    }
}
static const char *jetsonModeLabel(uint8_t m) {
    switch (m) {
        case 0: return "IDLE";  case 1: return "AUTO";
        case 2: return "MAN";   case 3: return "STOP";
        case 4: return "RREV";  case 5: return "FAULT";
        case 6: return "ESTOP"; default: return "?";
    }
}

void updateDisplay(float rpm, uint16_t steer_comm) {
    watchdogDisplay();
    if (!s_display_ok) return;

    display.clearDisplay();
    display.setTextColor(SH110X_WHITE);
    display.setTextSize(1);

    display.setCursor(0, 0);
    display.print(F("VCS:")); display.print(vcsStateLabel(currentState));
    display.setCursor(68, 0);
    display.print(F("JET:")); display.print(jetsonModeLabel(getANSCommandMode()));

    display.setCursor(0, 13);
    display.print(F("RPM:  ")); display.println((int)rpm);

    display.setCursor(0, 26);
    display.print(F("STR:  ")); display.print(steer_comm);
    display.print(F(" / 1000"));

    display.setCursor(0, 39);
    display.print(F("BRK:"));
    display.print(isPhysicalBrakePressed() ? F("ON ") : F("OFF"));
    display.print(F("   REV:"));
    display.print(isReverseEngaged() ? F("ON") : F("OFF"));

    display.drawFastHLine(0, 51, 128, SH110X_WHITE);
    display.setCursor(0, 55);
    display.print(F("SEM2026  PH0017003"));
    display.display();
}

// ─────────────────────────────────────────────────────────────
//  updateDebugDisplay — called from DisplayLoop when in debug
// ─────────────────────────────────────────────────────────────
void updateDebugDisplay() {
    watchdogDisplay();
    if (!s_display_ok) return;

    display.clearDisplay();
    display.setTextColor(SH110X_WHITE);
    display.setTextSize(1);

    static const char *titles[] = {
        "SPEED MAN", "SPEED AUTO", "STEER READ", "STEER CTRL", "BRAKE"
    };
    display.setCursor(0, 0);
    display.print(F("[D]")); display.print(titles[g_debug_screen]);
    char idx[5]; snprintf(idx, sizeof(idx), " %d/4", (int)g_debug_screen);
    display.setCursor(104, 0); display.print(idx);

    switch (g_debug_screen) {

        case DBG_SPEED_MANUAL: {
            float    rpm     = getMeasuredRPM();
            uint8_t  dac     = getThrottleDacOut();
            uint32_t pmv     = getThrottlePedalMv();
            int      pct_out = dac * 100 / 255;
            char buf[22];
            display.setCursor(0, 11);
            snprintf(buf, sizeof(buf), "RPM:%5.0f DAC:%3d%%", rpm, pct_out);
            display.print(buf);
            display.setCursor(0, 21);
            snprintf(buf, sizeof(buf), "PED:%4lumV A:%4u",
                     (unsigned long)pmv, (unsigned)current_throttle_adc);
            display.print(buf);
            display.setCursor(0, 31);
            if (dbg_throttle_override_dac >= 0)
                snprintf(buf, sizeof(buf), "OVR ON  DAC=%d",
                         dbg_throttle_override_dac);
            else
                snprintf(buf, sizeof(buf), "OVR OFF (pedal active)");
            display.print(buf);
            display.drawFastHLine(0, 43, 128, SH110X_WHITE);
            display.setCursor(0, 47);
            display.print(F("throttle N | throttle off"));
            display.setCursor(0, 57);
            display.print(F("n/p=scr  exit=normal"));
            break;
        }

        case DBG_SPEED_AUTO: {
            float   rpm_act = getMeasuredRPM();
            float   rpm_tgt = dbg_target_rpm;
            uint8_t dac     = getThrottleDacOut();
            float   spd     = rpm_act * WHEEL_CIRCUMFERENCE_M / 60.0f;
            char buf[22];
            display.setCursor(0, 11);
            snprintf(buf, sizeof(buf), "ACT:%5.0f TGT:%5.0f",
                     rpm_act, rpm_tgt);
            display.print(buf);
            display.setCursor(0, 21);
            snprintf(buf, sizeof(buf), "ERR:%+6.0f  DAC:%3d",
                     rpm_tgt - rpm_act, (int)dac);
            display.print(buf);
            display.setCursor(0, 31);
            snprintf(buf, sizeof(buf), "SPD:%.2fm/s  %.1fkm/h",
                     spd, spd * 3.6f);
            display.print(buf);
            display.drawFastHLine(0, 43, 128, SH110X_WHITE);
            display.setCursor(0, 47);
            display.print(F("target_rpm N"));
            display.setCursor(0, 57);
            display.print(F("n/p=scr  exit=normal"));
            break;
        }

        case DBG_STEER_READ: {
            uint32_t raw_mv = getSteeringRawMv();
            uint16_t comm   = getMeasuredSteering();
            float    ang    = ((float)comm - 500.0f) / 500.0f * 18.0f;
            char buf[22];
            display.setCursor(0, 10);
            snprintf(buf, sizeof(buf), "RAW:%4lumV POS:%4u",
                     (unsigned long)raw_mv, (unsigned)comm);
            display.print(buf);
            drawBarGraph(0, 21, 128, 9, (uint16_t)raw_mv);
            display.setCursor(0,   33); display.print('L');
            display.setCursor(61,  33); display.print('C');
            display.setCursor(122, 33); display.print('R');
            display.setCursor(0, 39);
            const char *dir = (comm < 400) ? "LEFT  "
                            : (comm > 600) ? "RIGHT " : "CNTRL";
            snprintf(buf, sizeof(buf), "%-6s %+.1fdeg ENA:OFF",
                     dir, ang);
            display.print(buf);
            display.setCursor(0, 49);
            display.print(F("set_full_l/r/center"));
            display.setCursor(0, 57);
            display.print(F("n/p=scr  exit=normal"));
            break;
        }

        case DBG_STEER_CTRL: {
            uint32_t raw_mv = getSteeringRawMv();
            int32_t  err    = (int32_t)dbg_steer_target_mv
                            - (int32_t)raw_mv;
            char buf[22];
            display.setCursor(0, 10);
            snprintf(buf, sizeof(buf), "RAW:%4lumV TGT:%4lu",
                     (unsigned long)raw_mv,
                     (unsigned long)dbg_steer_target_mv);
            display.print(buf);
            drawBarGraph(0, 21, 128, 9, (uint16_t)raw_mv);
            display.setCursor(0,   33); display.print('L');
            display.setCursor(61,  33); display.print('C');
            display.setCursor(122, 33); display.print('R');
            display.setCursor(0, 39);
            snprintf(buf, sizeof(buf), "ERR:%+5ldmV ENA:ON",
                     (long)err);
            display.print(buf);
            display.setCursor(0, 49);
            display.print(F("steer_mv N (pot mV)"));
            display.setCursor(0, 57);
            display.print(F("n/p=scr  exit=normal"));
            break;
        }

        case DBG_BRAKE: {
            bool sw  = isPhysicalBrakePressed();
            bool lim = (digitalRead(PIN_LIMIT_SWITCH) == LOW);  // active LOW
            bool mc  = (digitalRead(BRAKE_MC_PIN) == LOW);
            char buf[22];
            display.setCursor(0, 10);
            snprintf(buf, sizeof(buf), "SW:%-7s LIM:%-5s",
                     sw  ? "PRESSED" : "RELEASE",
                     lim ? "HIT"     : "CLEAR");
            display.print(buf);
            display.setCursor(0, 20);
            snprintf(buf, sizeof(buf), "MC_BRK:%-6s RMS:%lu",
                     mc  ? "ACTIVE" : "off",
                     (unsigned long)dbg_retract_ms);
            display.print(buf);
            display.setCursor(0, 30);
            display.print(F("DBG: Btn=EXTEND Rel=RETR"));
            display.setCursor(0, 40);
            display.print(F("brake_ms N | print_cal"));
            display.setCursor(0, 50);
            display.print(F("Normal op not affected"));
            display.setCursor(0, 58);
            display.print(F("n/p=scr  exit=normal"));
            break;
        }

        default: break;
    }

    display.display();
}