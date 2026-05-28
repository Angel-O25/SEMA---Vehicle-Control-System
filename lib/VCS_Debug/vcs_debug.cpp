// ============================================================
//  vcs_debug.cpp — SIDLAK 2 VCS Debug / Calibration Mode
//  Team Wired PH0017003 | Shell Eco-marathon 2026
// ============================================================

#include "vcs_debug.h"
#include "vcs_calibration.h"
#include "vcs_steering.h"
#include "vcs_lowbrake.h"
#include "vcs_pins.h"

// ── Global state ─────────────────────────────────────────────
volatile bool        g_debug_mode              = false;
volatile DebugScreen g_debug_screen            = DBG_SPEED_MANUAL;

volatile int         dbg_throttle_override_dac = -1;
volatile uint32_t    dbg_steer_target_mv       = (uint32_t)STEER_POT_CENTER_MV;
volatile float       dbg_target_rpm            = 0.0f;

volatile uint32_t    dbg_pot_min_mv    = (uint32_t)STEER_POT_MIN_MV;
volatile uint32_t    dbg_pot_center_mv = (uint32_t)STEER_POT_CENTER_MV;
volatile uint32_t    dbg_pot_max_mv    = (uint32_t)STEER_POT_MAX_MV;
volatile uint32_t    dbg_retract_ms    = BRAKE_RETRACT_MS;

// ── Serial input buffer ───────────────────────────────────────
static char    s_buf[64];
static uint8_t s_len = 0;

// ─────────────────────────────────────────────────────────────
//  Screen change handler — called whenever g_debug_screen changes
// ─────────────────────────────────────────────────────────────
static void onScreenChange(DebugScreen old_screen, DebugScreen new_screen) {
    // Leaving steer_ctrl → restore normal stepper speed
    if (old_screen == DBG_STEER_CTRL && new_screen != DBG_STEER_CTRL) {
        ledcSetup(0, STEPPER_MAX_HZ, 8);
        ledcAttachPin(PIN_STEER_PUL, 0);
        Serial.printf("[STEER] Speed restored to %d Hz\r\n", (int)STEPPER_MAX_HZ);
    }

    // Entering steer_ctrl → center target + slow speed
    if (new_screen == DBG_STEER_CTRL) {
        dbg_steer_target_mv = (uint32_t)STEER_POT_CENTER_MV;
        ledcSetup(0, 300, 8);         // 300 Hz — safe for manual test
        ledcAttachPin(PIN_STEER_PUL, 0);
        Serial.println(F("[STEER] Debug steer_ctrl entered"));
        Serial.println(F("        Speed: 300 Hz (slow). Centering first."));
        Serial.printf( "        Valid range: %d – %d mV   center: %d mV\r\n",
                       (int)STEER_POT_MIN_MV, (int)STEER_POT_MAX_MV,
                       (int)STEER_POT_CENTER_MV);
        Serial.println(F("        Type 'steer_mv N' to test. Auto-returns to center after 1.5s."));
    }
}

// ─────────────────────────────────────────────────────────────
//  Auto-return to center after target is reached
//  Called every 50 ms from DebugTask
// ─────────────────────────────────────────────────────────────
static void handleSteerCtrlAutoReturn() {
    if (!g_debug_mode || g_debug_screen != DBG_STEER_CTRL) return;

    // Already at center — nothing to do
    if (dbg_steer_target_mv == (uint32_t)STEER_POT_CENTER_MV) return;

    static uint32_t s_reached_ms = 0;
    uint32_t raw = getSteeringRawMv();
    int32_t  err = (int32_t)raw - (int32_t)dbg_steer_target_mv;

    if (abs(err) < 80) {                          // within 80 mV of target
        if (s_reached_ms == 0) {
            s_reached_ms = millis();
            Serial.printf("[STEER] Reached %lu mV — returning to center in 1.5s\r\n",
                          (unsigned long)dbg_steer_target_mv);
        }
        if (millis() - s_reached_ms > 1500) {
            dbg_steer_target_mv = (uint32_t)STEER_POT_CENTER_MV;
            s_reached_ms = 0;
            Serial.println(F("[STEER] Returning to center"));
        }
    } else {
        s_reached_ms = 0;
    }
}

// ─────────────────────────────────────────────────────────────
static void printHelp() {
    Serial.println(F("\n=== DEBUG COMMANDS ==="));
    Serial.println(F("  n / p              next / prev screen"));
    Serial.println(F("  screen <0-4>       jump to screen"));
    Serial.println(F("  screen speed_manual / speed_auto / steer_read / steer_ctrl / brake"));
    Serial.println(F("  throttle <0-100>   force DAC output % (speed screens only)"));
    Serial.println(F("  throttle off       release DAC override"));
    Serial.println(F("  target_rpm <N>     set auto RPM target (speed_auto screen)"));
    Serial.println(F("  steer_mv <N>       set steer target mV (steer_ctrl screen)"));
    Serial.println(F("  steer_test <N> [ccw]  RAW pulse test: N seconds, bypass PID"));
    Serial.println(F("  steer_ena on|off   force ENA pin (test DM542 enable/disable)"));
    Serial.println(F("  steer_dir 0|1      force DIR pin"));
    Serial.println(F("  set_full_l         store current pot mV as STEER_POT_MIN_MV"));
    Serial.println(F("  set_center         store current pot mV as STEER_POT_CENTER_MV"));
    Serial.println(F("  set_full_r         store current pot mV as STEER_POT_MAX_MV"));
    Serial.println(F("  brake_ms <N>       set BRAKE_RETRACT_MS"));
    Serial.println(F("  print_cal          print copy-pasteable calibration defines"));
    Serial.println(F("  exit               leave debug mode"));
    Serial.println();
}

// ─────────────────────────────────────────────────────────────
static void parseCommand(const char *cmd) {

    // ── Exit ─────────────────────────────────────────────────
    if (strcmp(cmd, "exit") == 0) {
        DebugScreen old = g_debug_screen;
        onScreenChange(old, DBG_SPEED_MANUAL);   // restore speed if in steer_ctrl
        dbg_throttle_override_dac = -1;
        g_debug_mode = false;
        Serial.println(F("[DEBUG] Exited debug mode — normal operation resumed"));
        return;
    }

    // ── Help ─────────────────────────────────────────────────
    if (strcmp(cmd, "?") == 0 || strcmp(cmd, "help") == 0) {
        printHelp();
        return;
    }

    // ── Screen navigation ────────────────────────────────────
    if (strcmp(cmd, "n") == 0) {
        DebugScreen old = g_debug_screen;
        DebugScreen nxt = (DebugScreen)((g_debug_screen + 1) % DBG_SCREEN_COUNT);
        g_debug_screen  = nxt;
        onScreenChange(old, nxt);
        return;
    }
    if (strcmp(cmd, "p") == 0) {
        DebugScreen old = g_debug_screen;
        int s = (int)g_debug_screen - 1;
        if (s < 0) s = DBG_SCREEN_COUNT - 1;
        DebugScreen nxt = (DebugScreen)s;
        g_debug_screen  = nxt;
        onScreenChange(old, nxt);
        return;
    }
    if (strncmp(cmd, "screen ", 7) == 0) {
        const char *arg = cmd + 7;
        int n = -1;
        if      (strcmp(arg, "speed_manual") == 0) n = 0;
        else if (strcmp(arg, "speed_auto")   == 0) n = 1;
        else if (strcmp(arg, "steer_read")   == 0) n = 2;
        else if (strcmp(arg, "steer_ctrl")   == 0) n = 3;
        else if (strcmp(arg, "brake")        == 0) n = 4;
        else n = constrain(atoi(arg), 0, DBG_SCREEN_COUNT - 1);
        DebugScreen old = g_debug_screen;
        DebugScreen nxt = (DebugScreen)n;
        g_debug_screen  = nxt;
        onScreenChange(old, nxt);
        return;
    }

    // ── Throttle override ────────────────────────────────────
    if (strncmp(cmd, "throttle ", 9) == 0) {
        const char *arg = cmd + 9;
        if (strcmp(arg, "off") == 0) {
            dbg_throttle_override_dac = -1;
            Serial.println(F("[DEBUG] throttle override released"));
        } else {
            int pct = constrain(atoi(arg), 0, 100);
            dbg_throttle_override_dac = (pct * 255) / 100;
            Serial.printf("[DEBUG] throttle -> %d%% (DAC=%d)\n", pct, dbg_throttle_override_dac);
        }
        return;
    }

    // ── Auto RPM target ──────────────────────────────────────
    if (strncmp(cmd, "target_rpm ", 11) == 0) {
        dbg_target_rpm = (float)atof(cmd + 11);
        Serial.printf("[DEBUG] target_rpm = %.1f RPM\n", dbg_target_rpm);
        return;
    }

    // ── Steer target (pot mV) — validated against pot range ──
    if (strncmp(cmd, "steer_mv ", 9) == 0) {
        int32_t requested = atoi(cmd + 9);
        int32_t pot_lo    = (int32_t)min(STEER_POT_MIN_MV, STEER_POT_MAX_MV);
        int32_t pot_hi    = (int32_t)max(STEER_POT_MIN_MV, STEER_POT_MAX_MV);

        if (requested < pot_lo || requested > pot_hi) {
            Serial.printf("[STEER] Out of range! Valid: %d – %d mV  (center: %d)\r\n",
                          pot_lo, pot_hi, (int)STEER_POT_CENTER_MV);
        } else {
            dbg_steer_target_mv = (uint32_t)requested;
            int32_t dist = abs(requested - (int32_t)STEER_POT_CENTER_MV);
            Serial.printf("[STEER] Target: %d mV  (%+d mV from center)\r\n",
                          (int)requested, (int)(requested - (int32_t)STEER_POT_CENTER_MV));
            Serial.println(F("        Will auto-return to center after 1.5s at target."));
        }
        return;
    }

    // ── Pot calibration ──────────────────────────────────────
    if (strcmp(cmd, "set_full_l") == 0) {
        dbg_pot_min_mv = getSteeringRawMv();
        Serial.printf("[DEBUG] STEER_POT_MIN_MV  = %lu mV (full-left)\n",
                      (unsigned long)dbg_pot_min_mv);
        return;
    }
    if (strcmp(cmd, "set_center") == 0) {
        dbg_pot_center_mv = getSteeringRawMv();
        Serial.printf("[DEBUG] STEER_POT_CENTER_MV = %lu mV (center/straight)\n",
                      (unsigned long)dbg_pot_center_mv);
        return;
    }
    if (strcmp(cmd, "set_full_r") == 0) {
        dbg_pot_max_mv = getSteeringRawMv();
        Serial.printf("[DEBUG] STEER_POT_MAX_MV  = %lu mV (full-right)\n",
                      (unsigned long)dbg_pot_max_mv);
        return;
    }

    // ── Brake retract time ───────────────────────────────────
    if (strncmp(cmd, "brake_ms ", 9) == 0) {
        dbg_retract_ms = (uint32_t)constrain(atoi(cmd + 9), 100, 5000);
        Serial.printf("[DEBUG] BRAKE_RETRACT_MS = %lu ms\n", (unsigned long)dbg_retract_ms);
        return;
    }

    // ── Print calibration ────────────────────────────────────
    if (strcmp(cmd, "print_cal") == 0) {
        Serial.println(F("\n--- COPY TO vcs_calibration.h ---"));
        Serial.printf("#define STEER_POT_MIN_MV    %.1ff   // full-left\n",  (float)dbg_pot_min_mv);
        Serial.printf("#define STEER_POT_CENTER_MV %.1ff   // straight-ahead\n", (float)dbg_pot_center_mv);
        Serial.printf("#define STEER_POT_MAX_MV    %.1ff   // full-right\n", (float)dbg_pot_max_mv);
        Serial.printf("#define BRAKE_RETRACT_MS    %lu\n",  (unsigned long)dbg_retract_ms);
        Serial.println(F("---------------------------------\n"));
        return;
    }


    // ── Raw stepper pulse test (bypasses all PID logic) ──────────
    // Usage: steer_test 2     → pulse CW for 2 seconds
    //        steer_test 2 ccw → pulse CCW for 2 seconds
    // If motor moves: PID/mapping bug. If not: hardware/wiring bug.
    if (strncmp(cmd, "steer_test ", 11) == 0) {
        int secs = constrain(atoi(cmd + 11), 1, 5);
        bool ccw  = (strstr(cmd + 11, "ccw") != nullptr);
        Serial.printf("[STEER_TEST] RAW PULSE TEST: %ds %s — bypassing PID\r\n",
                      secs, ccw ? "CCW" : "CW");
        Serial.printf("[STEER_TEST] ENA->HIGH, DIR->%s, duty=128 @ %dHz\r\n",
                      ccw ? "LOW" : "HIGH", (int)STEPPER_MAX_HZ);
        Serial.printf("[STEER_TEST] Watch motor. If NO movement -> hardware/wiring issue.\r\n");
        Serial.printf("[STEER_TEST] If movement -> PID or mapping bug.\r\n");

        digitalWrite(PIN_STEER_ENA, HIGH);
        delayMicroseconds(20);                         // ENA setup time
        digitalWrite(PIN_STEER_DIR, ccw ? LOW : HIGH);
        delayMicroseconds(5);
        ledcWrite(0, 128);                             // start pulses

        vTaskDelay((secs * 1000) / portTICK_PERIOD_MS);

        ledcWrite(0, 0);
        digitalWrite(PIN_STEER_ENA, LOW);
        uint32_t raw = getSteeringRawMv();
        Serial.printf("[STEER_TEST] Done. Final pot reading: %lu mV\r\n",
                      (unsigned long)raw);
        return;
    }

    // ── Force ENA pin (test DM542 enable) ────────────────────────
    // steer_ena on  → GPIO23 HIGH  (motor enabled/locked)
    // steer_ena off → GPIO23 LOW   (motor disabled/free)
    if (strncmp(cmd, "steer_ena ", 10) == 0) {
        const char *arg = cmd + 10;
        bool en = (strcmp(arg, "on") == 0 || strcmp(arg, "1") == 0);
        digitalWrite(PIN_STEER_ENA, en ? HIGH : LOW);
        Serial.printf("[STEER_ENA] GPIO23 -> %s (%s)\r\n",
                      en ? "HIGH" : "LOW",
                      en ? "motor ENABLED/locked" : "motor DISABLED/free");
        Serial.printf("[STEER_ENA] Try rotating shaft by hand to confirm.\r\n");
        return;
    }

    // ── Force DIR pin ─────────────────────────────────────────────
    if (strncmp(cmd, "steer_dir ", 10) == 0) {
        bool hi = (atoi(cmd + 10) != 0);
        digitalWrite(PIN_STEER_DIR, hi ? HIGH : LOW);
        Serial.printf("[STEER_DIR] GPIO19 -> %s\r\n", hi ? "HIGH" : "LOW");
        return;
    }

    // ── OLED force reset ─────────────────────────────────────
    if (strcmp(cmd, "oled_reset") == 0) {
        extern void forceResetDisplay();
        forceResetDisplay();
        Serial.println("[DEBUG] OLED reset triggered");
        return;
    }

    // ── Unknown ──────────────────────────────────────────────
    Serial.printf("[DEBUG] Unknown command: \"%s\"  (type ? for help)\n", cmd);
}

// ─────────────────────────────────────────────────────────────
void DebugTask(void *pvParameters) {
    for (;;) {
        // Auto-return steer to center after reaching target
        handleSteerCtrlAutoReturn();

        while (Serial.available()) {
            char c = (char)Serial.read();

            if (c == '\n' || c == '\r') {
                if (s_len > 0) {
                    s_buf[s_len] = '\0';
                    s_len = 0;

                    if (!g_debug_mode) {
                        if (strcmp(s_buf, "debug") == 0) {
                            g_debug_mode = true;
                            Serial.println(F("\n[DEBUG] Mode active — type ? for help, exit to leave"));
                            printHelp();
                        }
                    } else {
                        parseCommand(s_buf);
                    }
                }
            } else if (s_len < 63) {
                s_buf[s_len++] = c;
            }
        }

        // ── Brake debug ───────────────────────────────────────
        if (g_debug_mode && g_debug_screen == DBG_BRAKE) {
            forceBrakeEngagement(isPhysicalBrakePressed());
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}