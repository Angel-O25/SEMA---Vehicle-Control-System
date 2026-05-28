#ifndef VCS_DEBUG_H
#define VCS_DEBUG_H

// ============================================================
//  vcs_debug.h — SIDLAK 2 VCS Debug / Calibration Mode
//  Team Wired PH0017003 | Shell Eco-marathon 2026
//
//  Enter via serial:  type "debug" and press Enter
//  Exit via serial:   type "exit" and press Enter
//
//  Navigation:        "n" = next screen, "p" = prev screen
//                     "screen N" (0-4) = jump to screen
//
//  Screens:
//    0  SPEED MANUAL   - view RPM/DAC/pedal, force throttle
//    1  SPEED AUTO     - view RPM/DAC/target, set target RPM
//    2  STEER READ     - view raw pot mV + bar graph (motor off)
//    3  STEER CTRL     - drive stepper to target pot mV
//    4  BRAKE          - view switch/limit/actuator state
//
//  Calibration commands:
//    set_full_l        - store current pot mV as STEER_POT_MIN_MV
//    set_center        - store current pot mV as STEER_POT_CENTER_MV
//    set_full_r        - store current pot mV as STEER_POT_MAX_MV
//    brake_ms <N>      - set BRAKE_RETRACT_MS
//    print_cal         - print copy-pasteable vcs_calibration.h defines
// ============================================================

#include <Arduino.h>

enum DebugScreen : uint8_t {
    DBG_SPEED_MANUAL = 0,
    DBG_SPEED_AUTO   = 1,
    DBG_STEER_READ   = 2,
    DBG_STEER_CTRL   = 3,
    DBG_BRAKE        = 4,
    DBG_SCREEN_COUNT = 5
};

// ── Global flags (read by CommLoop, DisplayLoop, throttle, steering) ──
extern volatile bool        g_debug_mode;
extern volatile DebugScreen g_debug_screen;

// ── Debug throttle override ──
// Set by "throttle N" command. -1 = not overriding (normal operation).
// Overrides only in DBG_SPEED_MANUAL or DBG_SPEED_AUTO.
extern volatile int         dbg_throttle_override_dac;

// ── Debug steer target (pot mV) ──
// Set by "steer_mv N" command. Used only in DBG_STEER_CTRL.
extern volatile uint32_t    dbg_steer_target_mv;

// ── Debug RPM target ──
// Set by "target_rpm N" command. Used only in DBG_SPEED_AUTO.
extern volatile float       dbg_target_rpm;

// ── Calibration overrides (RAM, survives until power cycle) ──
// Updated by set_full_l / set_center / set_full_r / brake_ms commands.
// print_cal dumps these as copy-pasteable vcs_calibration.h defines.
extern volatile uint32_t    dbg_pot_min_mv;
extern volatile uint32_t    dbg_pot_center_mv;
extern volatile uint32_t    dbg_pot_max_mv;
extern volatile uint32_t    dbg_retract_ms;

// ── State query helpers ──
inline bool isDebugMode()           { return g_debug_mode; }
inline bool isDebugSteerControl()   { return g_debug_mode && g_debug_screen == DBG_STEER_CTRL; }
inline bool isDebugThrottleActive() {
    return g_debug_mode &&
           dbg_throttle_override_dac >= 0 &&
           (g_debug_screen == DBG_SPEED_MANUAL || g_debug_screen == DBG_SPEED_AUTO);
}
inline bool isDebugAutoSpeed() {
    return g_debug_mode && g_debug_screen == DBG_SPEED_AUTO;
}

// ── Task entry point (create at priority 1, Core 0) ──
void DebugTask(void *pvParameters);

#endif // VCS_DEBUG_H