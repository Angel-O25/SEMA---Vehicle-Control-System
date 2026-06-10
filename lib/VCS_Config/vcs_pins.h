#ifndef VCS_PINS_H
#define VCS_PINS_H

#include <Arduino.h>

// ==============================================================================
// MODULE:      VCS_Pins (ESP32-WROOM-32, 38-pin DevKit)
// PROJECT:     SIDLAK 2 — Shell Eco-marathon 2026
// ==============================================================================

// --- Sensors & ADCs ---
#define PIN_HALL_A         36   // Input-only silicon (no pull-up/down possible)
#define PIN_HALL_B         39   // Input-only silicon (no pull-up/down possible)
#define PIN_HALL_SPEED     32   // Hall C — normal GPIO, INPUT_PULLDOWN in firmware
#define PIN_THROTTLE_IN    34   // ADC1_CH6 — pedal via 5V→3.3V divider
#define PIN_STEER_POT      35   // ADC1_CH7 — 3590S steering pot (3.3V powered)

// --- Switches (Digital Inputs) ---
#define PIN_DMS_LEFT       33   // Active HIGH, INPUT_PULLDOWN in firmware
#define PIN_DMS_RIGHT      27   // Active HIGH, INPUT_PULLDOWN in firmware
#define PIN_LOWBRAKE_IN    14   // Brake switch, ACTIVE LOW (pressed=LOW), INPUT_PULLDOWN
#define PIN_LIMIT_SWITCH   13   // Brake actuator end-stop, active HIGH, INPUT_PULLDOWN
#define PIN_REVERSE_IN     26   // Active LOW (latching toggle); 10k pull-up on PCB —
                                // configure as plain INPUT, NOT INPUT_PULLUP/DOWN

// --- Actuators & Outputs ---
#define PIN_THROTTLE_OUT   25   // DAC1 (0–3.3V) → LM358 → 0–4.7V to motor controller
#define PIN_STEER_PUL      18   // LEDC ch8 → BC547 → DQ860MA PUL-  (HIGH=step, LOW=idle)
#define PIN_STEER_DIR      19   // BC547 → DQ860MA DIR-              (HIGH=right, LOW=left)
#define PIN_STEER_ENA      23   // BC547 → DQ860MA ENA-              (LOW=enabled, HIGH=disabled)


// --- TB6612 Brake Actuator (channels paralleled) ---
#define TB6612_IN1_PIN      4
#define TB6612_IN2_PIN      2
#define TB6612_PWM_PIN      5

// --- Misc Outputs ---
#define BRAKE_MC_PIN       12   // Brake signal to motor controller (active LOW)
#define PIN_RELAY_STROBE   15   // 2N2222 → relay coil  (HIGH = relay ON)

// --- Communications ---
#define JETSON_RX_PIN      16
#define JETSON_TX_PIN      17
#define I2C_SDA_PIN        21
#define I2C_SCL_PIN        22

#endif // VCS_PINS_H