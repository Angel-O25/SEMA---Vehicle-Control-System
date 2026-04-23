#ifndef VCS_PINS_H
#define VCS_PINS_H

#include <Arduino.h>

// ==============================================================================
// MODULE:      VCS_Pins (ESP32-WROOM-32 38-pin)
// DESCRIPTION: Mapped for Shell Eco-marathon 2026 Autonomous Vehicle
// ==============================================================================

// --- Sensors & ADCs ---
#define PIN_HALL_A         36  // Input-only (via TXB0108)
#define PIN_HALL_B         39  // Input-only (via TXB0108)
#define PIN_HALL_SPEED     32  // Replaces legacy D10 (Hall C / Speed)
#define PIN_THROTTLE_IN    34  // ADC1_CH6 (Voltage Divider)
#define PIN_STEER_POT      35  // ADC1_CH7 (3590S)

// --- Switches (Digital Inputs) ---
#define PIN_DMS_LEFT       33  // Active HIGH
#define PIN_DMS_RIGHT      27  // Active HIGH
#define PIN_LOWBRAKE_IN    14  // Replaces legacy D8
#define PIN_LIMIT_SWITCH   13  // Brake actuator limit
#define PIN_REVERSE_IN     26  // Active LOW, 10k PCB pull-up
#define PIN_SPEED_SW_LOW   255 // DEPRECATED: Handled by Jetson/UART now
#define PIN_SPEED_SW_HIGH  255 // DEPRECATED: Handled by Jetson/UART now

// --- Actuators & Outputs ---
#define PIN_THROTTLE_OUT   25  // DAC1 output to LM358
#define PIN_STEER_PUL      18  // LEDC channel 0
#define PIN_STEER_DIR      19  // To DM542
#define PIN_STEER_ENA      23  // To DM542 (LOW=disabled)

// --- TB6612 Brake Actuator & Relays ---
#define TB6612_IN1_PIN      4 
#define TB6612_IN2_PIN      2 
#define TB6612_PWM_PIN      5 
#define BRAKE_MC_PIN       12  // Active LOW to Motor Controller
#define PIN_RELAY_STROBE   15  // 2N2222 Transistor
#define PIN_REVERSE_OUT    255 // DEPRECATED: Handled by hardware passthrough

// --- Communications ---
#define JETSON_RX_PIN      16 
#define JETSON_TX_PIN      17 
#define I2C_SDA_PIN        21 
#define I2C_SCL_PIN        22 

#endif // VCS_PINS_H