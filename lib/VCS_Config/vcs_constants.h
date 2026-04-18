#ifndef VCS_CONSTANTS_H
#define VCS_CONSTANTS_H

#include <Arduino.h>

// ==========================================
// System Architecture & Simulation
// ==========================================
#define SIMULATION_MODE 1      // 1 = Digital Twin Mode, 0 = 1500W BLDC Control Mode
#define V_LOGIC         3.3f   // Strict 3.3V Logic Level for Nano 33 BLE

// Compile-time announcement so the wrong-mode build is obvious in the IDE log
#if SIMULATION_MODE
  #pragma message ("VCS BUILD >>> SIMULATION_MODE = 1  (Digital Twin, no live motor output)")
#else
  #pragma message ("VCS BUILD >>> SIMULATION_MODE = 0  (LIVE 1500W MOTOR CONTROL - verify E-stop)")
#endif

// ==========================================
// System Frequencies & Timing (Deterministic)
// ==========================================
#define FREQ_CONTROL_HZ     1000  // Core loop frequency (1ms)
#define FREQ_STEER_CTRL_HZ  100   // Steering divider (10ms)
#define FREQ_COMM_HZ        100   // UART/State Machine frequency (10ms)
#define FREQ_UI_HZ          20    // OLED Telemetry frequency (50ms)
#define DEBOUNCE_TIME_MS    50    // Debounce delay for physical brake (50ms)

// ==========================================
// Motor & Powertrain (1500W E-Bike Setup)
// ==========================================
// IMPORTANT: Pole-pair count MUST match the actual motor, or measured RPM will
// be proportionally wrong and the Speed PI loop will mis-track by the same ratio.
// The value below is specific to the Gogo Zion 1000W/1500W hub motor used on this vehicle.
// Generic 1500W hub motors often have 23-30 pole pairs - do NOT copy this value blindly.
// Verification: spin wheel one full revolution by hand, count Hall transitions, divide by 6.
#define MOTOR_POLE_PAIRS      16    // Gogo Zion 1000W/1500W (verified on bench)
#define HALL_TRANSITIONS_REV  6     // 6 transitions per electrical cycle
#define GEAR_REDUCTION        1.0f  // Direct drive Hub Motor

// ==========================================
// Control Loop Parameters (10-bit Standard)
// ==========================================
// Speed PI (Tuned for 1500W inertia)
#define SPEED_KP              0.8f  
#define SPEED_KI              0.15f 
// WARNING: MAX_PWM_OUT=1023 assumes 10-bit PWM. The Nano 33 BLE defaults to 8-bit.
// You MUST call analogWriteResolution(10) once in setup() or every throttle write
// will silently clip to 255 and the motor will top out at ~25% of expected duty.
#define MAX_PWM_OUT           1023  // 10-bit PWM (0-3.3V) - requires analogWriteResolution(10)
#define MIN_PWM_OUT           0     // Hard Kill

// Steering PID (Tuned for Stepper Driver response)
#define STEER_KP              1.2f  
#define STEER_KI              0.05f 
#define STEER_KD              0.01f 
#define STEER_DEADZONE        5     // Ignore errors < 5 units (10-bit scale)

// ==========================================
// Communication Protocol Ranges (ANS -> VCS)
// ==========================================
#define COMM_SPEED_MIN        0     // E-bike controllers are forward-only usually
#define COMM_SPEED_MAX        3000  // Maximum Target RPM
#define COMM_STEER_LEFT       0     
#define COMM_STEER_CENTER     500   
#define COMM_STEER_RIGHT      1000  
#define COMM_BRAKE_MIN        0     
#define COMM_BRAKE_MAX        1     // Binary Brake State (0=Off, 1=On)

// ==========================================
// Physical Interface Mapping (10-bit ADC)
// ==========================================
// Nano 33 BLE ADC is linear. Thresholds are 0-1023.
#define THROTTLE_DEADBAND     50    // ~0.16V deadzone
#define THROTTLE_MIN_INPUT    150   // 0.5V signal (typical e-bike pedal start)
#define THROTTLE_MAX_INPUT    950   // 3.0V signal (typical e-bike pedal max)

// Note: window is slightly asymmetric (50..970). If steering mapping assumes
// symmetry around a center, left and right end-stops will land at different
// logical values. The steering module should either map MIN->LEFT, MAX->RIGHT
// explicitly, or use a calibrated center offset.
#define STEER_POT_MIN         50    // Physical hardware limit (Left)
#define STEER_POT_MAX         970   // Physical hardware limit (Right)

#endif // VCS_CONSTANTS_H