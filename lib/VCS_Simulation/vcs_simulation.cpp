/* ==============================================================================
 * MODULE:        VCS_Simulation (v1.3)
 * RESPONSIBILITY: Physics Emulation & Signal Testing (Uplink/Downlink)
 * ------------------------------------------------------------------------------
 * SETUP INSTRUCTIONS FOR SIMULATION MODE (NANO 33 BLE):
 * * 1. PHYSICAL CONNECTIONS (3.3V Logic Only):
 * - Nano 33 BLE (TX / Pin D1)  --->  RPi (RXD / Pin 10)
 * - Nano 33 BLE (RX / Pin D0)  --->  RPi (TXD / Pin 8)
 * - COMMON GROUND: Connect Nano GND to RPi GND (Critical for UART stability).
 * * * 2. POWER SAFETY:
 * - DISCONNECT the 60V Main Battery. Simulation runs on USB power only.
 * - DISCONNECT the 1500W Motor Phase wires if the controller is powered.
 * * * 3. PERIPHERALS NEEDED:
 * - DMS Button: Connect to PIN_DMS_BUTTON and GND.
 * - Brake Switch: Connect to PIN_LOWBRAKE_IN and GND.
 * - Note: In SIMULATION_MODE, the physical Steering Pot and Hall Sensors 
 * are ignored, but the safety overrides remain strictly active.
 * * * 4. SOFTWARE ACTIVATION:
 * - Set '#define SIMULATION_MODE 1' in 'vcs_constants.h'.
 * ============================================================================== */

#include "vcs_simulation.h"
#include "vcs_state_machine.h"
#include "vcs_throttle.h" // Gives us access to current_pwm_duty

// Global Simulated Variables (Starting at dead center and 0 RPM)
float sim_steer_pos = COMM_STEER_CENTER; 
float sim_motor_rpm = 0.0f;

/**
 * @brief Updates the "Digital Twin" of the vehicle physics.
 * @param pulse_freq Current frequency sent to the stepper (Hz)
 * @param direction  True for Right, False for Left
 * @param throttle_pwm (Ignored in v1.3 - Reads globally for accuracy)
 */
void updateSimulatedPhysics(int pulse_freq, bool direction, uint16_t throttle_pwm) {
    #if SIMULATION_MODE
    
    // --- 1. STEERING PHYSICS SIMULATION ---
    // Scale: 2000Hz = Max speed. 0.001 is the integration constant for 10ms loops.
    if (pulse_freq > 0) {
        float move = (float)pulse_freq * 0.001f;
        if (direction) {
            sim_steer_pos += move;
        } else {
            sim_steer_pos -= move;
        }
    }
    sim_steer_pos = constrain(sim_steer_pos, COMM_STEER_LEFT, COMM_STEER_RIGHT);

    // --- 2. MOTOR PHYSICS SIMULATION ---
    // Read the exact 10-bit PWM duty cycle being outputted by vcs_throttle
    // Map it from 0-1023 up to the maximum RPM allowed by the protocol
    float target_rpm = map(current_pwm_duty, MIN_PWM_OUT, MAX_PWM_OUT, 0, COMM_SPEED_MAX); 
    
    // Simple inertia model: RPM follows throttle with a slight delay.
    // At 1500W, the real car has mass. This 5% approach simulates that mechanical lag.
    sim_motor_rpm += (target_rpm - sim_motor_rpm) * 0.05f; 

    #endif
}

float getSimulatedRPM() { 
    return sim_motor_rpm; 
}

float getSimulatedSteering() { 
    return sim_steer_pos; 
}