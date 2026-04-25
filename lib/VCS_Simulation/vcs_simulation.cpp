/* ==============================================================================
 * MODULE:        VCS_Simulation (v2.1) - ESP32 Native
 * RESPONSIBILITY: Digital Twin physics simulator for bench testing without the motor.
 * ============================================================================== */

#include "vcs_simulation.h"
#include "vcs_throttle.h"   // current_pwm_duty

static float sim_steer_pos = COMM_STEER_CENTER;
static float sim_motor_rpm = 0.0f;

static constexpr float SIM_DT_S      = 1.0f / (float)FREQ_STEER_CTRL_HZ;  // 0.01 s @ 100 Hz
static constexpr float SIM_RPM_TAU_S = 0.20f;                              // 1500 W inertia ~200 ms
static constexpr float SIM_RPM_ALPHA = SIM_DT_S / SIM_RPM_TAU_S;

void updateSimulatedPhysics(int pulse_freq, bool direction) {
    #if SIMULATION_MODE

    // --- 1. STEERING PHYSICS ------------------------------------------------
    if (pulse_freq > 0) {
        const float move = (float)pulse_freq * SIM_DT_S;
        if (direction) sim_steer_pos += move;
        else           sim_steer_pos -= move;
    }
    sim_steer_pos = constrain(sim_steer_pos,
                              (float)COMM_STEER_LEFT,
                              (float)COMM_STEER_RIGHT);

    // --- 2. MOTOR PHYSICS ---------------------------------------------------
    const uint16_t duty = constrain((int)current_pwm_duty, MIN_PWM_OUT, MAX_PWM_OUT);
    const float target_rpm = (float)map(duty,
                                        MIN_PWM_OUT, MAX_PWM_OUT,
                                        0, COMM_SPEED_MAX);
    sim_motor_rpm += (target_rpm - sim_motor_rpm) * SIM_RPM_ALPHA;

    #else
    (void)pulse_freq;
    (void)direction;
    #endif
}

float getSimulatedRPM()      { return sim_motor_rpm; }
float getSimulatedSteering() { return sim_steer_pos; }