#include "vcs_steering.h"

// EMA Smoothing for Steering Input
float smoothedSteering = 0.0f;
const float emaAlphaSteering = 0.15f; // 0.15 is the smoothing weight (f-suffix forces float math, not double)

// Tone cache: only s_last_freq_hz needs file scope because early-return paths
// (security override, deadband) invalidate it so the next active call always
// reprograms tone(). s_last_dir is only used in the hardware actuation block
// (#else below), so it lives there as a function-scope static — avoids an
// unused-variable warning in SIMULATION_MODE builds.
static int s_last_freq_hz = -1;

#if SIMULATION_MODE
// Simulated "Digital Twin" steering position.
// Originally this was expected to live in vcs_simulation.cpp but isn't
// actually defined there, so we maintain it locally. The actuation block
// integrates commanded step frequency * direction into this position each
// control tick, giving the PID loop a plant to close against in sim.
//
// STEPS_TO_COMM_SCALE converts stepper pulses into COMM units (0..1000
// across the full steering range). Tune this to match your mechanics:
// (COMM_STEER_RIGHT - COMM_STEER_LEFT) / (steps_per_rev * revs_lock_to_lock).
// A smaller value = slower sim; a larger value = faster sim.
static constexpr float STEPS_TO_COMM_SCALE = 0.05f;

static float    s_sim_steer_pos_f = (float)COMM_STEER_CENTER; // float accumulator
static uint16_t sim_steer_pos     = COMM_STEER_CENTER;        // read by getMeasuredSteering()
#endif

// PID Variables
float setpoint, input, output;
QuickPID steeringPID(&input, &output, &setpoint);

// Sample time for 100 Hz loop
const float Ts_s = 1.0f / FREQ_STEER_CTRL_HZ;

void initSteering() {
    pinMode(PIN_STEER_POT, INPUT);
    pinMode(PIN_STEER_DIR, OUTPUT);
    pinMode(PIN_STEER_ENA, OUTPUT);
    pinMode(PIN_STEER_PUL, OUTPUT);
    
    // Using 10-bit resolution to sync with the rest of the Nano 33 BLE system
    analogReadResolution(10); 

    // Initial state: Disabled/Free
    // (Note: On standard TB6600/DM542 stepper drivers, ENA HIGH = Disabled. 
    // Leave this LOW if your specific driver requires LOW to disable).
    digitalWrite(PIN_STEER_ENA, HIGH); 
    noTone(PIN_STEER_PUL); // Ensure stepper is not pulsing

    // QuickPID Configuration
    steeringPID.SetTunings(STEER_KP, STEER_KI, STEER_KD);
    steeringPID.SetSampleTimeUs(Ts_s * 1000000);
    steeringPID.SetOutputLimits(-255.0f, 255.0f);      // Negative = Left, Positive = Right
    steeringPID.SetMode(QuickPID::Control::automatic);
}

uint16_t getMeasuredSteering() {
    uint16_t current_pos;

    // --- 1. DATA ACQUISITION ---
#if SIMULATION_MODE
    // In Simulation, we use the "Digital Twin" position
    current_pos = (uint16_t)constrain(sim_steer_pos, COMM_STEER_LEFT, COMM_STEER_RIGHT);
#else
    // --- EMA FILTER INJECTION ---
    // Read the raw noisy pin
    int rawSteering = analogRead(PIN_STEER_POT);

    // DISCONNECTION CHECK (Hardware Security)
    // Run on RAW value BEFORE the EMA, otherwise a glitch gets smoothed
    // into the valid range and safety detection fails.
    if (rawSteering < 12 || rawSteering > 1010) {
        // Also reset the filter state so we don't carry a stale value forward
        // when the pot is reconnected.
        smoothedSteering = (float)rawSteering;
        return COMM_STEER_CENTER;
    }

    // Apply the EMA math to filter out motor spikes.
    // All literals are float ('f' suffix) to avoid double-precision promotion,
    // which is software-emulated on the Cortex-M4F (~10x slower than float).
    smoothedSteering = (emaAlphaSteering * (float)rawSteering)
                     + ((1.0f - emaAlphaSteering) * smoothedSteering);

    // Assign the clean, smoothed value to raw_adc for downstream mapping
    int raw_adc = (int)smoothedSteering;

    // MAPPING PHYSICAL ADC TO COMM SCALE (0-1000)
    int mapped_pos = map(raw_adc, 0, 1023, COMM_STEER_LEFT, COMM_STEER_RIGHT);
    current_pos = (uint16_t)constrain(mapped_pos, COMM_STEER_LEFT, COMM_STEER_RIGHT);
#endif

    // --- 2. VELOCITY CHECK (The "Slew Rate" Fix) ---
    static uint16_t last_pos = COMM_STEER_CENTER;

    // BUG FIX: Both operands were uint16_t, so (current_pos - last_pos) wrapped
    // modulo 65536 when current_pos < last_pos. abs() of an already-wrapped
    // unsigned value is a no-op, so the branch fired in the wrong direction.
    // Cast to signed before subtracting.
    int32_t delta = (int32_t)current_pos - (int32_t)last_pos;
    if (delta > 50) {
        current_pos = last_pos + 50;
    } else if (delta < -50) {
        current_pos = last_pos - 50;
    }

    last_pos = current_pos;
    return current_pos;
}

void updateSteeringPID(uint16_t target_position, bool is_automatic) {
    input = (float)getMeasuredSteering();
    setpoint = (float)target_position;

    // --- SECURITY OVERRIDE ---
    if (!is_automatic || currentState == FAULT_STATE || currentState == ESTOP_STATE) {
        digitalWrite(PIN_STEER_ENA, HIGH); // Disable motor (verify this matches your driver spec!)
        noTone(PIN_STEER_PUL);
        s_last_freq_hz = -1; // Invalidate tone cache so next active call reprograms
        return;
    }

    steeringPID.Compute();

    // Deadband check
    // Use fabsf() for floats. Arduino's abs() is a macro that double-evaluates
    // its argument (so "setpoint - input" would be computed twice).
    if (fabsf(setpoint - input) < STEER_DEADZONE) {
        noTone(PIN_STEER_PUL);
        digitalWrite(PIN_STEER_ENA, LOW); // Enable motor holding torque
        s_last_freq_hz = -1; // Invalidate tone cache so next active call reprograms
        return;
    }

   // --- HARDWARE ACTUATION ---
    digitalWrite(PIN_STEER_ENA, LOW); // Ensure motor is Locked/Enabled before moving!

    bool dir = (output > 0);
    digitalWrite(PIN_STEER_DIR, dir ? HIGH : LOW);

    // fabsf() for float. Also floor to >=1 so map() never produces 0 Hz,
    // which would cause tone(pin, 0) to stop the pulse entirely.
    float effort = fabsf(output);
    if (effort < 1.0f) effort = 1.0f;
    int step_frequency_hz = map((long)effort, 0, 255, 50, 2000);

#if SIMULATION_MODE
    // Delegate raw physics (e.g. motor RPM, inertia) to the sim module.
    updateSimulatedPhysics(step_frequency_hz, dir);

    // Integrate the stepper command into the local Digital Twin position.
    // Over one control tick (Ts_s seconds), we emit step_frequency_hz * Ts_s
    // pulses; each pulse moves the mechanism by STEPS_TO_COMM_SCALE COMM units.
    const float delta = (float)step_frequency_hz * Ts_s * STEPS_TO_COMM_SCALE;
    s_sim_steer_pos_f += dir ? +delta : -delta;

    // Clamp to the valid COMM range so the sim position can't drift out of
    // bounds if the PID is saturated for an extended period.
    if (s_sim_steer_pos_f < (float)COMM_STEER_LEFT)  s_sim_steer_pos_f = (float)COMM_STEER_LEFT;
    if (s_sim_steer_pos_f > (float)COMM_STEER_RIGHT) s_sim_steer_pos_f = (float)COMM_STEER_RIGHT;
    sim_steer_pos = (uint16_t)s_sim_steer_pos_f;
#else
    // Only reprogram tone() when frequency changes meaningfully. On Mbed
    // Nano 33 BLE, tone() reconfigures a Ticker/PWM on every call, which
    // introduces jitter and isn't free at 100 Hz control rate.
    static bool s_last_dir = false; // function-scope: only used in this branch
    if (abs(step_frequency_hz - s_last_freq_hz) > 5 || dir != s_last_dir) {
        tone(PIN_STEER_PUL, step_frequency_hz);
        s_last_freq_hz = step_frequency_hz;
        s_last_dir = dir;
    }
#endif
}