#include "vcs_steering.h"

#if defined(ESP32_VCS)
    #include "esp_adc_cal.h"
    #include "driver/adc.h"
    extern esp_adc_cal_characteristics_t adc_chars; // Pull the calibration profile initialized in vcs_throttle
#endif

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

    #if defined(ESP32_VCS)
        // Explicitly initialize LEDC Channel 0 to prevent RTOS panics
        ledcSetup(0, 1000, 8); // Channel 0, 1kHz baseline, 8-bit res
        ledcAttachPin(PIN_STEER_PUL, 0);
    #else
        pinMode(PIN_STEER_PUL, OUTPUT);
    #endif
    
    #if defined(ESP32_VCS)
        // Configure ADC1 Channel 7 (GPIO 35) for the Steering Potentiometer
        adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_12);
    #elif !defined(__AVR__)
        analogReadResolution(10); 
    #endif

    digitalWrite(PIN_STEER_ENA, HIGH); 
    
    #if defined(ESP32_VCS)
        ledcWrite(0, 0); // 0% duty cycle = no pulses
    #else
        noTone(PIN_STEER_PUL); 
    #endif

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
    #if defined(ESP32_VCS)
        // Take a 16-sample hardware average for maximum stability
        uint32_t sum = 0;
        for (int i = 0; i < 16; i++) sum += adc1_get_raw(ADC1_CHANNEL_7);
        uint32_t pot_mv = esp_adc_cal_raw_to_voltage(sum / 16, &adc_chars);
        
        // Map 0-3300mV back to the 0-1023 scale the legacy EMA math expects
        int rawSteering = map(pot_mv, 0, 3300, 0, 1023);
    #else
        int rawSteering = analogRead(PIN_STEER_POT);
    #endif

    // DISCONNECTION CHECK (Hardware Security)

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
        digitalWrite(PIN_STEER_ENA, HIGH); 
        
        #if defined(ESP32_VCS)
            ledcWrite(0, 0); // Hardware stop
        #else
            noTone(PIN_STEER_PUL);
        #endif
        
        s_last_freq_hz = -1; 
        return;
    }

    steeringPID.Compute();

    // Deadband check
    // Use fabsf() for floats. Arduino's abs() is a macro that double-evaluates
    // its argument (so "setpoint - input" would be computed twice).
    // Deadband check
    if (fabsf(setpoint - input) < STEER_DEADZONE) {
        #if defined(ESP32_VCS)
            ledcWrite(0, 0); // Stop pulses, hold position
        #else
            noTone(PIN_STEER_PUL);
        #endif
        
        digitalWrite(PIN_STEER_ENA, LOW); // Enable motor holding torque
        s_last_freq_hz = -1; 
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
    static bool s_last_dir = false; 
    if (abs(step_frequency_hz - s_last_freq_hz) > 5 || dir != s_last_dir) {
        #if defined(ESP32_VCS)
            // ESP32 v2.0.17 LEDC Hardware Timer Implementation
            ledcSetup(0, step_frequency_hz, 8); // Channel 0, frequency, 8-bit res
            ledcAttachPin(PIN_STEER_PUL, 0);
            ledcWrite(0, 128); // 50% duty cycle for the pulse
        #else
            tone(PIN_STEER_PUL, step_frequency_hz);
        #endif
        
        s_last_freq_hz = step_frequency_hz;
        s_last_dir = dir;
    }
#endif
}