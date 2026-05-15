#include "vcs_steering.h"
#include "esp_adc_cal.h"
#include "driver/adc.h"
#include "vcs_calibration.h"
#include "vcs_constants.h"

extern esp_adc_cal_characteristics_t adc_chars;

float smoothedSteering = 0.0f;


static int s_last_freq_hz = -1;
static bool s_last_dir = false;
bool dir;


#if SIMULATION_MODE
static constexpr float STEPS_TO_COMM_SCALE = 0.05f;
static float    s_sim_steer_pos_f = (float)COMM_STEER_CENTER; 
static uint16_t sim_steer_pos     = COMM_STEER_CENTER;        
#endif

// FIX 9: file-scope statics to lock down PID I/O variables
static float setpoint = 0.0f;
static float input    = 0.0f;
static float output   = 0.0f;

QuickPID steeringPID(&input, &output, &setpoint);

const float Ts_s = 1.0f / FREQ_STEER_CTRL_HZ;

void initSteering() {
    pinMode(PIN_STEER_POT, INPUT);
    pinMode(PIN_STEER_DIR, OUTPUT);
    pinMode(PIN_STEER_ENA, OUTPUT);

    // Initialize LEDC Channel 0 once. Frequency will be changed live
    // via ledcChangeFrequency() — never re-attach the pin afterward.
    ledcSetup(0, 1000, 8);
    ledcAttachPin(PIN_STEER_PUL, 0);
    
    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_12);

    digitalWrite(PIN_STEER_ENA, HIGH); 
    ledcWrite(0, 0);

    steeringPID.SetTunings(STEER_KP, STEER_KI, STEER_KD);
    steeringPID.SetSampleTimeUs(Ts_s * 1000000);   // 10000us @ 100Hz — already correct
    steeringPID.SetOutputLimits(-255.0f, 255.0f);      
    steeringPID.SetMode(QuickPID::Control::automatic);
}

uint16_t getMeasuredSteering() {
    uint16_t current_pos;

    // --- 1. DATA ACQUISITION ---
#if SIMULATION_MODE
    current_pos = (uint16_t)constrain(sim_steer_pos, COMM_STEER_LEFT, COMM_STEER_RIGHT);
#else
    uint32_t sum = 0;
    for (int i = 0; i < 16; i++) sum += adc1_get_raw(ADC1_CHANNEL_7);
    uint32_t pot_mv = esp_adc_cal_raw_to_voltage(sum / 16, &adc_chars);
    int rawSteering = map(pot_mv, 0, 3300, 0, 1023);

    // DISCONNECTION CHECK
    if (rawSteering < 12 || rawSteering > 1010) {
        smoothedSteering = (float)rawSteering;
        return COMM_STEER_CENTER;
    }

    smoothedSteering = (STEER_EMA_ALPHA * (float)rawSteering)
                     + ((1.0f - STEER_EMA_ALPHA) * smoothedSteering);

    int raw_adc = (int)smoothedSteering;
    int mapped_pos = map(raw_adc, 0, 1023, COMM_STEER_LEFT, COMM_STEER_RIGHT);
    current_pos = (uint16_t)constrain(mapped_pos, COMM_STEER_LEFT, COMM_STEER_RIGHT);
#endif

    // --- 2. VELOCITY CHECK (Slew Rate Fix) ---
    // NOTE: this slew limit must NOT be applied if the function is
    // called more than once per control tick (e.g., by telemetry
    // and the PID loop both). Cache the value at the top of the
    // tick and reuse it. See ControlTask integration notes.
    static uint16_t last_pos = COMM_STEER_CENTER;

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
    input    = (float)getMeasuredSteering();
    setpoint = (float)target_position;

    // --- SAFETY LOCKOUT (renamed from misleading "SECURITY OVERRIDE") ---
    if (!is_automatic) {
        digitalWrite(PIN_STEER_ENA, HIGH); 
        ledcWrite(0, 0);
        s_last_freq_hz = -1; 

        if (fabsf(output) < 5.0f) {
        dir = s_last_dir;
    } else {
        dir = (output > 0);
    }

    // Before changing direction in updateSteeringPID():
    if (dir != s_last_dir) {
        digitalWrite(PIN_STEER_DIR, dir ? HIGH : LOW);
        delayMicroseconds(STEER_DM542_DIR_SETUP_US);   // 5µs from cal
    }

        // FIX 2: clear PID state on transition out of automatic.
        // Without this, the integral persists into the next autonomous
        // entry and the steering motor jolts.
        if (steeringPID.GetMode() != (uint8_t)QuickPID::Control::manual) {
            steeringPID.SetMode(QuickPID::Control::manual);
            steeringPID.Initialize();
        }
        return;
    }

    // Re-enter automatic cleanly when conditions allow
    if (steeringPID.GetMode() == (uint8_t)QuickPID::Control::manual) {
        steeringPID.SetMode(QuickPID::Control::automatic);
    }

    steeringPID.Compute();

    // Deadband check
    if (fabsf(setpoint - input) < STEER_DEADZONE) {
        ledcWrite(0, 0);
        digitalWrite(PIN_STEER_ENA, LOW);
        s_last_freq_hz = -1; 
        return;
    }

   // --- HARDWARE ACTUATION ---
    digitalWrite(PIN_STEER_ENA, LOW); 

    // FIX 10: anti-chatter band on direction flip — prevents rapid
    // direction reversals when output crosses zero.
    static bool s_last_dir = false;
    bool dir;
    if (fabsf(output) < 5.0f) {
        dir = s_last_dir;
    } else {
        dir = (output > 0);
    }
    digitalWrite(PIN_STEER_DIR, dir ? HIGH : LOW);

    float effort = fabsf(output);
    if (effort < 1.0f) effort = 1.0f;
    int step_frequency_hz = map((long)effort, 0, 255, 50, 2000);

#if SIMULATION_MODE
    updateSimulatedPhysics(step_frequency_hz, dir);

    const float delta = (float)step_frequency_hz * Ts_s * STEPS_TO_COMM_SCALE;
    s_sim_steer_pos_f += dir ? +delta : -delta;

    if (s_sim_steer_pos_f < (float)COMM_STEER_LEFT)  s_sim_steer_pos_f = (float)COMM_STEER_LEFT;
    if (s_sim_steer_pos_f > (float)COMM_STEER_RIGHT) s_sim_steer_pos_f = (float)COMM_STEER_RIGHT;
    sim_steer_pos = (uint16_t)s_sim_steer_pos_f;
#else
    if (abs(step_frequency_hz - s_last_freq_hz) > 5 || dir != s_last_dir) {
        // FIX 7: use ledcChangeFrequency() instead of ledcSetup() +
        // ledcAttachPin() — the latter causes a brief glitch on the
        // pin which can drop step pulses. ChangeFrequency is glitch-free.
        ledcChangeFrequency(0, step_frequency_hz, 8);
        ledcWrite(0, 128); // 50% duty
        
        s_last_freq_hz = step_frequency_hz;
        s_last_dir = dir;
    }
#endif
}