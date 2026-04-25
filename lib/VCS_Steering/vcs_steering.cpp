#include "vcs_steering.h"
#include "esp_adc_cal.h"
#include "driver/adc.h"

extern esp_adc_cal_characteristics_t adc_chars; // Pull the calibration profile initialized in vcs_throttle

// EMA Smoothing for Steering Input
float smoothedSteering = 0.0f;
const float emaAlphaSteering = 0.15f; 

static int s_last_freq_hz = -1;

#if SIMULATION_MODE
static constexpr float STEPS_TO_COMM_SCALE = 0.05f;
static float    s_sim_steer_pos_f = (float)COMM_STEER_CENTER; 
static uint16_t sim_steer_pos     = COMM_STEER_CENTER;        
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

    // Explicitly initialize LEDC Channel 0 to prevent RTOS panics
    ledcSetup(0, 1000, 8); // Channel 0, 1kHz baseline, 8-bit res
    ledcAttachPin(PIN_STEER_PUL, 0);
    
    // Configure ADC1 Channel 7 (GPIO 35) for the Steering Potentiometer
    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_12);

    digitalWrite(PIN_STEER_ENA, HIGH); 
    ledcWrite(0, 0); // 0% duty cycle = no pulses

    // QuickPID Configuration
    steeringPID.SetTunings(STEER_KP, STEER_KI, STEER_KD);
    steeringPID.SetSampleTimeUs(Ts_s * 1000000);
    steeringPID.SetOutputLimits(-255.0f, 255.0f);      
    steeringPID.SetMode(QuickPID::Control::automatic);
}

uint16_t getMeasuredSteering() {
    uint16_t current_pos;

    // --- 1. DATA ACQUISITION ---
#if SIMULATION_MODE
    current_pos = (uint16_t)constrain(sim_steer_pos, COMM_STEER_LEFT, COMM_STEER_RIGHT);
#else
    // Take a 16-sample hardware average for maximum stability
    uint32_t sum = 0;
    for (int i = 0; i < 16; i++) sum += adc1_get_raw(ADC1_CHANNEL_7);
    uint32_t pot_mv = esp_adc_cal_raw_to_voltage(sum / 16, &adc_chars);
    
    // Map 0-3300mV back to the 0-1023 scale the legacy EMA math expects
    int rawSteering = map(pot_mv, 0, 3300, 0, 1023);

    // DISCONNECTION CHECK (Hardware Security)
    if (rawSteering < 12 || rawSteering > 1010) {
        smoothedSteering = (float)rawSteering;
        return COMM_STEER_CENTER;
    }

    smoothedSteering = (emaAlphaSteering * (float)rawSteering)
                     + ((1.0f - emaAlphaSteering) * smoothedSteering);

    int raw_adc = (int)smoothedSteering;

    // MAPPING PHYSICAL ADC TO COMM SCALE (0-1000)
    int mapped_pos = map(raw_adc, 0, 1023, COMM_STEER_LEFT, COMM_STEER_RIGHT);
    current_pos = (uint16_t)constrain(mapped_pos, COMM_STEER_LEFT, COMM_STEER_RIGHT);
#endif

    // --- 2. VELOCITY CHECK (The "Slew Rate" Fix) ---
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
    input = (float)getMeasuredSteering();
    setpoint = (float)target_position;

    // --- SECURITY OVERRIDE ---
    if (!is_automatic || currentState == FAULT_STATE) {
        digitalWrite(PIN_STEER_ENA, HIGH); 
        ledcWrite(0, 0); // Hardware stop
        s_last_freq_hz = -1; 
        return;
    }

    steeringPID.Compute();

    // Deadband check
    if (fabsf(setpoint - input) < STEER_DEADZONE) {
        ledcWrite(0, 0); // Stop pulses, hold position
        digitalWrite(PIN_STEER_ENA, LOW); // Enable motor holding torque
        s_last_freq_hz = -1; 
        return;
    }

   // --- HARDWARE ACTUATION ---
    digitalWrite(PIN_STEER_ENA, LOW); 

    bool dir = (output > 0);
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
    static bool s_last_dir = false; 
    if (abs(step_frequency_hz - s_last_freq_hz) > 5 || dir != s_last_dir) {
        // ESP32 v2.0.17 LEDC Hardware Timer Implementation
        ledcSetup(0, step_frequency_hz, 8); 
        ledcAttachPin(PIN_STEER_PUL, 0);
        ledcWrite(0, 128); // 50% duty cycle for the pulse
        
        s_last_freq_hz = step_frequency_hz;
        s_last_dir = dir;
    }
#endif
}