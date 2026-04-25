#include "vcs_throttle.h"
#include "vcs_threespeed.h"
#include "vcs_state_machine.h"
#include "vcs_constants.h"
#include "vcs_pins.h"
#include "vcs_hallsensor.h" 

#include "esp_adc_cal.h"
#include "driver/adc.h"

esp_adc_cal_characteristics_t adc_chars;

uint16_t current_throttle_adc = 0;
uint16_t current_pwm_duty = 0;

float smoothedThrottle = 0.0f;
const float emaAlphaThrottle = 0.15f; 

float measured_rpm = 0.0f;
float target_rpm = 0.0f;
float throttle_pwm_out = 0.0f;

QuickPID speedPID(&measured_rpm, &throttle_pwm_out, &target_rpm);

void initThrottle() {
    pinMode(PIN_THROTTLE_OUT, OUTPUT);
    pinMode(PIN_THROTTLE_IN, INPUT);

    // Factory eFuse calibrated ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_12); // GPIO 34
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, 1100, &adc_chars);

    speedPID.SetTunings(SPEED_KP, SPEED_KI, 0.0f);
    speedPID.SetOutputLimits(MIN_PWM_OUT, MAX_PWM_OUT);
    speedPID.SetSampleTimeUs(1000); 
    speedPID.SetMode(QuickPID::Control::manual);
}

void updateThrottle(float current_rpm_in, float target_rpm_in) {
    // --- EMA FILTER INJECTION ---
    uint32_t sum = 0;
    for (int i = 0; i < 16; i++) sum += adc1_get_raw(ADC1_CHANNEL_6);
    uint32_t pedal_mv = esp_adc_cal_raw_to_voltage(sum / 16, &adc_chars);
    int rawThrottle = map(pedal_mv, 0, 3300, 0, 1023); 

    smoothedThrottle = (emaAlphaThrottle * (float)rawThrottle)
                     + ((1.0f - emaAlphaThrottle) * smoothedThrottle);
    current_throttle_adc = (uint16_t)smoothedThrottle;

    // --- NEW: FETCH HARDWARE SPEED LIMIT ---
    float speed_multiplier = getMaxThrottleMultiplier(); 
    int dynamic_max_pwm = MIN_PWM_OUT + (int)((MAX_PWM_OUT - MIN_PWM_OUT) * speed_multiplier);

    speedPID.SetOutputLimits(MIN_PWM_OUT, dynamic_max_pwm);

    // --- 1. HARDWARE SAFETY LOCKOUT ---
    bool isBrakePressed = (digitalRead(PIN_LOWBRAKE_IN) == LOW);

    if ((currentState != AUTONOMOUS_STATE && currentState != MANUAL_STATE) || isBrakePressed) {
        current_pwm_duty = MIN_PWM_OUT;
        
        // Map the 0-1023 PI output down to 0-255 for the 8-bit true DAC
        int dac_val = map(current_pwm_duty, 0, 1023, 0, 255);
        dacWrite(25, constrain(dac_val, 0, 255));
        
        speedPID.SetMode(QuickPID::Control::manual);
        throttle_pwm_out = MIN_PWM_OUT;
        return; 
    }

    // --- 2. AUTONOMOUS CONTROL (PID) ---
    if (currentState == AUTONOMOUS_STATE) {
        if (speedPID.GetMode() == (uint8_t)QuickPID::Control::manual) {
            throttle_pwm_out = (float)current_pwm_duty;
            speedPID.SetMode(QuickPID::Control::automatic);
        }

        measured_rpm = current_rpm_in;
        target_rpm = target_rpm_in;

        if (consumeNewRPMSample() && speedPID.Compute()) {
            current_pwm_duty = (uint16_t)throttle_pwm_out;
            int dac_val = map(current_pwm_duty, 0, 1023, 0, 255);
            dacWrite(25, constrain(dac_val, 0, 255));
        }
    } 
    // --- 3. MANUAL CONTROL (Pass-Through) ---
    else if (currentState == MANUAL_STATE) {
        int mapped_pwm = MIN_PWM_OUT;
        
        if (current_throttle_adc > THROTTLE_MIN_INPUT) {
            mapped_pwm = map(current_throttle_adc, THROTTLE_MIN_INPUT, THROTTLE_MAX_INPUT, MIN_PWM_OUT, dynamic_max_pwm);
            mapped_pwm = constrain(mapped_pwm, MIN_PWM_OUT, dynamic_max_pwm);
        }
        
        current_pwm_duty = mapped_pwm;
        int dac_val = map(current_pwm_duty, 0, 1023, 0, 255);
        dacWrite(25, constrain(dac_val, 0, 255));
        
        throttle_pwm_out = mapped_pwm;
        speedPID.SetMode(QuickPID::Control::manual);
    }
}

bool isThrottlePedalPressed() {
    return (current_throttle_adc > (THROTTLE_MIN_INPUT + 15));
}