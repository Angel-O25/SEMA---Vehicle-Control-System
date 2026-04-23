#include "vcs_throttle.h"
#include "vcs_threespeed.h"
#include "vcs_state_machine.h"
#include "vcs_constants.h"
#include "vcs_pins.h"
#include "vcs_hallsensor.h" // For consumeNewRPMSample()

#if defined(ESP32_VCS)
    #include "esp_adc_cal.h"
    #include "driver/adc.h"
    esp_adc_cal_characteristics_t adc_chars;
#endif

// Global Telemetry Variables
uint16_t current_throttle_adc = 0;
uint16_t current_pwm_duty = 0;

// EMA Smoothing for Throttle Input
float smoothedThrottle = 0.0f;
const float emaAlphaThrottle = 0.15f; // 0.15 is the smoothing weight (f-suffix forces float math, not double)

// PID Variables
float measured_rpm = 0.0f;
float target_rpm = 0.0f;
float throttle_pwm_out = 0.0f;

// Initialize QuickPID
QuickPID speedPID(&measured_rpm, &throttle_pwm_out, &target_rpm);

void initThrottle() {
    pinMode(PIN_THROTTLE_OUT, OUTPUT);
    pinMode(PIN_THROTTLE_IN, INPUT);

    #if defined(ESP32_VCS)
        // Factory eFuse calibrated ADC
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_12); // GPIO 34
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, 1100, &adc_chars);
    #endif

    // Sync Nano 33 BLE hardware to the 10-bit standard (0-1023).
    // The Arduino Nano 33 BLE core does NOT define "NANO_33_BLE" — the actual
    // predefined macros are ARDUINO_ARDUINO_NANO33BLE and/or ARDUINO_ARCH_MBED_NANO.
    // With the old ifdef, these two lines were silently skipped, leaving the
    // board at default 10-bit read / 8-bit write — which disagreed with
    // MAX_PWM_OUT assumptions elsewhere.
    #if defined(ARDUINO_ARDUINO_NANO33BLE) || defined(ARDUINO_ARCH_MBED_NANO) || defined(NANO_33_BLE)
    analogReadResolution(10);
    analogWriteResolution(10);
    #endif

    // Apply constants from vcs_constants.h
    speedPID.SetTunings(SPEED_KP, SPEED_KI, 0.0f);
    speedPID.SetOutputLimits(MIN_PWM_OUT, MAX_PWM_OUT);
    
    // CRITICAL: Sync QuickPID to our 1kHz Mbed ControlTask thread
    speedPID.SetSampleTimeUs(1000); 
    
    // Start in manual to prevent accidental windup on boot
    speedPID.SetMode(QuickPID::Control::manual);
}

void updateThrottle(float current_rpm_in, float target_rpm_in) {
    // --- EMA FILTER INJECTION ---
    // All literals are float ('f' suffix) to avoid double-precision promotion,
    // which is software-emulated on the Cortex-M4F (~10x slower than float).
    // --- EMA FILTER INJECTION ---
    #if defined(ESP32_VCS)
        uint32_t sum = 0;
        for (int i = 0; i < 16; i++) sum += adc1_get_raw(ADC1_CHANNEL_6);
        uint32_t pedal_mv = esp_adc_cal_raw_to_voltage(sum / 16, &adc_chars);
        // Map 0-3300mV back to the 0-1023 scale your legacy EMA math expects
        int rawThrottle = map(pedal_mv, 0, 3300, 0, 1023); 
    #else
        int rawThrottle = analogRead(PIN_THROTTLE_IN);
    #endif

    smoothedThrottle = (emaAlphaThrottle * (float)rawThrottle)
                     + ((1.0f - emaAlphaThrottle) * smoothedThrottle);
    current_throttle_adc = (uint16_t)smoothedThrottle;

    // --- NEW: FETCH HARDWARE SPEED LIMIT ---
    float speed_multiplier = getMaxThrottleMultiplier(); 
    
    // Calculate the dynamic max PWM based on the 3-position switch
    // E.g., If MAX_PWM is 1023 and multiplier is 0.6, dynamic max is ~613.
    int dynamic_max_pwm = MIN_PWM_OUT + (int)((MAX_PWM_OUT - MIN_PWM_OUT) * speed_multiplier);

    // Update PID limits dynamically so the integral windup respects the physical switch
    speedPID.SetOutputLimits(MIN_PWM_OUT, dynamic_max_pwm);

    // --- 1. HARDWARE SAFETY LOCKOUT ---
    bool isBrakePressed = (digitalRead(PIN_LOWBRAKE_IN) == LOW);

    // If the state machine says we shouldn't be driving OR driver hits the brake
    if ((currentState != AUTONOMOUS_STATE && currentState != MANUAL_STATE) || isBrakePressed) {
        current_pwm_duty = MIN_PWM_OUT;
        #if defined(ESP32_VCS)
            // Map the 0-1023 PI output down to 0-255 for the 8-bit true DAC
            int dac_val = map(current_pwm_duty, 0, 1023, 0, 255);
            dacWrite(25, constrain(dac_val, 0, 255));
        #else
            analogWrite(PIN_THROTTLE_OUT, current_pwm_duty);
        #endif
        
        speedPID.SetMode(QuickPID::Control::manual);
        throttle_pwm_out = MIN_PWM_OUT;
        return; 
    }

    // --- 2. AUTONOMOUS CONTROL (PID) ---
    if (currentState == AUTONOMOUS_STATE) {
        if (speedPID.GetMode() == (uint8_t)QuickPID::Control::manual) {
            // BUMPLESS TRANSFER (MANUAL -> AUTO):
            // Seed the PID output with the CURRENT PWM before flipping to
            // automatic. Otherwise QuickPID initializes its internal bumpless
            // state against whatever stale value throttle_pwm_out held
            // (typically 0 after a brake event), producing a step-down.
            throttle_pwm_out = (float)current_pwm_duty;
            speedPID.SetMode(QuickPID::Control::automatic);
        }

        measured_rpm = current_rpm_in;
        target_rpm = target_rpm_in;

        // Gate Compute() on a fresh Hall sample. The control task ticks at
        // 1 kHz but the Hall window is 100 ms, so 99 of every 100 ticks
        // would otherwise run the integrator on identical (stale) error,
        // accelerating windup and producing a sluggish Ki response.
        if (consumeNewRPMSample() && speedPID.Compute()) {
            current_pwm_duty = (uint16_t)throttle_pwm_out;
            #if defined(ESP32_VCS)
            // Map the 0-1023 PI output down to 0-255 for the 8-bit true DAC
            int dac_val = map(current_pwm_duty, 0, 1023, 0, 255);
            dacWrite(25, constrain(dac_val, 0, 255));
        #else
            analogWrite(PIN_THROTTLE_OUT, current_pwm_duty);
        #endif
        }
    } 
    // --- 3. MANUAL CONTROL (Pass-Through) ---
    else if (currentState == MANUAL_STATE) {
        int mapped_pwm = MIN_PWM_OUT;
        
        if (current_throttle_adc > THROTTLE_MIN_INPUT) {
            // Use the dynamic_max_pwm instead of the absolute MAX_PWM_OUT
            mapped_pwm = map(current_throttle_adc, THROTTLE_MIN_INPUT, THROTTLE_MAX_INPUT, MIN_PWM_OUT, dynamic_max_pwm);
            mapped_pwm = constrain(mapped_pwm, MIN_PWM_OUT, dynamic_max_pwm);
        }
        
        current_pwm_duty = mapped_pwm;
        #if defined(ESP32_VCS)
            // Map the 0-1023 PI output down to 0-255 for the 8-bit true DAC
            int dac_val = map(current_pwm_duty, 0, 1023, 0, 255);
            dacWrite(25, constrain(dac_val, 0, 255));
        #else
            analogWrite(PIN_THROTTLE_OUT, current_pwm_duty);
        #endif
        
        // BUMPLESS TRANSFER: 
        throttle_pwm_out = mapped_pwm;
        speedPID.SetMode(QuickPID::Control::manual);
    }
}

// [ADDED] Helper function for the State Machine to check for manual override
bool isThrottlePedalPressed() {
    // Add a small safety margin above the minimum to prevent noise from dropping auto mode
    return (current_throttle_adc > (THROTTLE_MIN_INPUT + 15));
}