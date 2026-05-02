# Project Files Export

Export time: 4/30/2026, 10:23:27 PM

Source directory: `lib`

Output file: `project-files.md`

## Directory Structure

```
lib
├── VCS_Actuators
│   ├── vcs_deadman.cpp
│   ├── vcs_deadman.h
│   ├── vcs_lowbrake.cpp
│   ├── vcs_lowbrake.h
│   ├── vcs_relays.cpp
│   ├── vcs_relays.h
│   ├── vcs_throttle.cpp
│   └── vcs_throttle.h
├── VCS_Comm
│   ├── vcs_uart.cpp
│   └── vcs_uart.h
├── VCS_Config
│   ├── vcs_constants.h
│   └── vcs_pins.h
├── VCS_Display
│   ├── vcs_display.cpp
│   └── vcs_display.h
├── VCS_Hall
│   ├── vcs_hallsensor.cpp
│   └── vcs_hallsensor.h
├── VCS_Simulation
│   ├── vcs_simulation.cpp
│   ├── vcs_simulation.h
│   ├── vcs_web.cpp
│   └── vcs_web.h
├── VCS_Speed
│   ├── vcs_reverse.cpp
│   ├── vcs_reverse.h
│   ├── vcs_threespeed.cpp
│   └── vcs_threespeed.h
├── VCS_Steering
│   ├── vcs_steering.cpp
│   └── vcs_steering.h
├── VCS_System
│   ├── vcs_state_machine.cpp
│   └── vcs_state_machine.h
└── README
```

## File Statistics

- Total files: 29
- Total size: 70.0 KB

### File Type Distribution

| Extension | Files | Total Size |
| --- | --- | --- |
| .h | 15 | 14.4 KB |
| .cpp | 13 | 54.5 KB |
| (no extension) | 1 | 1.1 KB |

## File Contents

### README

```plaintext
// README

This directory is intended for project specific (private) libraries.
PlatformIO will compile them to static libraries and link into the executable file.

The source code of each library should be placed in a separate directory
("lib/your_library_name/[Code]").

For example, see the structure of the following example libraries `Foo` and `Bar`:

|--lib
|  |
|  |--Bar
|  |  |--docs
|  |  |--examples
|  |  |--src
|  |     |- Bar.c
|  |     |- Bar.h
|  |  |- library.json (optional. for custom build options, etc) https://docs.platformio.org/page/librarymanager/config.html
|  |
|  |--Foo
|  |  |- Foo.c
|  |  |- Foo.h
|  |
|  |- README --> THIS FILE
|
|- platformio.ini
|--src
   |- main.c

Example contents of `src/main.c` using Foo and Bar:
```
#include <Foo.h>
#include <Bar.h>

int main (void)
{
  ...
}

```

The PlatformIO Library Dependency Finder will find automatically dependent
libraries by scanning project source files.

More information about PlatformIO Library Dependency Finder
- https://docs.platformio.org/page/librarymanager/ldf.html

```

### VCS_Actuators\vcs_deadman.cpp

```cpp
// VCS_Actuators\vcs_deadman.cpp
#include "vcs_deadman.h"
#include "vcs_pins.h"

// =========================================================
// Two grip switches feed an AND gate. Both must be held
// for DEBOUNCE_TICKS consecutive samples before the FSM
// will let us promote into AUTONOMOUS_STATE.
// =========================================================

// Internal state
static bool leftGripPressed   = false;
static bool rightGripPressed  = false;
static bool autoStateRequested = false;

// Debounce: 3 ticks × 10 ms = 30 ms required to flip state
static const uint8_t DEBOUNCE_TICKS = 3;

void initDeadman() {
    // Active HIGH, no PCB pull resistor — internal pull-down keeps the
    // line at LOW when the switch is open.
    pinMode(PIN_DMS_LEFT,  INPUT_PULLDOWN);
    pinMode(PIN_DMS_RIGHT, INPUT_PULLDOWN);
}

void updateDeadman() {
    // HIGH = pressed (switch ties GPIO to 3.3V)
    bool rawLeft  = (digitalRead(PIN_DMS_LEFT)  == HIGH);
    bool rawRight = (digitalRead(PIN_DMS_RIGHT) == HIGH);

    // Saturating up/down counters per channel
    static uint8_t leftCounter  = 0;
    static uint8_t rightCounter = 0;

    if (rawLeft  && leftCounter  < DEBOUNCE_TICKS) leftCounter++;
    if (!rawLeft && leftCounter  > 0)              leftCounter--;
    leftGripPressed  = (leftCounter  >= DEBOUNCE_TICKS);

    if (rawRight  && rightCounter < DEBOUNCE_TICKS) rightCounter++;
    if (!rawRight && rightCounter > 0)              rightCounter--;
    rightGripPressed = (rightCounter >= DEBOUNCE_TICKS);

    // Strict AND
    autoStateRequested = (leftGripPressed && rightGripPressed);
}

bool isDeadmanActive() {
    return autoStateRequested;
}
```

### VCS_Actuators\vcs_deadman.h

```plaintext
// VCS_Actuators\vcs_deadman.h
#ifndef VCS_DEADMAN_H
#define VCS_DEADMAN_H

#include <Arduino.h>

// Configures GPIO 33 + 27 as INPUT_PULLDOWN (active-HIGH switches, no PCB pull).
void initDeadman();

// Polled at 100Hz to debounce both DMS switches.
void updateDeadman();

// Returns true ONLY if BOTH grips have been actively held (debounced).
bool isDeadmanActive();

#endif // VCS_DEADMAN_H
```

### VCS_Actuators\vcs_lowbrake.cpp

```cpp
// VCS_Actuators\vcs_lowbrake.cpp
#include "vcs_lowbrake.h"

// =========================================================
// Brake subsystem
//   - Physical brake switch (GPIO 14, active HIGH)
//   - Brake limit switch  (GPIO 13, active HIGH = at limit)
//   - Linear actuator     (TB6612 H-bridge)
//   - Brake-to-MC signal  (GPIO 12, active LOW = brake on)
// =========================================================

bool is_brake_pressed = false;

// Debounce state for the pedal switch
static uint32_t lastDebounceTime = 0;
static int      lastButtonState  = LOW;   // INPUT_PULLDOWN -> idle reads LOW

// Time-bounded retract state (no lower limit switch on the actuator)
static bool     retracting           = false;
static uint32_t retract_started_ms   = 0;

// Forward decl
static void brake_actuator_extend();
static void brake_actuator_retract();
static void brake_actuator_coast();

void initLowBrake() {
    // Active HIGH inputs, no PCB pull resistors
    pinMode(PIN_LOWBRAKE_IN,  INPUT_PULLDOWN);
    pinMode(PIN_LIMIT_SWITCH, INPUT_PULLDOWN);

    // TB6612 actuator pins
    pinMode(TB6612_IN1_PIN, OUTPUT);
    pinMode(TB6612_IN2_PIN, OUTPUT);
    pinMode(TB6612_PWM_PIN, OUTPUT);

    // Brake signal to motor controller (active LOW)
    pinMode(BRAKE_MC_PIN, OUTPUT);

    // Boot in fully-engaged brake (safe default)
    forceBrakeEngagement(true);
}

void updateLowBrake() {
    // ------------------------------------------------------
    // 1. Debounced read of the physical brake switch
    //    Pin is active HIGH with internal pull-down:
    //    HIGH == pressed, LOW == released.
    // ------------------------------------------------------
    int reading = digitalRead(PIN_LOWBRAKE_IN);
    if (reading != lastButtonState) {
        lastDebounceTime = millis();
    }
    if ((millis() - lastDebounceTime) > DEBOUNCE_TIME_MS) {
        is_brake_pressed = (reading == HIGH);
    }
    lastButtonState = reading;

    // ------------------------------------------------------
    // 2. Instant override: human pushing the pedal beats the FSM.
    // ------------------------------------------------------
    if (is_brake_pressed) {
        forceBrakeEngagement(true);
    } else if (currentState == MANUAL_STATE || currentState == AUTONOMOUS_STATE) {
        // Only release while the FSM says we're in a driving state.
        // FAULT / STOPPING / INIT / IDLE all keep the brake on.
        forceBrakeEngagement(false);
    }

    // ------------------------------------------------------
    // 3. Time-bounded retract — there is no end-stop on the
    //    retract direction, so we cut power after BRAKE_RETRACT_MS
    //    to avoid slamming the actuator into its mechanical limit.
    // ------------------------------------------------------
    if (retracting && (millis() - retract_started_ms >= BRAKE_RETRACT_MS)) {
        brake_actuator_coast();
        retracting = false;
    }
}

void forceBrakeEngagement(bool engage) {
    if (engage) {
        // MC brake signal: active LOW = motor power cut
        digitalWrite(BRAKE_MC_PIN, LOW);

        // Extend actuator until limit switch trips
        if (digitalRead(PIN_LIMIT_SWITCH) == HIGH) {
            // Already at full extension — coast & hold
            brake_actuator_coast();
        } else {
            brake_actuator_extend();
        }
        retracting = false;
    } else {
        // Refuse to release while the human still has the pedal down
        if (is_brake_pressed) return;

        // Release MC brake first
        digitalWrite(BRAKE_MC_PIN, HIGH);

        // Start a timed retract; updateLowBrake() will cut power
        // after BRAKE_RETRACT_MS so we don't overdrive the actuator.
        if (!retracting) {
            brake_actuator_retract();
            retract_started_ms = millis();
            retracting         = true;
        }
    }
}

bool isPhysicalBrakePressed() {
    return is_brake_pressed;
}

// =========================================================
// TB6612 helpers (channels paralleled, single direction pair)
// =========================================================
static void brake_actuator_extend() {
    digitalWrite(TB6612_IN1_PIN, HIGH);
    digitalWrite(TB6612_IN2_PIN, LOW);
    analogWrite(TB6612_PWM_PIN, BRAKE_PWM);
}

static void brake_actuator_retract() {
    digitalWrite(TB6612_IN1_PIN, LOW);
    digitalWrite(TB6612_IN2_PIN, HIGH);
    analogWrite(TB6612_PWM_PIN, BRAKE_PWM);
}

static void brake_actuator_coast() {
    digitalWrite(TB6612_IN1_PIN, LOW);
    digitalWrite(TB6612_IN2_PIN, LOW);
    analogWrite(TB6612_PWM_PIN, 0);
}
```

### VCS_Actuators\vcs_lowbrake.h

```plaintext
// VCS_Actuators\vcs_lowbrake.h
#ifndef VCS_LOWBRAKE_H
#define VCS_LOWBRAKE_H

#include <Arduino.h>
#include "vcs_pins.h"
#include "vcs_constants.h"
#include "vcs_state_machine.h"

// True while the physical brake switch is debounced-pressed.
extern bool is_brake_pressed;

void initLowBrake();
void updateLowBrake();

// Forces brake state from the FSM (engage on FAULT/INIT/STOPPING entry).
void forceBrakeEngagement(bool engage);

bool isPhysicalBrakePressed();

#endif // VCS_LOWBRAKE_H
```

### VCS_Actuators\vcs_relays.cpp

```cpp
// VCS_Actuators\vcs_relays.cpp
#include "vcs_relays.h"
#include "vcs_pins.h"

// --- HARDWARE CONFIGURATION ---
// Most standard opto-isolated Arduino relay modules are Active Low.
// If your specific relay module turns ON when it receives 5V, swap these!
#define RELAY_ENERGIZED   LOW   // Coil gets power, NO closes, NC opens
#define RELAY_DEENERGIZED HIGH  // Coil loses power, NO opens, NC closes

void initRelays() {
    pinMode(PIN_RELAY_STROBE, OUTPUT);
    
    #if !defined(ESP32_VCS)
        pinMode(PIN_RELAY_STATE, OUTPUT);
    #endif

    // Default to Manual Mode on boot (Safe State)
    digitalWrite(PIN_RELAY_STROBE, RELAY_DEENERGIZED);
    
    #if !defined(ESP32_VCS)
        digitalWrite(PIN_RELAY_STATE, RELAY_DEENERGIZED);
    #endif
}

void updateRelays(bool isAutonomous) {
    if (isAutonomous) {
        // --- AUTONOMOUS MODE ---
        // 1. Strobe: Relay energized -> NO contact closes -> 12V flows to Orange Strobe
        digitalWrite(PIN_RELAY_STROBE, RELAY_ENERGIZED);
        
        #if !defined(ESP32_VCS)
            // 2. Organizer State: Relay energized
            digitalWrite(PIN_RELAY_STATE, RELAY_ENERGIZED);
        #endif
        
    } else {
        // --- MANUAL MODE ---
        // 1. Strobe: Relay de-energized -> NO contact opens -> Strobe turns OFF
        digitalWrite(PIN_RELAY_STROBE, RELAY_DEENERGIZED);
        
        #if !defined(ESP32_VCS)
            // 2. Organizer State: Relay de-energized 
            digitalWrite(PIN_RELAY_STATE, RELAY_DEENERGIZED);
        #endif
    }
}
```

### VCS_Actuators\vcs_relays.h

```plaintext
// VCS_Actuators\vcs_relays.h
#ifndef VCS_RELAYS_H
#define VCS_RELAYS_H

#include <Arduino.h>

void initRelays();
void updateRelays(bool isAutonomous);

#endif // VCS_RELAYS_H
```

### VCS_Actuators\vcs_throttle.cpp

```cpp
// VCS_Actuators\vcs_throttle.cpp
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
```

### VCS_Actuators\vcs_throttle.h

```plaintext
// VCS_Actuators\vcs_throttle.h
#ifndef VCS_THROTTLE_H
#define VCS_THROTTLE_H

#include <Arduino.h>
#include "vcs_pins.h"
#include "vcs_constants.h"
#include "vcs_state_machine.h"
#include <QuickPID.h>

extern uint16_t current_throttle_adc;
extern uint16_t current_pwm_duty;

void initThrottle();
void updateThrottle(float measured_rpm, float target_rpm);
bool isThrottlePedalPressed();

#endif // VCS_THROTTLE_H
```

### VCS_Comm\vcs_uart.cpp

```cpp
// VCS_Comm\vcs_uart.cpp
#include "vcs_uart.h"
#include "vcs_hallsensor.h"
#include "vcs_steering.h"
#include "vcs_simulation.h"
#include "vcs_state_machine.h"
#include "vcs_pins.h"
#include "vcs_reverse.h"
#include "vcs_web.h"
static constexpr bool UART_DEBUG_LOGS = false;

// =========================================================
// Sidlak-2 Frame (fixed 14 bytes, both directions)
//
// RX (Jetson -> ESP32, msg type 0x01):
//   [0]=AA [1]=55 [2]=01 [3]=07 [4]=Mode
//   [5]=RPM_H  [6]=RPM_L
//   [7]=Steer_H [8]=Steer_L
//   [9]=Brake [10]=Reverse
//   [11]=CRC_H [12]=CRC_L [13]=FF
//
// TX (ESP32 -> Jetson, msg type 0x02):
//   [0]=AA [1]=55 [2]=02 [3]=07
//   [4]=RPM_H [5]=RPM_L
//   [6]=Steer_H [7]=Steer_L
//   [8]=State [9]=Gear [10]=Reverse
//   [11]=CRC_H [12]=CRC_L [13]=FF
//
// CRC16 polynomial 0xA001, init 0xFFFF, computed over bytes 2..10
// (Type + Length + Payload), big-endian on the wire.
// =========================================================

// =========================================================
// Mux-protected target state — written from RX, read from
// any task via the getters at the bottom of this file.
// =========================================================
static portMUX_TYPE uartMux = portMUX_INITIALIZER_UNLOCKED;

static volatile uint8_t  target_mode              = 0;
static volatile float    target_rpm               = 0.0f;
static volatile uint16_t target_steering          = 500;
static volatile uint8_t  target_brake             = 0;
static volatile bool     target_direction_reverse = false;
static volatile uint32_t last_valid_packet_time   = 0;
extern String last_rx_hex;
extern String last_tx_hex;

// Legacy mirrors (see header note). Updated alongside the
// mux-protected versions for backward source-compat.
uint8_t  current_target_brake = 0;
int16_t  current_target_rpm   = 0;
uint16_t current_target_steer = 0;
uint8_t  current_target_mode  = 0;
uint32_t last_uart_time       = 0;

// =========================================================
// Forward decls
// =========================================================
static uint16_t calculateCRC16(const uint8_t *data, uint8_t length);
static void     processCommandPacket(const uint8_t *packet);

// =========================================================
// INIT
// =========================================================
void initUART() {
    Serial2.begin(115200, SERIAL_8N1, JETSON_RX_PIN, JETSON_TX_PIN);

    portENTER_CRITICAL(&uartMux);
    last_valid_packet_time = millis();
    portEXIT_CRITICAL(&uartMux);
    last_uart_time = millis();
}

// =========================================================
// SINGLE RX PARSER
// Fixed-length 14-byte frame.  Discards bytes until a valid
// AA 55 header appears, then reads the rest of the frame in
// one shot. Footer + CRC must both validate.
// =========================================================
void handleIncomingUART() {
    while (Serial2.available() >= 14) {
        if (Serial2.read() != 0xAA) continue;
        if (Serial2.peek() != 0x55) continue;
        Serial2.read();  // consume 0x55

        uint8_t pkt[14];
        pkt[0] = 0xAA;
        pkt[1] = 0x55;
        if (Serial2.readBytes(&pkt[2], 12) != 12) continue;

        // Footer
        if (pkt[13] != 0xFF) {
            vcs_log("UART: bad footer");
            continue;
        }

        // CRC over bytes 2..10 (Type + Length + 7-byte payload = 9 bytes)
        uint16_t recvCRC = ((uint16_t)pkt[11] << 8) | pkt[12];
        uint16_t calcCRC = calculateCRC16(&pkt[2], 9);
        if (recvCRC != calcCRC) {
            if (UART_DEBUG_LOGS) {
                Serial.print(F("UART CRC mismatch: calc="));
                Serial.print(calcCRC, HEX);
                Serial.print(F(" recv="));
                Serial.println(recvCRC, HEX);
            }
            continue;
        }

        // After CRC check succeeds
        last_rx_hex = "";
        for(int i=0; i<14; i++) {
            if(pkt[i] < 0x10) last_rx_hex += "0";
            last_rx_hex += String(pkt[i], HEX) + " ";
        }
        last_rx_hex.toUpperCase();

        // Print the valid 14-byte packet to your PC Serial Monitor
        if (UART_DEBUG_LOGS) {
            printHexDebug("RX [JETSON -> ESP32]: ", pkt, 14);
        }
        // -------
        
        // Frame is valid — record timestamp and dispatch
        last_uart_time = millis();
        portENTER_CRITICAL(&uartMux);
        last_valid_packet_time = last_uart_time;
        portEXIT_CRITICAL(&uartMux);

        processCommandPacket(pkt);
    }
}

// =========================================================
// PACKET DISPATCH
// =========================================================
static void processCommandPacket(const uint8_t *pkt) {
    uint8_t msgType = pkt[2];
    if (msgType != 0x01) return;          // Only Jetson command packets here

    // ---- Decode payload (BIG-ENDIAN, per protocol spec) ----
    uint8_t  mode      =  pkt[4];
    int16_t  rpm       =  (int16_t)(((uint16_t)pkt[5] << 8) | pkt[6]);
    uint16_t steer     =  ((uint16_t)pkt[7] << 8) | pkt[8];
    uint8_t  brake     =  pkt[9];
    bool     reverseOn = (pkt[10] == 1);

    // Update legacy mirrors first (unprotected, single-writer)
    current_target_mode  = mode;
    current_target_rpm   = rpm;
    current_target_steer = steer;
    current_target_brake = brake;

    // Constrained, mux-protected handoff to control tasks
    portENTER_CRITICAL(&uartMux);
    target_mode              = mode;
    target_rpm               = (float)constrain(rpm,   COMM_SPEED_MIN, COMM_SPEED_MAX);
    target_steering          = (uint16_t)constrain(steer, COMM_STEER_LEFT, COMM_STEER_RIGHT);
    target_brake             = (uint8_t) constrain(brake, COMM_BRAKE_MIN,  COMM_BRAKE_MAX);
    target_direction_reverse = reverseOn;
    portEXIT_CRITICAL(&uartMux);
}

// =========================================================
// CRC16 (Modbus, polynomial 0xA001, init 0xFFFF)
// =========================================================
static uint16_t calculateCRC16(const uint8_t *data, uint8_t length) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xA001;
            else         crc >>= 1;
        }
    }
    return crc;
}

// =========================================================
// TELEMETRY TX — broadcastVehicleTelemetry()
// Writes a 14-byte frame mirroring the RX format.
// =========================================================
void broadcastVehicleTelemetry() {
    float    rpm;
    uint16_t steer;
    uint8_t  state = (uint8_t)currentState;
    uint8_t  rev   = isReverseEngaged() ? 1 : 0;

    uint8_t gear = 1;  // Normal
    if      (current_drive_mode == DRIVE_LOW)  gear = 0;
    else if (current_drive_mode == DRIVE_HIGH) gear = 2;

#if SIMULATION_MODE
    rpm   = getSimulatedRPM();
    steer = getSimulatedSteering();
    Serial.print(F("SIM -> RPM:"));  Serial.print(rpm);
    Serial.print(F(" Steer:"));       Serial.print(steer);
    Serial.print(F(" Gear:"));        Serial.print(gear);
    Serial.print(F(" Rev:"));         Serial.println(rev);
#else
    rpm   = getMeasuredRPM();
    steer = getMeasuredSteering();
#endif

    int16_t rpmInt = (int16_t)rpm;

    // Build [Type][Len][Payload(7)]  = 9 bytes covered by CRC
    uint8_t buf[9];
    buf[0] = 0x02;                            // msg type
    buf[1] = 0x07;                            // payload length
    buf[2] = (uint8_t)((rpmInt >> 8) & 0xFF); // RPM_H
    buf[3] = (uint8_t)( rpmInt       & 0xFF); // RPM_L
    buf[4] = (uint8_t)((steer  >> 8) & 0xFF); // Steer_H
    buf[5] = (uint8_t)( steer        & 0xFF); // Steer_L
    buf[6] = state;
    buf[7] = gear;
    buf[8] = rev;

    uint16_t crc = calculateCRC16(buf, 9);

    // Debug Telemetry + frame assembly
    uint8_t txDebug[14] = {
        0xAA, 0x55, buf[0], buf[1], buf[2], buf[3], buf[4], 
        buf[5], buf[6], buf[7], buf[8], 
        (uint8_t)((crc >> 8) & 0xFF), (uint8_t)(crc & 0xFF), 0xFF
    };
    if (UART_DEBUG_LOGS) {
        printHexDebug("TX [ESP32 -> JETSON]: ", txDebug, 14);
    }
    
    // Always broadcast telemetry to Jetson (both live and simulation modes)
    // so handshake health can be validated during bench tests.
    Serial2.write((uint8_t)0xAA);
    Serial2.write((uint8_t)0x55);
    Serial2.write(buf, 9);
    Serial2.write((uint8_t)((crc >> 8) & 0xFF));
    Serial2.write((uint8_t)( crc       & 0xFF));
    Serial2.write((uint8_t)0xFF);

    last_tx_hex = "";
        for(int i=0; i<14; i++) {
            if(txDebug[i] < 0x10) last_tx_hex += "0";
            last_tx_hex += String(txDebug[i], HEX) + " ";
        }
        last_tx_hex.toUpperCase();
}

// =========================================================
// THREAD-SAFE GETTERS
// =========================================================
uint8_t getANSCommandMode() {
    portENTER_CRITICAL(&uartMux);
    uint8_t v = target_mode;
    portEXIT_CRITICAL(&uartMux);
    return v;
}

float getTargetRPM() {
    portENTER_CRITICAL(&uartMux);
    float v = target_rpm;
    portEXIT_CRITICAL(&uartMux);
    return v;
}

uint16_t getTargetSteering() {
    portENTER_CRITICAL(&uartMux);
    uint16_t v = target_steering;
    portEXIT_CRITICAL(&uartMux);
    return v;
}

uint8_t getTargetBrake() {
    portENTER_CRITICAL(&uartMux);
    uint8_t v = target_brake;
    portEXIT_CRITICAL(&uartMux);
    return v;
}

bool getANSReverseCommand() {
    portENTER_CRITICAL(&uartMux);
    bool v = target_direction_reverse;
    portEXIT_CRITICAL(&uartMux);
    return v;
}

bool ansHeartbeatReceived() {
    portENTER_CRITICAL(&uartMux);
    uint32_t t = last_valid_packet_time;
    portEXIT_CRITICAL(&uartMux);
    return (millis() - t) <= 500;   // 500 ms — tolerates 10 dropped packets at 20Hz
}

bool isJetsonStopLineActive() {
    // Treat any non-zero brake request as a stop-line command
    return (getTargetBrake() > 0);
}

// =========================================================
// UART UPDATE TASK (STUB)
// =========================================================
void updateUART() {
    // RX is handled securely by handleIncomingUART() on Core 1.
    // TX is handled by broadcastVehicleTelemetry() on Core 0.
    // This stub safely satisfies the CommTask sequence without 
    // causing a hardware UART race condition between the dual cores.
}

// =========================================================
// DEBUG HELPER: Print Byte Array as Hex String
// =========================================================
void printHexDebug(const char* prefix, const uint8_t* data, uint8_t length) {
    Serial.print(prefix);
    for (uint8_t i = 0; i < length; i++) {
        if (data[i] < 0x10) Serial.print("0"); // Pad single digits with a leading zero
        Serial.print(data[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}
```

### VCS_Comm\vcs_uart.h

```plaintext
// VCS_Comm\vcs_uart.h
#ifndef VCS_UART_H
#define VCS_UART_H

#include <Arduino.h>
#include "vcs_pins.h"
#include "vcs_constants.h"
#include "vcs_threespeed.h"

// Pulled in for the gear byte in broadcastVehicleTelemetry()
extern DriveMode current_drive_mode;

// =========================================================
// Core UART functions
// =========================================================
// Single Serial2 reader. Call from EXACTLY ONE task (task_control)
// to avoid a hardware-FIFO race between cores.
void initUART();
void handleIncomingUART();
void updateUART();

// Telemetry TX. Safe to call from any single task.
void broadcastVehicleTelemetry();

// One-time setup
void printHexDebug(const char* prefix, const uint8_t* data, uint8_t length);



// =========================================================
// Thread-safe getters (mux-protected reads of last RX command)
// =========================================================
uint8_t  getANSCommandMode();
float    getTargetRPM();
uint16_t getTargetSteering();
uint8_t  getTargetBrake();
bool     getANSReverseCommand();
bool     ansHeartbeatReceived();
bool     isJetsonStopLineActive();

void printHexDebug();// =========================================================
// Legacy unprotected globals — kept for source-compat with
// modules that read them directly. New code should use the
// getters above.
// =========================================================
extern uint8_t  current_target_brake;
extern int16_t  current_target_rpm;
extern uint16_t current_target_steer;
extern uint8_t  current_target_mode;
extern uint32_t last_uart_time;

#endif // VCS_UART_H
```

### VCS_Config\vcs_constants.h

```plaintext
// VCS_Config\vcs_constants.h
#ifndef VCS_CONSTANTS_H
#define VCS_CONSTANTS_H

#include <Arduino.h>

// ==========================================
// System Architecture & Simulation
// ==========================================
#define SIMULATION_MODE 0      // 1 = Digital Twin Mode, 0 = LIVE 1500W BLDC Control
#define V_LOGIC         3.3f   // ESP32 logic level (used by ADC scaling helpers)

#if SIMULATION_MODE
  #pragma message ("VCS BUILD >>> SIMULATION_MODE = 1  (Digital Twin, no live motor output)")
#else
  #pragma message ("VCS BUILD >>> SIMULATION_MODE = 0  (LIVE 1500W MOTOR CONTROL - verify E-stop)")
#endif

// ==========================================
// System Frequencies & Timing
// ==========================================
#define FREQ_CONTROL_HZ     1000  // Core control loop (1 ms)
#define FREQ_STEER_CTRL_HZ  100   // Steering inner loop (10 ms)
#define FREQ_COMM_HZ        100   // Comms / inputs sweep (10 ms)
#define FREQ_UI_HZ          20    // OLED + telemetry (50 ms)
#define DEBOUNCE_TIME_MS    50    // Debounce window for physical brake

// ==========================================
// Motor & Powertrain
// ==========================================
// IMPORTANT: pole-pair count MUST match the actual motor on the car.
// Verify by spinning the wheel one mechanical revolution by hand,
// counting Hall transitions, and dividing by 6.
//
// VCS_TECHNICAL_SPECIFICATION lists POLE_PAIRS = 23 with
// HALL_TRANSITIONS_PER_MECH_REV = 138 (= 6 × 23). The value below
// (16) is from an earlier bench measurement. RECONCILE before first
// live run — these cannot both be right.
#define MOTOR_POLE_PAIRS      16
#define HALL_TRANSITIONS_REV  6     // 6 transitions per electrical cycle
#define GEAR_REDUCTION        1.0f

// ==========================================
// Throttle Output (ESP32 DAC, 8-bit native)
// ==========================================
// GPIO25 -> LM358 -> motor controller.  dacWrite() takes 0–255.
// --- Legacy Bridging Macros for Throttle PID (10-bit scale) ---
#define MIN_PWM_OUT 0
#define MAX_PWM_OUT 1023
#define THROTTLE_MIN_INPUT 180
#define THROTTLE_MAX_INPUT 850
// ==========================================
// Speed PI (target RPM -> DAC value)
// ==========================================
// Gains scaled from the legacy 0–1023 PWM values (KP=0.8, KI=0.15) by
// 1/4 to match the 0–255 DAC output range. These are STARTING POINTS
// only — re-tune on the bench against the real motor controller.
#define SPEED_KP              0.20f
#define SPEED_KI              0.0375f

// ==========================================
// Steering PID (target -> stepper effort)
// ==========================================
// PID setpoint and input both run in COMM units (0..1000 across full
// steering travel), so the deadzone is in the same space.
#define STEER_KP              1.2f
#define STEER_KI              0.05f
#define STEER_KD              0.01f
#define STEER_DEADZONE        5     // COMM units (~0.5% of full travel)

// ==========================================
// Brake Actuator (TB6612 + 12V linear)
// ==========================================
#define BRAKE_PWM             200   // 0–255, applied to TB6612 PWMA/PWMB
#define BRAKE_RETRACT_MS      900   // Time-limited retract (no lower limit switch).
                                    // Calibrate physically: power the actuator at
                                    // BRAKE_PWM and time a full retract from
                                    // engaged to fully released.

// ==========================================
// Communication Protocol Ranges (ANS -> VCS)
// ==========================================
#define COMM_SPEED_MIN        0
#define COMM_SPEED_MAX        3000  // Maximum target RPM
#define COMM_STEER_LEFT       0
#define COMM_STEER_CENTER     500
#define COMM_STEER_RIGHT      1000
#define COMM_BRAKE_MIN        0
#define COMM_BRAKE_MAX        1     // Binary (0=Off, 1=On)

// ==========================================
// Physical Interface Mapping (ESP32 ADC -> mV)
// ==========================================
// Thresholds below are in MILLIVOLTS as returned by
// esp_adc_cal_raw_to_voltage().  Always go through the calibrated
// reader — never compare these against raw analogRead() values.

// --- Throttle pedal (via 10k/18k divider, ~0–3210 mV at full press) ---
#define THROTTLE_DEADBAND_MV   50
#define THROTTLE_MIN_INPUT_MV  150
#define THROTTLE_MAX_INPUT_MV  3000

// --- Steering pot (3590S, 3.3V powered) ---
#define STEER_POT_MIN_MV       200
#define STEER_POT_CENTER_MV    1650
#define STEER_POT_MAX_MV       3100

#endif // VCS_CONSTANTS_H
```

### VCS_Config\vcs_pins.h

```plaintext
// VCS_Config\vcs_pins.h
#ifndef VCS_PINS_H
#define VCS_PINS_H

#include <Arduino.h>

// ==============================================================================
// MODULE:      VCS_Pins (ESP32-WROOM-32, 38-pin DevKit)
// PROJECT:     SIDLAK 2 — Shell Eco-marathon 2026
// ==============================================================================

// --- Sensors & ADCs ---
#define PIN_HALL_A         36   // Input-only silicon (no pull-up/down possible)
#define PIN_HALL_B         39   // Input-only silicon (no pull-up/down possible)
#define PIN_HALL_SPEED     32   // Hall C — normal GPIO, INPUT_PULLDOWN in firmware
#define PIN_THROTTLE_IN    34   // ADC1_CH6 — pedal via 5V→3.3V divider
#define PIN_STEER_POT      35   // ADC1_CH7 — 3590S steering pot (3.3V powered)

// --- Switches (Digital Inputs) ---
#define PIN_DMS_LEFT       33   // Active HIGH, INPUT_PULLDOWN in firmware
#define PIN_DMS_RIGHT      27   // Active HIGH, INPUT_PULLDOWN in firmware
#define PIN_LOWBRAKE_IN    14   // Brake switch, active HIGH, INPUT_PULLDOWN
#define PIN_LIMIT_SWITCH   13   // Brake actuator end-stop, active HIGH, INPUT_PULLDOWN
#define PIN_REVERSE_IN     26   // Active LOW (latching toggle); 10k pull-up on PCB —
                                // configure as plain INPUT, NOT INPUT_PULLUP/DOWN

// --- Actuators & Outputs ---
#define PIN_THROTTLE_OUT   25   // DAC1 (0–3.3V) → LM358 → 0–4.7V to motor controller
#define PIN_STEER_PUL      18   // LEDC ch0 → DM542 PUL-
#define PIN_STEER_DIR      19   // → DM542 DIR-
#define PIN_STEER_ENA      23   // → DM542 ENA-  (LOW = disabled, HIGH = enabled)


// --- TB6612 Brake Actuator (channels paralleled) ---
#define TB6612_IN1_PIN      4
#define TB6612_IN2_PIN      2
#define TB6612_PWM_PIN      5

// --- Misc Outputs ---
#define BRAKE_MC_PIN       12   // Brake signal to motor controller (active LOW)
#define PIN_RELAY_STROBE   15   // 2N2222 → relay coil  (HIGH = relay ON)

// --- Communications ---
#define JETSON_RX_PIN      16
#define JETSON_TX_PIN      17
#define I2C_SDA_PIN        21
#define I2C_SCL_PIN        22

#endif // VCS_PINS_H
```

### VCS_Display\vcs_display.cpp

```cpp
// VCS_Display\vcs_display.cpp
#include "vcs_display.h"
#include "vcs_state_machine.h"
#include "vcs_reverse.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

Adafruit_SSD1306 display(128, 64, &Wire, -1);

void initDisplay() {
    // Explicitly assign I2C pins for ESP32 to prevent defaults routing elsewhere
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        // Log I2C failure if necessary
    }

    Wire.setClock(400000);
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE); 
}

void updateDisplay(float rpm, uint16_t steer, DriveMode speedMode) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.setTextSize(2);
    
    if(currentState == FAULT_STATE) {
        display.println("!! CRITICAL !!");
    } else {
        display.println(currentState == AUTONOMOUS_STATE ? "AUTO MODE" : "MANUAL");
    }

    display.setTextSize(1);
    
    display.print("\nRPM:   ");
    display.println(rpm, 1);
    
    display.print("STEER: ");
    display.println(steer);
    
    display.print("GEAR:  ");
    if (isReverseEngaged()) {
        display.println("REVERSE"); 
    } else {
        display.println(speedMode == DRIVE_LOW ? "LOW" : (speedMode == DRIVE_HIGH ? "HIGH" : "MED"));
    }
    
    if (currentState == MANUAL_STATE && getDMSHoldStartTime() > 0) {
        display.setCursor(0, 45);
        display.print("ENGAGING AUTO..."); 
        
        int progress = (millis() - getDMSHoldStartTime()) / 10; 
        
        display.drawRect(0, 55, 128, 5, SSD1306_WHITE);
        display.fillRect(0, 55, (progress * 128) / 100, 5, SSD1306_WHITE);
    }
    
    display.display(); 
}
```

### VCS_Display\vcs_display.h

```plaintext
// VCS_Display\vcs_display.h
#ifndef VCS_DISPLAY_H
#define VCS_DISPLAY_H

#include <Arduino.h>
#include <Wire.h>
#include "vcs_threespeed.h"
#include "vcs_pins.h"

void initDisplay();
void updateDisplay(float rpm, uint16_t steer, DriveMode speedMode);

#endif // VCS_DISPLAY_H
```

### VCS_Hall\vcs_hallsensor.cpp

```cpp
// VCS_Hall\vcs_hallsensor.cpp
#include "vcs_hallsensor.h"
#include "vcs_pins.h"

#define WHEEL_CIRCUMFERENCE_M 1.2764f

volatile uint32_t hall_pulse_count = 0;
uint32_t last_calc_time = 0;
float current_rpm = 0.0f;
static bool s_new_rpm_sample = false;

// MUST be declared before attachInterrupt uses it
void IRAM_ATTR handleHallInterrupt() {
    hall_pulse_count++;
}

void initHallSensors() {
    // ESP32 supports internal pulldowns
    pinMode(PIN_HALL_SPEED, INPUT_PULLDOWN); 
    last_calc_time = millis();
}

void hall_interrupts_attach() {
    attachInterrupt(digitalPinToInterrupt(PIN_HALL_SPEED), handleHallInterrupt, RISING);
}

void hall_interrupts_detach() {
    detachInterrupt(digitalPinToInterrupt(PIN_HALL_SPEED));
}

void updateHallCalculations() {
    uint32_t now = millis();
    uint32_t elapsed = now - last_calc_time;

    if (elapsed >= 100) {
        // 1. Atomically read and reset pulse count
        noInterrupts(); 
        uint32_t pulses = hall_pulse_count;
        hall_pulse_count = 0;
        interrupts();

        // 2. Math for RPM
        float pulses_per_rev = (float)MOTOR_POLE_PAIRS; 
        pulses_per_rev *= GEAR_REDUCTION; 
        
        if (pulses > 0) {
            current_rpm = ((float)pulses / pulses_per_rev) * (60000.0f / (float)elapsed);
        } else {
            current_rpm = 0.0f;
        }

        last_calc_time = now;
        s_new_rpm_sample = true; 
    }
}

bool consumeNewRPMSample() {
    if (s_new_rpm_sample) {
        s_new_rpm_sample = false;
        return true;
    }
    return false;
}

float getMeasuredRPM() {
    #if SIMULATION_MODE
        return getSimulatedRPM();
    #else
        return current_rpm;
    #endif
}

float getMeasuredSpeedKmh() {
    float rpm = getMeasuredRPM();
    return (rpm * WHEEL_CIRCUMFERENCE_M * 60.0f) / 1000.0f;
}
```

### VCS_Hall\vcs_hallsensor.h

```plaintext
// VCS_Hall\vcs_hallsensor.h
#ifndef VCS_HALLSENSOR_H
#define VCS_HALLSENSOR_H

#include <Arduino.h>
#include "vcs_pins.h"
#include "vcs_constants.h"
#include "vcs_simulation.h"

void initHallSensors();
void hall_interrupts_attach();
void hall_interrupts_detach();
void updateHallCalculations();

float getMeasuredRPM();
float getMeasuredSpeedKmh();
bool consumeNewRPMSample();

#endif // VCS_HALLSENSOR_H
```

### VCS_Simulation\vcs_simulation.cpp

```cpp
// VCS_Simulation\vcs_simulation.cpp
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
```

### VCS_Simulation\vcs_simulation.h

```plaintext
// VCS_Simulation\vcs_simulation.h
#ifndef VCS_SIMULATION_H
#define VCS_SIMULATION_H

#include <Arduino.h>
#include "vcs_constants.h"

void updateSimulatedPhysics(int pulse_freq, bool direction);
float getSimulatedRPM();
float getSimulatedSteering();

#endif // VCS_SIMULATION_H
```

### VCS_Simulation\vcs_web.cpp

```cpp
// VCS_Simulation\vcs_web.cpp
#include <Arduino.h>
#include "vcs_constants.h"
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#include "vcs_state_machine.h"
#include "vcs_hallsensor.h"
#include "vcs_steering.h"
#include "vcs_threespeed.h"
#include "vcs_reverse.h"
#include "vcs_uart.h"     
#include "vcs_lowbrake.h" 
#include "vcs_deadman.h"  
#include "vcs_web.h"

extern DriveMode current_drive_mode;

// Added scope for target tracking to prevent compiler errors
extern int16_t  current_target_rpm;
extern uint16_t current_target_steer;
extern uint8_t  current_target_brake;
extern uint8_t  current_target_mode;

// Network credentials sanitized for baseline security
const char* ssid = "SIDLAK_VCS_LIVE";
const char* password = "sidlak_secure"; 
String last_rx_hex = "00 00 00 00 00 00 00 00 00 00 00 00 00 00";
String last_tx_hex = "00 00 00 00 00 00 00 00 00 00 00 00 00 00";


#define WHEEL_CIRCUMFERENCE_M 1.2764f 
#define MAX_STEERING_ANGLE_DEG 35.0f  

AsyncWebServer server(80);
String systemLogBuffer = "";

void vcs_log(String msg) {
    Serial.println("[VCS LOG] " + msg); 
    systemLogBuffer += msg + "|"; 
}

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>SIDLAK 2: LIVE VCS DASHBOARD</title>
    <style>
        body { font-family: 'Courier New', Courier, monospace; background-color: #050505; color: #00FF00; padding: 20px; }
        .grid-container { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; max-width: 1100px; }
        .panel { border: 1px solid #00FF00; padding: 15px; background: #000; margin-bottom: 20px; box-shadow: 0 0 10px rgba(0,255,0,0.2); }
        h3 { margin-top: 0; color: #00FF00; border-bottom: 1px solid #00FF00; padding-bottom: 5px; text-transform: uppercase; }
        table { width: 100%; border-collapse: collapse; margin-top: 10px; }
        th, td { border: 1px solid #333; padding: 10px; text-align: left; }
        button { background-color: #000; color: #00FF00; border: 1px solid #00FF00; padding: 12px; cursor: pointer; font-weight: bold; width: 100%; margin-bottom: 10px; text-transform: uppercase; }
        button:hover { background-color: #00FF00; color: #000; }
        #logBox { height: 300px; overflow-y: scroll; background: #000; color: #0f0; padding: 10px; border: 1px solid #333; font-size: 0.9em; margin-top: 10px; }
        .recording { background-color: #ff0000 !important; color: #fff !important; border: none; animation: blink 1s infinite; }
        @keyframes blink { 50% { opacity: 0.5; } }
    </style>
</head>
<body>
    <h2>SIDLAK 2: LIVE VEHICLE CONTROL SYSTEM</h2>
    <div class="grid-container">
        <div class="panel">
            <h3>Physical Telemetry</h3>
            <table>
                <tr><td>FSM State</td><td id="live_state" style="font-weight:bold;">--</td></tr>
                <tr><td>DMS Safety</td><td id="live_dms">--</td></tr>
                <tr><td>Live Speed</td><td id="live_speed">--</td></tr>
                <tr><td>Wheel RPM</td><td id="live_rpm">--</td></tr>
                <tr><td>Steer Angle</td><td id="live_steer">--</td></tr>
                <tr><td>Gear/Dir</td><td id="live_dir">--</td></tr>
                <tr><td>3-Speed Switch</td><td id="live_3speed">--</td></tr>
            </table>
        </div>
        <div class="panel">
            <h3>Data Logger</h3>
            <button id="recBtn" onclick="toggleRecording()">START RECORDING</button>
            <button onclick="downloadCSV()">DOWNLOAD TELEMETRY (CSV)</button>
            <button onclick="clearLogs()" style="border-color: #444; color: #888;">CLEAR LOGS</button>
        </div>

        <div class="panel">
            <h3>Jetson Target Requests (UART)</h3>
            <table>
                <tr><td>Target Mode</td><td id="live_t_mode" style="color:#ff00ff; font-weight:bold;">--</td></tr>
                <tr><td>Target RPM</td><td id="live_t_rpm" style="color:#ff00ff;">--</td></tr>
                <tr><td>Target Steer</td><td id="live_t_steer" style="color:#ff00ff;">--</td></tr>
                <tr><td>Target Brake</td><td id="live_t_brake" style="color:#ff00ff;">--</td></tr>
            </table>
        </div>
    </div>
    <div class="panel">

    <div class="panel" style="grid-column: span 2;">
            <h3>Raw UART Stream (Hex)</h3>
            <div style="display: flex; gap: 10px;">
                <div style="flex: 1;">
                    <span style="color: #00FFFF;">[JETSON -> ESP32]</span>
                    <div id="rx_hex" style="background: #111; padding: 10px; border: 1px solid #00FFFF; margin-top: 5px;">--</div>
                </div>
                <div style="flex: 1;">
                    <span style="color: #FFFF00;">[ESP32 -> JETSON]</span>
                    <div id="tx_hex" style="background: #111; padding: 10px; border: 1px solid #FFFF00; margin-top: 5px;">--</div>
                </div>
            </div>
        </div>

        <h3>System Activity Log</h3>
        <div id="logBox"></div>
    </div>

    

    <script>
        let isRecording = false;
        let csvRows = [["Timestamp", "FSM_State", "DMS", "RPM", "Speed_kmh", "Steer_Deg", "Dir"]];

        function appendLog(msg) {
            const log = document.getElementById('logBox');
            log.innerHTML += `[${new Date().toLocaleTimeString()}] ${msg}<br>`;
            log.scrollTop = log.scrollHeight;
        }

        function toggleRecording() {
            isRecording = !isRecording;
            const btn = document.getElementById('recBtn');
            btn.innerText = isRecording ? "STOPPING & SAVING..." : "START RECORDING";
            btn.classList.toggle('recording', isRecording);
            appendLog(isRecording ? "SESSION STARTED" : "SESSION PAUSED");
        }

        function clearLogs() {
            document.getElementById('logBox').innerHTML = '';
            csvRows = [["Timestamp", "FSM_State", "DMS", "RPM", "Speed_kmh", "Steer_Deg", "Dir"]];
            appendLog("Logs cleared.");
        }

        function downloadCSV() {
            if(csvRows.length < 2) return alert("No data recorded!");
            let content = csvRows.map(e => e.join(",")).join("\n");
            let blob = new Blob([content], { type: 'text/csv' });
            let a = document.createElement('a');
            a.href = window.URL.createObjectURL(blob);
            a.download = `SIDLAK_LIVE_LOG_${new Date().getTime()}.csv`;
            a.click();
        }

        setInterval(() => {
            fetch('/data').then(r => r.json()).then(data => {
                document.getElementById('live_state').innerText = data.state;
                document.getElementById('live_dms').innerText = data.dms;
                document.getElementById('live_rpm').innerText = data.rpm;
                document.getElementById('live_speed').innerText = data.speed + " km/h";
                document.getElementById('live_steer').innerText = data.steer_angle + "°";
                document.getElementById('live_dir').innerText = data.dir;
                document.getElementById('live_3speed').innerText = data.three_speed;
                document.getElementById('live_t_rpm').innerText = data.t_rpm;
                document.getElementById('live_t_steer').innerText = data.t_steer;
                document.getElementById('live_t_brake').innerText = data.t_brake + "%";
                document.getElementById('live_t_mode').innerText = data.t_mode;
                document.getElementById('rx_hex').innerText = data.rx_hex;
                document.getElementById('tx_hex').innerText = data.tx_hex;
                
                if(isRecording) {
                    csvRows.push([new Date().toLocaleTimeString(), data.state, data.dms, data.rpm, data.speed, data.steer_angle, data.dir]);
                }

                if (data.sys_logs && data.sys_logs !== "") {
                    data.sys_logs.split('|').forEach(msg => {
                        if(msg.length > 0) appendLog("ESP32: " + msg);
                    });
                }
            }).catch(err => console.error("Telemetry Break:", err));
        }, 200);
        
        appendLog("System Online: Ready for live telemetry.");
    </script>
</body>
</html>
)rawliteral";

String getTelemetryJSON() {
    String fsm_state = String(getStateName(currentState));
    bool dms_active = isDeadmanActive();
    float rpm = getMeasuredRPM();
    uint16_t steer_adc = getMeasuredSteering();
    bool is_rev = isReverseEngaged();

    float speed_kmh = getMeasuredSpeedKmh();
    float steer_angle = (((float)steer_adc - 500.0f) / 500.0f) * MAX_STEERING_ANGLE_DEG;

    int16_t rpm_int = (int16_t)rpm;
    uint8_t rpm_h = (rpm_int >> 8) & 0xFF;
    uint8_t rpm_l = rpm_int & 0xFF;
    uint8_t steer_h = (steer_adc >> 8) & 0xFF;
    uint8_t steer_l = steer_adc & 0xFF;
    uint8_t u_state = 3; 

    String speed_mode = "UNKNOWN";
    if (current_drive_mode == DRIVE_LOW) speed_mode = "LOW (30%)";
    else if (current_drive_mode == DRIVE_MED) speed_mode = "MID (60%)";
    else if (current_drive_mode == DRIVE_HIGH) speed_mode = "HIGH (100%)";

    String json;
    json.reserve(512); 


    
    json = "{";
    json += "\"state\":\"" + fsm_state + "\",";
    json += "\"dms\":\"" + String(dms_active ? "HELD (OK)" : "RELEASED") + "\",";
    json += "\"rpm\":\"" + String(rpm, 1) + "\",";
    json += "\"speed\":\"" + String(speed_kmh, 1) + "\",";
    json += "\"steer_angle\":\"" + String(steer_angle, 1) + "\",";
    json += "\"dir\":\"" + String(is_rev ? "REVERSE" : "FORWARD") + "\",";
    
    json += "\"rpm_h\":\"0x" + String(rpm_h, HEX) + "\",";
    json += "\"rpm_l\":\"0x" + String(rpm_l, HEX) + "\",";
    json += "\"steer_h\":\"0x" + String(steer_h, HEX) + "\",";
    json += "\"steer_l\":\"0x" + String(steer_l, HEX) + "\",";
    json += "\"u_state\":\"" + String(u_state) + "\",";
    json += "\"three_speed\":\"" + speed_mode + "\",";

    json += "\"t_rpm\":\"" + String(current_target_rpm) + "\",";
    json += "\"t_steer\":\"" + String(current_target_steer) + "\",";
    json += "\"t_brake\":\"" + String(current_target_brake) + "\",";
    json += "\"t_mode\":\"" + String(current_target_mode) + "\",";
    json += "\"rx_hex\":\"" + last_rx_hex + "\",";
    json += "\"tx_hex\":\"" + last_tx_hex + "\",";
    json += "\"sys_logs\":\"" + systemLogBuffer + "\""; 
    json += "}"; 

    systemLogBuffer = ""; 
    return json;
}

void initWebServer() {
    WiFi.softAP(ssid, password);
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *req){ req->send(200, "text/html", index_html); });
    server.on("/data", HTTP_GET, [](AsyncWebServerRequest *req){ req->send(200, "application/json", getTelemetryJSON()); });
    server.begin();
}

void WebServerTask(void *pvParameters) { 
    initWebServer(); 
    for(;;) vTaskDelay(100 / portTICK_PERIOD_MS); 
}
```

### VCS_Simulation\vcs_web.h

```plaintext
// VCS_Simulation\vcs_web.h
#ifndef VCS_WEB_H
#define VCS_WEB_H

#include <Arduino.h>
#include "vcs_constants.h"

// C++ Logging Bridge
void vcs_log(String msg);
String getSystemLogs();

void initWebServer();
void WebServerTask(void *pvParameters);

#endif // VCS_WEB_H
```

### VCS_Speed\vcs_reverse.cpp

```cpp
// VCS_Speed\vcs_reverse.cpp
#include "vcs_reverse.h"
#include "vcs_pins.h"
#include "vcs_hallsensor.h" 
#include "vcs_state_machine.h" 
#include "vcs_uart.h"

static bool reverseEngaged = false;

void initReverse() {
    // PCB has a physical 10k pull-UP. Configure as plain INPUT to prevent fighting the PCB.
    pinMode(PIN_REVERSE_IN, INPUT);
}

void updateReverse() {
    bool manualReverseRequested = isReverseSwitchPressed();
    bool autoReverseRequested = getANSReverseCommand(); 
    
    int currentRPM = getMeasuredRPM(); 

    static const int      STOPPED_ENTER_RPM    = 3;
    static const int      STOPPED_EXIT_RPM     = 8;
    static const uint8_t  STOPPED_CONFIRM_TICKS = 5;
    static bool           isStopped     = true; 
    static uint8_t        stoppedCounter = 0;

    bool rawStopped = isStopped
                        ? (currentRPM <  STOPPED_EXIT_RPM)   
                        : (currentRPM <  STOPPED_ENTER_RPM); 

    if (rawStopped != isStopped) {
        if (stoppedCounter < STOPPED_CONFIRM_TICKS) stoppedCounter++;
        if (stoppedCounter >= STOPPED_CONFIRM_TICKS) {
            isStopped = rawStopped;
            stoppedCounter = 0;
        }
    } else {
        stoppedCounter = 0;
    }
    
    bool isManual = (currentState == MANUAL_STATE || currentState == IDLE_STATE);
    bool isAuto = (currentState == AUTONOMOUS_STATE);

    bool triggerReverse = false;

    if (isStopped) {
        if (isManual && manualReverseRequested) {
            triggerReverse = true;
        } 
        else if (isAuto && autoReverseRequested) {
            triggerReverse = true;
        }
    }

    // On ESP32, the physical switch directly pulls the TXB0108 line LOW, 
    // so we only track the logical state here for telemetry.
    if (triggerReverse) {
        reverseEngaged = true;
    } else {
        reverseEngaged = false;
    }
}

bool isReverseEngaged() {
    return reverseEngaged;
}

bool isReverseSwitchPressed() {
    return (digitalRead(PIN_REVERSE_IN) == LOW);
}
```

### VCS_Speed\vcs_reverse.h

```plaintext
// VCS_Speed\vcs_reverse.h
#ifndef VCS_REVERSE_H
#define VCS_REVERSE_H

#include <Arduino.h>

void initReverse();
void updateReverse();
bool isReverseEngaged();
bool isReverseSwitchPressed();

#endif // VCS_REVERSE_H
```

### VCS_Speed\vcs_threespeed.cpp

```cpp
// VCS_Speed\vcs_threespeed.cpp
#include "vcs_threespeed.h"
#include "vcs_pins.h"

DriveMode current_drive_mode = DRIVE_MED; 
static float speed_limit_multiplier = 0.60f; // Default to 60% on boot

void initThreeSpeed() {
    setDriveMode(DRIVE_MED); // Default on boot
}

void updateThreeSpeed() {
    // Hardware switches are deprecated in the ESP32 layout.
    // Lock to 100% capacity; Jetson handles speed limiting via target RPM.
    if (current_drive_mode != DRIVE_HIGH) {
        setDriveMode(DRIVE_HIGH);
    }
    return; // Bypass physical pin polling entirely
}

void setDriveMode(DriveMode mode) {
    current_drive_mode = mode;
    
    switch (mode) {
        case DRIVE_LOW:
            speed_limit_multiplier = 0.30f; // 30% Max Throttle
            break;
        case DRIVE_MED:
            speed_limit_multiplier = 0.60f; // 60% Max Throttle
            break;
        case DRIVE_HIGH:
            speed_limit_multiplier = 1.00f; // 100% Max Throttle
            break;
    }
}

float getMaxThrottleMultiplier() {
    return speed_limit_multiplier;
}
```

### VCS_Speed\vcs_threespeed.h

```plaintext
// VCS_Speed\vcs_threespeed.h
#ifndef VCS_THREESPEED_H
#define VCS_THREESPEED_H

#include <Arduino.h>

enum DriveMode {
    DRIVE_LOW,
    DRIVE_MED,
    DRIVE_HIGH
};

extern DriveMode current_drive_mode;

void initThreeSpeed();
void updateThreeSpeed();
void setDriveMode(DriveMode mode);
float getMaxThrottleMultiplier();

#endif // VCS_THREESPEED_H
```

### VCS_Steering\vcs_steering.cpp

```cpp
// VCS_Steering\vcs_steering.cpp
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
```

### VCS_Steering\vcs_steering.h

```plaintext
// VCS_Steering\vcs_steering.h
#ifndef VCS_STEERING_H
#define VCS_STEERING_H

#include <Arduino.h>
#include <QuickPID.h>
#include "vcs_pins.h"
#include "vcs_constants.h"
#include "vcs_state_machine.h"
#include "vcs_simulation.h"

void initSteering();
uint16_t getMeasuredSteering();
void updateSteeringPID(uint16_t target_position, bool is_automatic);

#endif // VCS_STEERING_H
```

### VCS_System\vcs_state_machine.cpp

```cpp
// VCS_System\vcs_state_machine.cpp
#include "vcs_state_machine.h"
#include "vcs_uart.h"
#include "vcs_pins.h"
#include "vcs_deadman.h"
#include "vcs_lowbrake.h"
#include "vcs_throttle.h"
#include "vcs_reverse.h"
#include "vcs_web.h"
#include "vcs_hallsensor.h"   // getMeasuredRPM()
#include "vcs_steering.h"     // getMeasuredSteering()

// Hall ISR attach/detach lives in vcs_hallsensor.cpp
extern void hall_interrupts_attach();
extern void hall_interrupts_detach();

// Brake helper lives in vcs_lowbrake.cpp / vcs_relays.cpp
extern void forceBrakeEngagement(bool engage);

// =========================================================
// Global state
// =========================================================
VcsState currentState        = INIT_STATE;
uint32_t dmsStartTime        = 0;   // Marks beginning of DMS hold for MANUAL→AUTO promotion
uint32_t stoppingStartTime   = 0;   // Marks entry into STOPPING_STATE

// =========================================================
// INIT
// =========================================================
void initState_Machine() {
    currentState      = INIT_STATE;
    dmsStartTime      = 0;
    stoppingStartTime = 0;
}

// =========================================================
// FSM TICK — call from EXACTLY ONE task (task_control).
// =========================================================
void updateStateMachine(uint32_t faults) {
    static VcsState lastState = INIT_STATE;

    // ---------------------------------------------------------
    // 1. PRIORITY SAFETY OVERRIDES
    //    Heartbeat loss only forces FAULT while in AUTONOMOUS;
    //    losing the link during MANUAL is non-fatal by design.
    // ---------------------------------------------------------
    if (faults != VCS_FAULT_NONE ||
        (!ansHeartbeatReceived() && currentState == AUTONOMOUS_STATE)) {
        currentState = FAULT_STATE;
    }

    // ---------------------------------------------------------
    // 2. STATE TRANSITION LOGIC
    // ---------------------------------------------------------
    switch (currentState) {

        case INIT_STATE:
            if (millis() > 2000) currentState = IDLE_STATE;
            break;

        case IDLE_STATE:
            // ESP32 owns the car. Once init is complete, default to
            // MANUAL — we don't need the Jetson up just to let a human drive.
            currentState = MANUAL_STATE;
            break;

        case MANUAL_STATE:
            // DMS is the primary precondition for MANUAL→AUTONOMOUS.
            if (isDeadmanActive() && !isReverseEngaged()) {
                if (getANSCommandMode() == 1) {  // Jetson is requesting AUTO
                    if (dmsStartTime == 0) {
                        dmsStartTime = millis();
                        vcs_log("DMS HELD: 1s countdown for AUTO...");
                    }
                    if (millis() - dmsStartTime > DMS_HOLD_REQUIRED_MS) {
                        currentState = AUTONOMOUS_STATE;
                        dmsStartTime = 0;
                        vcs_log("FSM: -> AUTONOMOUS");
                    }
                } else {
                    static uint32_t lastLogTime = 0;
                    if (millis() - lastLogTime >= 2000) {
                        vcs_log("WAITING: Jetson must send AUTO (mode=1)");
                        lastLogTime = millis();
                    }
                }
            } else {
                dmsStartTime = 0;
            }
            break;

        case AUTONOMOUS_STATE:
            // Hard physical overrides → MANUAL
            if (isThrottlePedalPressed() || !isDeadmanActive()) {
                currentState = MANUAL_STATE;
                vcs_log("SAFETY: physical override -> MANUAL");
            }
            // Brake / stop-line → STOPPING
            else if (isPhysicalBrakePressed() || isJetsonStopLineActive()) {
                currentState      = STOPPING_STATE;
                stoppingStartTime = millis();
                vcs_log("FSM: brake / stop-line -> STOPPING");
            }
            // Soft override (Jetson asked to exit AUTO, or link lost)
            else if (getANSCommandMode() == 2 || !ansHeartbeatReceived()) {
                currentState = MANUAL_STATE;
                vcs_log("LINK: Jetson soft-exit / heartbeat lost -> MANUAL");
            }
            break;

        case STOPPING_STATE:
            // 3 s elapsed and stop-line cleared → resume AUTONOMOUS
            if ((millis() - stoppingStartTime >= 3000) && !isJetsonStopLineActive()) {
                currentState = AUTONOMOUS_STATE;
                vcs_log("FSM: stop cleared -> AUTONOMOUS");
            }
            // Driver intervention during stop → MANUAL
            else if (!isDeadmanActive() || isThrottlePedalPressed()) {
                currentState = MANUAL_STATE;
                vcs_log("SAFETY: override during STOPPING -> MANUAL");
            }
            break;

        case FAULT_STATE:
            // Recover only when ANS link is restored AND driver hands are off.
            if (ansHeartbeatReceived()) {
                currentState = IDLE_STATE;
            }
            break;
    }

    // ---------------------------------------------------------
    // 3. ENTRY ACTIONS (run once per state change)
    //    NOTE: this block must live OUTSIDE the switch above —
    //    if it sits after the last case it becomes unreachable.
    // ---------------------------------------------------------
    if (currentState != lastState) {

        // Engage brake whenever we leave a driving state
        if (!isDrivingState()) {
            forceBrakeEngagement(true);
        }

        // Hall ISR management:
        //   - Attach the first time we leave INIT (motor controller is up by now).
        //   - Detach on FAULT (we don't trust the hardware).
        //   - DO NOT detach on STOPPING — we still need RPM to verify zero speed.
        if (currentState == IDLE_STATE && lastState == INIT_STATE) {
            hall_interrupts_attach();
        } else if (currentState == FAULT_STATE) {
            hall_interrupts_detach();
        }

        Serial.print(F("VCS_STATE_MACHINE: -> "));
        Serial.println(getStateName(currentState));
        lastState = currentState;
    }
}

// =========================================================
// FAULT INJECTION
// =========================================================
void requestSoftwareEstop() {
    // No latched ESTOP state exists yet — route through FAULT for now.
    // FAULT is recoverable, so add a latched flag if true E-stop is needed.
    currentState = FAULT_STATE;
    vcs_log("ESTOP requested -> FAULT_STATE");
}

uint32_t getSystemFaults() {
    uint32_t f   = VCS_FAULT_NONE;
    uint32_t now = millis();
/*
    // 1. Heartbeat
    if (now - last_uart_time > 500) {
        f |= VCS_FAULT_HEARTBEAT_LOST;
    }

    // 2. Sensor sanity
    
    uint16_t steer = getMeasuredSteering();
    float    rpm   = getMeasuredRPM();
    if (steer < 5 || steer > 1018 || rpm > 600.0f) {
        f |= VCS_FAULT_SENSOR_SPIKE;
    }
*/
    return f;
}

// =========================================================
// HELPERS
// =========================================================
const char* getStateName(VcsState state) {
    switch (state) {
        case INIT_STATE:       return "INIT";
        case IDLE_STATE:       return "IDLE";
        case MANUAL_STATE:     return "MANUAL";
        case AUTONOMOUS_STATE: return "AUTONOMOUS";
        case FAULT_STATE:      return "FAULT";
        case STOPPING_STATE:   return "STOPPING";
        default:               return "UNKNOWN";
    }
}

bool isAutonomousMode()        { return currentState == AUTONOMOUS_STATE; }
uint32_t getDMSHoldStartTime() { return dmsStartTime; }
bool isDrivingState() {
    return (currentState == MANUAL_STATE || currentState == AUTONOMOUS_STATE);
}
```

### VCS_System\vcs_state_machine.h

```plaintext
// VCS_System\vcs_state_machine.h
#ifndef VCS_STATE_MACHINE_H
#define VCS_STATE_MACHINE_H

#include <Arduino.h>

/* ==============================================================================
 * MODULE:         VCS_StateMachine
 * RESPONSIBILITY: Core vehicle safety and state transition logic.
 *                 Revised for Shell Eco-marathon 2026 autonomous rules.
 * ============================================================================== */

// Sidlak Safety Hierarchy
enum VcsState {
    INIT_STATE,        // Power-on self-test & sensor stabilization
    IDLE_STATE,        // Standby; waiting for ANS heartbeat
    MANUAL_STATE,      // Human-in-the-loop control (default drive state)
    AUTONOMOUS_STATE,  // ANS-controlled driving (requires both DMS held)
    FAULT_STATE,       // Software / comms fail-safe (recoverable)
    STOPPING_STATE     // Transient 3 s pause on stop-line / brake assertion
                       // NOTE: a separate latched ESTOP_STATE is not modelled.
                       //       Treat FAULT as the highest unrecoverable state for now.
};

// --- Fault Bit Definitions ---
// Recoverable faults map to FAULT_STATE and clear when the underlying
// condition resolves.
#define VCS_FAULT_NONE               0x00000000u
#define VCS_FAULT_UART_CRC           0x00000001u   // CRC mismatch on command packet
#define VCS_FAULT_HEARTBEAT_LOST     0x00000002u   // ANS heartbeat timeout
#define VCS_FAULT_SENSOR_SPIKE       0x00000004u   // Hall / steering pot out-of-range
#define VCS_FAULT_OVERCURRENT        0x00000008u   // Motor controller fault line

// --- Safety Timing Constants ---
// (Candidate to relocate into vcs_constants.h.)
#define DMS_HOLD_REQUIRED_MS         1000u         // SEM spec: 1.0 s dual-grip hold

// Global state.
//
// NOTE on concurrency: currentState is currently written by task_control
// (the FSM owner) and read by other tasks. Word-aligned enum reads on
// ESP32 (Xtensa LX6) are single-instruction atomic in practice, so a
// plain extern is acceptable for now. Wrap in std::atomic<VcsState>
// during a future hardening pass if you start writing it from more
// than one task.
extern VcsState currentState;

// --- Initialization & main tick ---
void initState_Machine();
void updateStateMachine(uint32_t externalFaults);

// --- Fault injection ---
// Latch a software E-stop. Currently routes through FAULT_STATE; will
// route to a dedicated latched state once one is added.
void requestSoftwareEstop();
uint32_t getSystemFaults();

// --- Telemetry / display helpers ---
uint32_t    getDMSHoldStartTime();
const char* getStateName(VcsState state);

// --- Logic helpers ---
bool isAutonomousMode();
bool isDrivingState();

#endif // VCS_STATE_MACHINE_H
```

