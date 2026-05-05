# Project Files Export

Export time: 5/6/2026, 1:46:56 AM

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
│   ├── vcs_calibration.h
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

src
|--- main.cpp
```

## File Statistics

- Total files: 30
- Total size: 94.6 KB

### File Type Distribution

| Extension | Files | Total Size |
| --- | --- | --- |
| .h | 16 | 21.3 KB |
| .cpp | 13 | 72.3 KB |
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
#include "vcs_calibration.h"
#include "esp_adc_cal.h"
#include "driver/adc.h"

esp_adc_cal_characteristics_t adc_chars;

uint16_t current_throttle_adc = 0;
uint16_t current_pwm_duty = 0;

float smoothedThrottle = 0.0f;

// FIX 9: file-scope statics — prevent any external writes that
// could race with QuickPID's pointer-based setpoint/input access.
static float measured_rpm     = 0.0f;
static float target_rpm       = 0.0f;
static float throttle_pwm_out = 0.0f;

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

    // FIX 1: sample time MUST match actual call rate (100Hz = 10000us).
    // Previously set to 1000us — caused QuickPID's internal dt math to
    // be off by 10x, making tuned Kp/Ki gains behave incorrectly.
    speedPID.SetSampleTimeUs(10000);

    speedPID.SetMode(QuickPID::Control::manual);
}

void updateThrottle(float current_rpm_in, float target_rpm_in) {
    // --- EMA FILTER INJECTION ---
    uint32_t sum = 0;
    for (int i = 0; i < 16; i++) sum += adc1_get_raw(ADC1_CHANNEL_6);
    uint32_t pedal_mv = esp_adc_cal_raw_to_voltage(sum / 16, &adc_chars);
    int rawThrottle = map(pedal_mv, 0, 3300, 0, 1023); 

    smoothedThrottle = (THROTTLE_EMA_ALPHA * (float)rawThrottle)
                     + ((1.0f - THROTTLE_EMA_ALPHA) * smoothedThrottle);
    current_throttle_adc = (uint16_t)smoothedThrottle;

    // --- FETCH HARDWARE SPEED LIMIT ---
    float speed_multiplier = getMaxThrottleMultiplier(); 
    int dynamic_max_pwm = MIN_PWM_OUT + (int)((MAX_PWM_OUT - MIN_PWM_OUT) * speed_multiplier);

    speedPID.SetOutputLimits(MIN_PWM_OUT, dynamic_max_pwm);

    // --- 1. HARDWARE SAFETY LOCKOUT ---
    bool isBrakePressed = (digitalRead(PIN_LOWBRAKE_IN) == LOW);

    if ((currentState != AUTONOMOUS_STATE && currentState != MANUAL_STATE) || isBrakePressed) {
        current_pwm_duty = MIN_PWM_OUT;

        // FIX 3: use the named pin constant, not magic number 25
        int dac_val = map(current_pwm_duty, 0, 1023, 0, 255);
        dacWrite(PIN_THROTTLE_OUT, constrain(dac_val, 0, 255));

        // FIX 2: clear PID state on transition to manual.
        // QuickPID's SetMode(manual) does NOT clear the integral —
        // Initialize() does. Without this, the next AUTONOMOUS entry
        // inherits stale integral and surges the actuator.
        if (speedPID.GetMode() != (uint8_t)QuickPID::Control::manual) {
            speedPID.SetMode(QuickPID::Control::manual);
            speedPID.Initialize();
        }
        throttle_pwm_out = MIN_PWM_OUT;
        return; 
    }

    // --- 2. AUTONOMOUS CONTROL (PID) ---
    if (currentState == AUTONOMOUS_STATE) {
        if (speedPID.GetMode() == (uint8_t)QuickPID::Control::manual) {
            // Bumpless transfer: seed PID output with current pwm
            // before flipping to automatic, so the actuator doesn't jump.
            throttle_pwm_out = (float)current_pwm_duty;
            speedPID.SetMode(QuickPID::Control::automatic);
        }

        measured_rpm = current_rpm_in;
        target_rpm   = target_rpm_in;

        if (consumeNewRPMSample() && speedPID.Compute()) {
            current_pwm_duty = (uint16_t)throttle_pwm_out;
            int dac_val = map(current_pwm_duty, 0, 1023, 0, 255);
            dacWrite(PIN_THROTTLE_OUT, constrain(dac_val, 0, 255));   // FIX 3
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
        dacWrite(PIN_THROTTLE_OUT, constrain(dac_val, 0, 255));       // FIX 3
        
        throttle_pwm_out = mapped_pwm;

        // FIX 2: clear PID state when entering manual from autonomous
        if (speedPID.GetMode() != (uint8_t)QuickPID::Control::manual) {
            speedPID.SetMode(QuickPID::Control::manual);
            speedPID.Initialize();
        }
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
// ============================================================
//  vcs_uart.cpp — Rev fixes applied
//
//  CHANGES FROM PREVIOUS FIX REVISION:
//
//  FIX #13 — Stale display data cleared on link loss.
//             last_rx_hex and all target values were never
//             cleared when the Jetson disconnected. The web
//             page showed the last received packet forever —
//             including at boot with nothing connected.
//             handleIncomingUART() now clears all targets
//             and sets last_rx_hex to "-- NO LINK --" the
//             moment ansHeartbeatReceived() returns false.
//             target_steering clears to 500 (protocol neutral
//             midpoint), NOT 0 (which would command full-left).
//
//  All other fixes from previous revision retained unchanged:
//  FIX #1  last_valid_packet_time initialized to 0 not millis()
//  FIX #5  readBytes guarded with available() check
//  FIX #6  sync byte consume bug fixed
//  FIX #7  String += replaced with snprintf + single assignment
//  FIX #8  legacy mirrors written inside uartMux critical section
//  FIX #9  isJetsonStopLineActive() removed
//  FIX #12 broadcastVehicleTelemetry() takes uint8_t gear param
// ============================================================

#include "vcs_uart.h"
#include "vcs_hallsensor.h"
#include "vcs_steering.h"
#include "vcs_simulation.h"
#include "vcs_state_machine.h"
#include "vcs_pins.h"
#include "vcs_reverse.h"
#include "vcs_web.h"

static constexpr bool UART_DEBUG_LOGS = true;

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
// =========================================================

static portMUX_TYPE uartMux = portMUX_INITIALIZER_UNLOCKED;

static volatile uint8_t  target_mode              = 0;
static volatile float    target_rpm               = 0.0f;
static volatile uint16_t target_steering          = 500;   // 500 = protocol neutral
static volatile uint8_t  target_brake             = 0;
static volatile bool     target_direction_reverse = false;
static volatile uint32_t last_valid_packet_time   = 0;

extern String last_rx_hex;
extern String last_tx_hex;

// Legacy mirrors — written inside uartMux critical section (FIX #8)
uint8_t  current_target_brake = 0;
int16_t  current_target_rpm   = 0;
uint16_t current_target_steer = 500;   // 500 = protocol neutral
uint8_t  current_target_mode  = 0;
uint32_t last_uart_time       = 0;

static uint16_t calculateCRC16(const uint8_t *data, uint8_t length);
static void     processCommandPacket(const uint8_t *packet);

// =========================================================
// INIT
// =========================================================
void initUART() {
    Serial2.begin(115200, SERIAL_8N1, JETSON_RX_PIN, JETSON_TX_PIN);
    Serial2.setTimeout(20);
    portENTER_CRITICAL(&uartMux);
    // FIX #1: Initialize to 0, NOT millis().
    // Prevents ansHeartbeatReceived() returning true at boot
    // before any packet has ever been received.
    last_valid_packet_time = 0;
    portEXIT_CRITICAL(&uartMux);
    last_uart_time = millis();
}

// =========================================================
// RX PARSER
// =========================================================
void handleIncomingUART() {
    // FIX #13 ORDER: Process all waiting packets FIRST, then check
    // heartbeat. The original code checked heartbeat BEFORE the while
    // loop — if a packet was sitting in the buffer at the exact moment
    // the 500ms window expired, the clear fired first (setting
    // "-- NO LINK --"), then the while loop immediately processed the
    // waiting packet. The web page caught the brief stale state and
    // showed a blink. Processing packets first means last_valid_packet_time
    // is updated before the heartbeat check runs.

    while (Serial2.available() >= 14) {
        // Scan for AA 55 sync — discard junk until found
        if (Serial2.peek() != 0xAA) {
            Serial2.read();
            continue;
        }
        Serial2.read();   // consume 0xAA

        // FIX #6: If byte after 0xAA is not 0x55, not our frame.
        // Consume the junk byte unless it is 0xAA (new frame start).
        if (Serial2.peek() != 0x55) {
            if (Serial2.peek() != 0xAA) {
                Serial2.read();
            }
            continue;
        }
        Serial2.read();   // consume 0x55

        // FIX #5: Verify all 12 remaining bytes are buffered before
        // calling readBytes(). Prevents the 20ms setTimeout block
        // on a partial frame.
        if (Serial2.available() < 12) {
            break;
        }

        uint8_t pkt[14];
        pkt[0] = 0xAA;
        pkt[1] = 0x55;
        if (Serial2.readBytes(&pkt[2], 12) != 12) continue;

        if (pkt[13] != 0xFF) {
            vcs_log("UART: bad footer");
            continue;
        }

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

        last_uart_time = millis();
        portENTER_CRITICAL(&uartMux);
        last_valid_packet_time = last_uart_time;
        portEXIT_CRITICAL(&uartMux);

        // FIX #7: snprintf into fixed char buffer, single String assignment.
        // Avoids repeated heap allocation from String +=.
        {
            char rxHexBuf[14 * 3 + 1];
            char *p = rxHexBuf;
            for (int i = 0; i < 14; i++) {
                p += sprintf(p, "%02X ", pkt[i]);
            }
            last_rx_hex = String(rxHexBuf);
            last_rx_hex.toUpperCase();
        }

        processCommandPacket(pkt);
    }

    // FIX #13: Clear stale data AFTER processing all waiting packets.
    // Now that the while loop has run, last_valid_packet_time reflects
    // any packet that was buffered this tick. If the heartbeat is still
    // dead after draining the buffer, the link is genuinely lost.
    // target_steering = 500: protocol neutral — NOT 0 (full-left).
    if (!ansHeartbeatReceived()) {
        portENTER_CRITICAL(&uartMux);
        target_mode              = 0;
        target_rpm               = 0.0f;
        target_steering          = 500;
        target_brake             = 0;
        target_direction_reverse = false;
        current_target_mode      = 0;
        current_target_rpm       = 0;
        current_target_steer     = 500;
        current_target_brake     = 0;
        portEXIT_CRITICAL(&uartMux);
        last_rx_hex = "-- NO LINK --";
    }
}

// =========================================================
// PACKET DISPATCH
// =========================================================
static void processCommandPacket(const uint8_t *pkt) {
    uint8_t msgType = pkt[2];
    if (msgType != 0x01) return;

    uint8_t  mode      =  pkt[4];
    int16_t  rpm       =  (int16_t)(((uint16_t)pkt[5] << 8) | pkt[6]);
    uint16_t steer     =  ((uint16_t)pkt[7] << 8) | pkt[8];
    uint8_t  brake     =  pkt[9];
    bool     reverseOn = (pkt[10] == 1);

    // FIX #8: All writes inside one critical section.
    // Legacy mirrors previously written outside the mutex,
    // creating a race if any task read them concurrently.
    portENTER_CRITICAL(&uartMux);
    target_mode              = mode;
    target_rpm               = (float)constrain(rpm,   COMM_SPEED_MIN,  COMM_SPEED_MAX);
    target_steering          = (uint16_t)constrain(steer, COMM_STEER_LEFT, COMM_STEER_RIGHT);
    target_brake             = (uint8_t) constrain(brake, COMM_BRAKE_MIN,  COMM_BRAKE_MAX);
    target_direction_reverse = reverseOn;
    current_target_mode      = mode;
    current_target_rpm       = (int16_t)constrain(rpm,   COMM_SPEED_MIN,  COMM_SPEED_MAX);
    current_target_steer     = (uint16_t)constrain(steer, COMM_STEER_LEFT, COMM_STEER_RIGHT);
    current_target_brake     = (uint8_t) constrain(brake, COMM_BRAKE_MIN,  COMM_BRAKE_MAX);
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
// TELEMETRY TX
// FIX #12: gear passed as parameter. Caller (UITask) computes
// gear from current_drive_mode so uart does not depend on
// vcs_threespeed. Update vcs_uart.h signature to match.
// =========================================================
void broadcastVehicleTelemetry(uint8_t gear) {
    float    rpm;
    uint16_t steer;
    uint8_t  state = (uint8_t)currentState;
    uint8_t  rev   = isReverseEngaged() ? 1 : 0;

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

    uint8_t buf[9];
    buf[0] = 0x02;
    buf[1] = 0x07;
    buf[2] = (uint8_t)((rpmInt >> 8) & 0xFF);
    buf[3] = (uint8_t)( rpmInt       & 0xFF);
    buf[4] = (uint8_t)((steer  >> 8) & 0xFF);
    buf[5] = (uint8_t)( steer        & 0xFF);
    buf[6] = state;
    buf[7] = gear;
    buf[8] = rev;

    uint16_t crc = calculateCRC16(buf, 9);

    uint8_t txDebug[14] = {
        0xAA, 0x55, buf[0], buf[1], buf[2], buf[3], buf[4],
        buf[5], buf[6], buf[7], buf[8],
        (uint8_t)((crc >> 8) & 0xFF), (uint8_t)(crc & 0xFF), 0xFF
    };

    if (UART_DEBUG_LOGS) {
        printHexDebug("TX [ESP32 -> JETSON]: ", txDebug, 14);
    }

    Serial2.write((uint8_t)0xAA);
    Serial2.write((uint8_t)0x55);
    Serial2.write(buf, 9);
    Serial2.write((uint8_t)((crc >> 8) & 0xFF));
    Serial2.write((uint8_t)( crc       & 0xFF));
    Serial2.write((uint8_t)0xFF);

    // FIX #7: Single snprintf build, single String assignment.
    {
        char txHexBuf[14 * 3 + 1];
        char *p = txHexBuf;
        for (int i = 0; i < 14; i++) {
            p += sprintf(p, "%02X ", txDebug[i]);
        }
        last_tx_hex = String(txHexBuf);
        last_tx_hex.toUpperCase();
    }
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
    // FIX #1: t == 0 means no packet ever received.
    // millis() - 0 < 500 during first 500ms of boot — would return
    // true falsely without this guard.
    if (t == 0) return false;
    return (millis() - t) <= 500;
}

// FIX #9: isJetsonStopLineActive() removed.
// State machine checks (getTargetBrake() > 0) directly.
// Remove declaration from vcs_uart.h.

// =========================================================
// UART UPDATE TASK (STUB)
// =========================================================
void updateUART() {
    // Stub — RX: handleIncomingUART() in CommTask (Core 0).
    // TX: broadcastVehicleTelemetry() in UITask (Core 0).
}

// =========================================================
// DEBUG HELPER
// =========================================================
void printHexDebug(const char* prefix, const uint8_t* data, uint8_t length) {
    Serial.print(prefix);
    for (uint8_t i = 0; i < length; i++) {
        if (data[i] < 0x10) Serial.print("0");
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
void broadcastVehicleTelemetry(uint8_t gear);

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

### VCS_Config\vcs_calibration.h

```plaintext
// VCS_Config\vcs_calibration.h
#ifndef VCS_CALIBRATION_H
#define VCS_CALIBRATION_H

// ============================================================
//  vcs_calibration.h — SIDLAK 2 VCS
//  Team Wired PH0017003 | Shell Eco-marathon 2026
//
//  All values in this file require physical measurement or
//  bench/track tuning. Nothing here is a fixed system constant.
//
//  WORKFLOW:
//    1. Measure physical values (ADC endpoints, Hall count).
//    2. Update defines here.
//    3. Re-run wizard to confirm on car.
//    4. Tune PID/PI/EMA gains last, after mechanical cal is done.
//
//  DO NOT put protocol ranges, task frequencies, or fault codes
//  here — those belong in vcs_constants.h.
//
//  SECTIONS:
//    1. Motor & Hall sensor
//    2. Throttle ADC calibration
//    3. Throttle module tuning (EMA, PI)
//    4. Steering pot ADC calibration
//    5. Steering module tuning (EMA, PID)
//    6. Brake actuator calibration
// ============================================================


// ============================================================
//  SECTION 1 — MOTOR & HALL SENSOR
//
//  *** CRITICAL DISCREPANCY — RESOLVE BEFORE FIRST LIVE RUN ***
//
//  MOTOR_POLE_PAIRS = 23 with HALL_TRANSITIONS_PER_MECH_REV = 46
//  assumes a SINGLE Hall sensor (Hall C, GPIO32 only) with a
//  CHANGE interrupt: transitions/rev = pole_pairs × 2 = 23 × 2 = 46.
//
//  An earlier spec listed 138 transitions (= 6 × 23), which assumes
//  THREE Hall sensors. GPIO36/39 are removed from this design due
//  to ESP32 ECO 3.11 silicon bug. 138 is wrong for this hardware.
//
//  FIELD MEASUREMENT PROCEDURE (wizard Step 10):
//    1. 'hall reset'
//    2. Spin wheel exactly ONE full mechanical revolution.
//    3. 'hall status' → read Pulses field.
//    4. That count IS HALL_TRANSITIONS_PER_MECH_REV.
//    5. pole_pairs = measured_transitions / 2
//    6. Update both constants below and reflash.
// ============================================================
#define MOTOR_POLE_PAIRS              23   // *** VERIFY via wizard Step 10 ***
#define HALL_TRANSITIONS_PER_MECH_REV 46   // *** VERIFY — see note above ***
#define GEAR_REDUCTION                1.0f // Direct drive; update if gearbox fitted


// ============================================================
//  SECTION 2 — THROTTLE ADC CALIBRATION
//
//  All values are millivolts from esp_adc_cal.
//  Never compare against raw analogRead() counts.
//
//  HOW TO MEASURE:
//    'monitor throttle' in Serial Monitor.
//    Record mV at: zero-pedal (deadband), pedal-just-active,
//    and full-press. Update the three values below.
// ============================================================
#define THROTTLE_DEADBAND_MV         600   // Zero-pedal reads ~551mV; add margin
#define THROTTLE_MIN_INPUT_MV        650   // Pedal-just-active threshold
#define THROTTLE_MAX_INPUT_MV       3000   // Full-press mV — verify with 'monitor throttle'


// ============================================================
//  SECTION 3 — THROTTLE MODULE TUNING
//
//  THROTTLE_EMA_ALPHA:
//    Controls how aggressively the throttle ADC reading is
//    smoothed. Range 0.0–1.0.
//    Lower = smoother but slower to respond (more lag).
//    Higher = faster response but more ADC noise passes through.
//    0.15 is a typical starting point for a foot pedal.
//
//  SPEED_KP / SPEED_KI:
//    PI gains for the RPM → DAC control loop.
//    Output maps to 0–255 (8-bit DAC → LM358 → motor controller).
//
//  TUNING SEQUENCE (bench, no load first):
//    1. KI = 0. Raise KP until motor tracks step RPM changes
//       without large overshoot.
//    2. Add KI to eliminate steady-state RPM offset.
//       Keep KI small — integrator windup causes runaway.
//    3. Re-verify at full target RPM (COMM_SPEED_MAX = 3000).
// ============================================================
#define THROTTLE_EMA_ALPHA           0.15f  // ADC smoothing — see note above

#define SPEED_KP                     0.20f  // Starting point — retune on bench
#define SPEED_KI                     0.0375f


// ============================================================
//  SECTION 4 — STEERING POT ADC CALIBRATION
//
//  3590S pot powered at 3.3V. Values are calibrated ADC mV.
//  All three must be measured on car with wizard Steps 5–7.
//
//  HOW TO MEASURE:
//    Turn wheel to full LEFT  → 'set full_l' → note mV printed.
//    Turn wheel to full RIGHT → 'set full_r' → note mV printed.
//    Center steering          → 'set center' → note mV printed.
//    Update below and reflash.
// ============================================================
#define STEER_POT_MIN_MV            200    // PLACEHOLDER — run wizard Step 5
#define STEER_POT_CENTER_MV        1666    // PLACEHOLDER — run wizard Step 7
#define STEER_POT_MAX_MV           2766    // PLACEHOLDER — run wizard Step 6


// ============================================================
//  SECTION 5 — STEERING MODULE TUNING
//
//  STEER_EMA_ALPHA:
//    EMA smoothing for the steering pot ADC read.
//    Lower = smoother but adds lag to steering response.
//    Higher = more responsive but more noise reaches the PID.
//    0.15 matches the throttle alpha — adjust independently
//    if steering feels laggy or jittery.
//
//  STEER_KP / STEER_KI / STEER_KD:
//    PID gains for the stepper steering loop.
//    Error is in COMM units (0–1000 across full steering travel).
//    Setpoint and input are both in COMM units so the deadzone
//    is in the same space.
//
//  TUNING SEQUENCE (car, after pot calibration is done):
//    1. KI = 0, KD = 0. Raise KP until wheel reaches target.
//    2. If oscillating: lower KP slightly.
//    3. Add KD to dampen overshoot. Raise until smooth.
//    4. Add small KI only if steady-state offset remains.
//       Keep near 0 — integrator windup causes continuous drift.
//
//  STEER_DEADZONE:
//    Minimum error in COMM units before PID issues a correction.
//    Increase if stepper hunts (oscillates around target).
//    Decrease for tighter tracking at cost of more motor activity.
// ============================================================
#define STEER_EMA_ALPHA              0.15f  // ADC smoothing — see note above
#define STEER_KP                     1.2f
#define STEER_KI                     0.05f
#define STEER_KD                     0.01f
#define STEER_DEADZONE               5      // COMM units — increase if hunting


// ============================================================
//  SECTION 6 — BRAKE ACTUATOR CALIBRATION
//
//  TB6612FNG driving a 12V linear actuator.
//  Both values require physical timing on the car.
//
//  HOW TO CALIBRATE BRAKE_RETRACT_MS (wizard Step 3):
//    1. 'actuator extend' until limit switch stops it.
//    2. 'set retract_ms 600' then 'actuator retract'.
//    3. Observe — did it fully retract?
//       NO  → increase by 50ms and retry.
//       YES → decrease by 50ms until barely fully retracts.
//    4. Add 50ms safety margin to that minimum.
//    5. Update BRAKE_RETRACT_MS below.
// ============================================================
#define BRAKE_PWM                    255   // 0–255 to TB6612; 255 = full 12V (bench confirmed)
#define BRAKE_RETRACT_MS             2400   // PLACEHOLDER — calibrate via wizard Step 3


#endif // VCS_CALIBRATION_H
```

### VCS_Config\vcs_constants.h

```plaintext
// VCS_Config\vcs_constants.h
#ifndef VCS_CONSTANTS_H
#define VCS_CONSTANTS_H

#include <Arduino.h>

// ============================================================
//  vcs_constants.h — SIDLAK 2 VCS
//  Team Wired PH0017003 | Shell Eco-marathon 2026
//
//  Fixed system constants only — build flags, logic level,
//  task frequencies, protocol ranges, fault codes, and
//  legacy bridging macros.
//
//  CALIBRATABLE VALUES (PID gains, ADC endpoints, brake timing,
//  pot measurements, hall transitions) have been moved to:
//      vcs_calibration.h
//
//  This file includes vcs_calibration.h at the bottom, so all
//  existing #include "vcs_constants.h" in the codebase continue
//  to work without modification.
// ============================================================


// ============================================================
//  SYSTEM ARCHITECTURE & BUILD FLAGS
// ============================================================
#define SIMULATION_MODE 0   // 1 = Digital Twin, 0 = LIVE 1500W motor control
#define V_LOGIC         3.3f  // ESP32 logic level — used by ADC scaling helpers

#if SIMULATION_MODE
  #pragma message ("VCS BUILD >>> SIMULATION_MODE = 1  (Digital Twin, no live motor output)")
#else
  #pragma message ("VCS BUILD >>> SIMULATION_MODE = 0  (LIVE 1500W MOTOR CONTROL - verify E-stop)")
#endif

// ============================================================
//  PHYSICAL CONSTANTS
#define WHEEL_CIRCUMFERENCE_M 1.2764f


// ============================================================
//  TASK FREQUENCIES & TIMING
//  These are system architecture decisions, not tunable values.
//  Changing them affects FreeRTOS task scheduling — do not
//  adjust without reviewing all vTaskDelay() calls.
// ============================================================
#define FREQ_CONTROL_HZ    1000   // Core control loop    (1ms tick)
#define FREQ_STEER_CTRL_HZ  100   // Steering inner loop  (10ms tick)
#define FREQ_COMM_HZ        100   // Comms / inputs sweep (10ms tick)
#define FREQ_UI_HZ           20   // OLED + telemetry     (50ms tick)
#define DEBOUNCE_TIME_MS     50   // Physical brake switch debounce window


// ============================================================
//  COMMUNICATION PROTOCOL RANGES (Jetson → VCS)
//  These are the agreed protocol values between ESP32 and
//  Jetson. Do not change without updating both ends.
// ============================================================
#define COMM_SPEED_MIN      0
#define COMM_SPEED_MAX      3000    // Maximum target RPM from Jetson
#define COMM_STEER_LEFT     0
#define COMM_STEER_CENTER   500
#define COMM_STEER_RIGHT    1000
#define COMM_BRAKE_MIN      0
#define COMM_BRAKE_MAX      1       // Binary: 0 = off, 1 = on


// ============================================================
//  FAULT CODES
// ============================================================
#define VCS_FAULT_NONE        0x0000

// Jetson command timeout in autonomous mode.
// Triggered when no valid UART packet arrives within
// SIGNAL_TIMEOUT_MS (defined in main.cpp).
constexpr uint16_t FAULT_SIGNAL_TIMEOUT = 0x0004;


// ============================================================
//  LEGACY BRIDGING MACROS
//  Speed PI was originally written for a 0–1023 PWM output.
//  These let legacy throttle PID code compile unchanged while
//  the actual output path uses the 0–255 DAC range.
//  Do not use in new code — reference THROTTLE_MIN/MAX_INPUT_MV
//  from vcs_calibration.h directly.
// ============================================================
#define MIN_PWM_OUT       0
#define MAX_PWM_OUT       1023
#define THROTTLE_MIN_INPUT 650    // Legacy alias — use THROTTLE_MIN_INPUT_MV
#define THROTTLE_MAX_INPUT 3000   // Legacy alias — use THROTTLE_MAX_INPUT_MV


// ============================================================
//  CALIBRATABLE VALUES
//  Moved to vcs_calibration.h. Included here so all existing
//  #include "vcs_constants.h" continue to compile unchanged.
// ============================================================
#include "vcs_calibration.h"


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
#include "vcs_constants.h"

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
// ============================================================
//  vcs_web.cpp — fixes applied
//
//  CHANGES FROM ORIGINAL:
//
//  FIX #13a — last_rx_hex and last_tx_hex initialized to
//              "-- NO LINK --" instead of "00 00 00 00 ...".
//              The old initialization looked identical to a
//              valid all-zeros packet on the web dashboard.
//
//  FIX #13b — getTelemetryJSON() now checks ansHeartbeatReceived()
//              before including Jetson target values. When link
//              is dead, t_rpm / t_steer / t_brake / t_mode are
//              reported as "--" instead of stale numeric values.
//              A "jetson_link" boolean field is added to the JSON
//              so the frontend can react independently.
//
//  FIX #13c — Dashboard HTML: Jetson panel header now shows a
//              live ● LINKED / ● NO LINK indicator that updates
//              every poll cycle (200ms). Jetson target fields
//              and hex stream fields are blanked to "--" in
//              JavaScript when jetson_link is false — even if
//              the server sends stale numeric data.
//              This is a client-side redundant guard on top of
//              the server-side clear in FIX #13 (vcs_uart.cpp).
// ============================================================

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

extern int16_t  current_target_rpm;
extern uint16_t current_target_steer;
extern uint8_t  current_target_brake;
extern uint8_t  current_target_mode;

const char* ssid     = "SIDLAK_VCS_LIVE";
const char* password = "sidlak_secure";

// FIX #13a: Initialize to "-- NO LINK --" not "00 00 00 ...".
// The all-zeros string looked like a valid packet on the dashboard.
String last_rx_hex = "-- NO LINK --";
String last_tx_hex = "-- NO LINK --";

#define WHEEL_CIRCUMFERENCE_M  1.2764f
#define MAX_STEERING_ANGLE_DEG 35.0f

AsyncWebServer server(80);
String systemLogBuffer = "";

// --- NEW ADDITIONS START ---
AsyncWebSocket ws("/ws");

// HARDCODED RACE MODE TOGGLE: 
// 0 = Web & WiFi ON (Pit Mode)
// 1 = Web & WiFi OFF (Race Mode - maximum power saving)
const int DISABLE_WEB_WIFI = 0; 
// --- NEW ADDITIONS END ---


void vcs_log(String msg) {
    Serial.println("[VCS LOG] " + msg);
    systemLogBuffer += msg + "|";
}

// ============================================================
//  HTML — FIX #13c: added jetson link status indicator and
//  client-side guard that blanks Jetson fields when no link.
//  All other layout and styling unchanged.
// ============================================================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>SIDLAK 2: LIVE VCS DASHBOARD</title>
    <style>
        body { font-family: 'Courier New', Courier, monospace; background-color: #050505; color: #00FF00; padding: 20px; }
        .grid-container { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; max-width: 1200px; margin: auto; }
        .panel { border: 1px solid #00FF00; padding: 15px; background: #000; box-shadow: 0 0 10px rgba(0,255,0,0.2); }
        h3 { margin-top: 0; color: #00FF00; border-bottom: 1px solid #00FF00; padding-bottom: 5px; text-transform: uppercase; }
        table { width: 100%; border-collapse: collapse; margin-top: 10px; }
        th, td { border: 1px solid #333; padding: 5px 10px; text-align: left; }
        button { background-color: #000; color: #00FF00; border: 1px solid #00FF00; padding: 10px; cursor: pointer; font-weight: bold; width: 100%; margin-bottom: 10px; }
        button:hover { background-color: #00FF00; color: #000; }
        
        /* Dynamic State Banner */
        #state_banner { padding: 15px; text-align: center; font-size: 1.8em; font-weight: bold; border: 2px solid #00FF00; margin-bottom: 20px; }
        .state-idle { color: #FFFF00; border-color: #FFFF00; }
        .state-auto { color: #00FFFF; border-color: #00FFFF; }
        .state-error { color: #FF0000; border-color: #FF0000; animation: blink 1s infinite; }
        
        /* 14-Byte Inspector */
        .packet-grid { display: grid; grid-template-columns: repeat(14, 1fr); gap: 5px; text-align: center; margin-top: 10px; }
        .byte-box { border: 1px solid #555; padding: 5px; background: #111; font-size: 0.9em; transition: background-color 0.1s; }
        .byte-label { font-size: 0.6em; color: #888; display: block; margin-bottom: 3px; }
        .byte-val { font-weight: bold; }
        .byte-header { border-color: #00FFFF; color: #00FFFF; }
        .byte-crc { border-color: #FFFF00; color: #FFFF00; }
        
        @keyframes blink { 50% { opacity: 0.5; } }
    </style>
</head>
<body>
    <div id="state_banner" class="state-idle">WAITING FOR TELEMETRY...</div>

    <div class="grid-container">
        <div class="panel">
            <h3>Visual Telemetry</h3>
            <div style="display:flex; justify-content:space-between;">
                <div>
                    <span style="color:#00FF00;">Actual RPM</span> | <span style="color:#ff00ff;">Target RPM</span>
                    <canvas id="rpmChart" width="350" height="150" style="border:1px solid #333; margin-top:5px;"></canvas>
                </div>
                <div style="text-align:center;">
                    <span>Steering Vector</span><br>
                    <canvas id="steerCanvas" width="120" height="120" style="margin-top:15px;"></canvas>
                    <div id="live_steer_val" style="margin-top:5px;">0°</div>
                </div>
            </div>
            <table>
                <tr><td>Live Speed</td><td id="live_speed">--</td><td>Gear/Dir</td><td id="live_dir">--</td></tr>
                <tr><td>FSM State</td><td id="live_state">--</td><td>DMS Safety</td><td id="live_dms">--</td></tr>
            </table>
        </div>
        
        <div class="panel">
            <h3>Data Logger (RAM Buffer)</h3>
            <button id="recBtn" onclick="toggleRecording()">START 20Hz LOGGING</button>
            <button onclick="downloadCSV()">DOWNLOAD CSV</button>
            <div style="font-size: 0.8em; color: #888; margin-top: 10px;">
                Buffer Size: <span id="buf_size">0</span> / 15000<br>
                <span id="auto_dl_warn" style="color:#FF0000; display:none;">AUTO-DOWNLOAD TRIGGERED (LINK LOST)</span>
            </div>
        </div>

        <div class="panel" style="grid-column: span 2;">
            <h3>Jetson UART Protocol Inspector</h3>
            <div class="packet-grid" id="packet_inspector"></div>
            <div style="margin-top: 15px; font-size: 0.9em; display:flex; justify-content:space-between;">
                <div><span style="color:#00FFFF;">Target RPM: </span><span id="calc_rpm">--</span></div>
                <div><span style="color:#00FFFF;">Target Steer: </span><span id="calc_steer">--</span></div>
                <div><span style="color:#00FFFF;">Target Brake: </span><span id="calc_brake">--</span></div>
            </div>
        </div>
    </div>

    <script>
        // Data Logging State
        let isRecording = false;
        let csvRows = [["Timestamp", "FSM_State", "DMS", "Live_RPM", "Target_RPM", "Speed_kmh", "Steer_Deg"]];
        const MAX_ROWS = 15000;
        let lastLinkState = true;

        // Visual Setup
        const labels = ["HDR", "RPM_H", "RPM_L", "STR_H", "STR_L", "MODE", "BRK", "PAD1", "PAD2", "PAD3", "PAD4", "CRC_H", "CRC_L", "FTR"];
        const grid = document.getElementById('packet_inspector');
        let prevBytes = [];
        
        labels.forEach((lbl, i) => {
            let cls = (i===0 || i===13) ? "byte-header" : (i===11 || i===12) ? "byte-crc" : "";
            grid.innerHTML += `<div class="byte-box ${cls}" id="box${i}"><span class="byte-label">${lbl}</span><span class="byte-val" id="b${i}">00</span></div>`;
        });

        // Canvas Setup
        const rpmCtx = document.getElementById('rpmChart').getContext('2d');
        const steerCtx = document.getElementById('steerCanvas').getContext('2d');
        let rpmData = [], trpmData = [];

        function drawRPMGraph() {
            rpmCtx.clearRect(0, 0, 350, 150);
            if(rpmData.length < 2) return;
            let maxVal = Math.max(...rpmData, ...trpmData, 100);
            
            // Draw Target RPM (Pink)
            rpmCtx.beginPath();
            rpmCtx.strokeStyle = '#ff00ff';
            trpmData.forEach((val, i) => {
                let x = i * (350 / 50);
                let y = 150 - ((val / maxVal) * 150);
                i === 0 ? rpmCtx.moveTo(x, y) : rpmCtx.lineTo(x, y);
            });
            rpmCtx.stroke();

            // Draw Actual RPM (Green)
            rpmCtx.beginPath();
            rpmCtx.strokeStyle = '#00FF00';
            rpmData.forEach((val, i) => {
                let x = i * (350 / 50);
                let y = 150 - ((val / maxVal) * 150);
                i === 0 ? rpmCtx.moveTo(x, y) : rpmCtx.lineTo(x, y);
            });
            rpmCtx.stroke();
        }

        function drawSteering(angleStr) {
            let angle = parseFloat(angleStr) || 0;
            steerCtx.clearRect(0, 0, 120, 120);
            steerCtx.save();
            steerCtx.translate(60, 60);
            steerCtx.rotate(angle * Math.PI / 180);
            // Draw Tires
            steerCtx.fillStyle = '#FFF';
            steerCtx.fillRect(-40, -25, 20, 50);
            steerCtx.fillRect(20, -25, 20, 50);
            steerCtx.strokeStyle = '#555';
            steerCtx.beginPath(); steerCtx.moveTo(-30, 0); steerCtx.lineTo(30, 0); steerCtx.stroke();
            steerCtx.restore();
            document.getElementById('live_steer_val').innerText = angle.toFixed(1) + "°";
        }

        // WebSocket Setup
        let ws;
        function initWebSocket() {
            ws = new WebSocket(`ws://${window.location.hostname}/ws`);
            ws.onmessage = (e) => {
                const data = JSON.parse(e.data);
                
                // Update Text Telemetry
                document.getElementById('live_state').innerText = data.state;
                document.getElementById('live_dms').innerText = data.dms;
                document.getElementById('live_speed').innerText = data.speed + " km/h";
                document.getElementById('live_dir').innerText = data.dir;

                // State Banner Logic
                const banner = document.getElementById('state_banner');
                if (!data.jetson_link) {
                    banner.innerText = "LINK LOST - SYSTEM SAFED";
                    banner.className = "state-error";
                    if(lastLinkState && isRecording) {
                        document.getElementById('auto_dl_warn').style.display = "block";
                        downloadCSV(); // Auto-save failsafe
                    }
                } else {
                    banner.innerText = `FSM MODE: ${data.state}`;
                    banner.className = (data.state === "AUTONOMOUS") ? "state-auto" : "state-idle";
                }
                lastLinkState = data.jetson_link;

                // Graph Logic
                let currentRpm = parseFloat(data.rpm) || 0;
                let targetRpm = (data.t_rpm === "--") ? 0 : parseFloat(data.t_rpm);
                rpmData.push(currentRpm); trpmData.push(targetRpm);
                if(rpmData.length > 50) { rpmData.shift(); trpmData.shift(); }
                drawRPMGraph();
                drawSteering(data.steer_angle);

                // Packet Inspector Logic
                if (data.jetson_link && data.rx_hex !== "-- NO LINK --") {
                    const bytes = data.rx_hex.trim().split(" ");
                    if(bytes.length === 14) {
                        bytes.forEach((b, i) => {
                            let el = document.getElementById(`b${i}`);
                            if (prevBytes[i] && prevBytes[i] !== b) {
                                document.getElementById(`box${i}`).style.backgroundColor = "#550000"; // Flash delta
                                setTimeout(() => document.getElementById(`box${i}`).style.backgroundColor = "#111", 100);
                            }
                            el.innerText = b;
                        });
                        prevBytes = bytes;
                        
                        // Math breakdown
                        document.getElementById('calc_rpm').innerText = `${data.t_rpm} ((0x${bytes[1]}<<8)|0x${bytes[2]})`;
                        document.getElementById('calc_steer').innerText = `${data.t_steer} ((0x${bytes[3]}<<8)|0x${bytes[4]})`;
                        document.getElementById('calc_brake').innerText = `${data.t_brake}`;
                    }
                }

                // RAM Circular Logging
                if (isRecording) {
                    if (csvRows.length > MAX_ROWS) csvRows.splice(1, 1); // Keep header
                    csvRows.push([new Date().toISOString(), data.state, data.dms, data.rpm, data.t_rpm, data.speed, data.steer_angle]);
                    document.getElementById('buf_size').innerText = csvRows.length - 1;
                }
            };
            ws.onclose = () => setTimeout(initWebSocket, 1000);
        }

        function toggleRecording() {
            isRecording = !isRecording;
            const btn = document.getElementById('recBtn');
            btn.innerText = isRecording ? "RECORDING 20Hz... (CLICK STOP)" : "START 20Hz LOGGING";
            btn.style.backgroundColor = isRecording ? "#FF0000" : "#000";
        }

        function downloadCSV() {
            if (csvRows.length < 2) return;
            let content = csvRows.map(e => e.join(",")).join("\n");
            let a = document.createElement('a');
            a.href = window.URL.createObjectURL(new Blob([content], { type: 'text/csv' }));
            a.download = `VCS_RAM_LOG_${new Date().getTime()}.csv`;
            a.click();
        }

        window.onload = initWebSocket;
    </script>
</body>
</html>
)rawliteral";

// ============================================================
//  getTelemetryJSON()
//  FIX #13b: ansHeartbeatReceived() checked before including
//  Jetson target values. When link is dead, target fields are
//  reported as "--" and jetson_link is false. The frontend
//  uses jetson_link to gate display independently.
// ============================================================
String getTelemetryJSON() {
    String fsm_state = String(getStateName(currentState));
    bool   dms_active = isDeadmanActive();
    float  rpm        = getMeasuredRPM();
    uint16_t steer_adc = getMeasuredSteering();
    bool   is_rev     = isReverseEngaged();
    float  speed_kmh  = getMeasuredSpeedKmh();
    float  steer_angle = (((float)steer_adc - 500.0f) / 500.0f) * MAX_STEERING_ANGLE_DEG;

    int16_t  rpm_int  = (int16_t)rpm;
    uint8_t  rpm_h    = (rpm_int  >> 8) & 0xFF;
    uint8_t  rpm_l    =  rpm_int        & 0xFF;
    uint8_t  steer_h  = (steer_adc >> 8) & 0xFF;
    uint8_t  steer_l  =  steer_adc       & 0xFF;
    uint8_t  u_state  = 3;

    String speed_mode = "UNKNOWN";
    if      (current_drive_mode == DRIVE_LOW)  speed_mode = "LOW (30%)";
    else if (current_drive_mode == DRIVE_MED)  speed_mode = "MID (60%)";
    else if (current_drive_mode == DRIVE_HIGH) speed_mode = "HIGH (100%)";

    // FIX #13b: Check link once and use the result consistently.
    bool linked = ansHeartbeatReceived();

    String json;
    json.reserve(600);
    json = "{";
    json += "\"state\":\""       + fsm_state + "\",";
    json += "\"dms\":\""         + String(dms_active ? "HELD (OK)" : "RELEASED") + "\",";
    json += "\"rpm\":\""         + String(rpm, 1) + "\",";
    json += "\"speed\":\""       + String(speed_kmh, 1) + "\",";
    json += "\"steer_angle\":\"" + String(steer_angle, 1) + "\",";
    json += "\"dir\":\""         + String(is_rev ? "REVERSE" : "FORWARD") + "\",";
    json += "\"rpm_h\":\"0x"     + String(rpm_h,   HEX) + "\",";
    json += "\"rpm_l\":\"0x"     + String(rpm_l,   HEX) + "\",";
    json += "\"steer_h\":\"0x"   + String(steer_h, HEX) + "\",";
    json += "\"steer_l\":\"0x"   + String(steer_l, HEX) + "\",";
    json += "\"u_state\":\""     + String(u_state) + "\",";
    json += "\"three_speed\":\""  + speed_mode + "\",";

    // FIX #13b: When no link, report "--" instead of stale numeric values.
    // The frontend also guards these via jetson_link, but the server
    // sends clean data so stale values never leak through either path.
    if (linked) {
        json += "\"t_rpm\":\""   + String(current_target_rpm)   + "\",";
        json += "\"t_steer\":\"" + String(current_target_steer) + "\",";
        json += "\"t_brake\":\"" + String(current_target_brake) + "\",";
        json += "\"t_mode\":\""  + String(current_target_mode)  + "\",";
        json += "\"rx_hex\":\""  + last_rx_hex + "\",";
    } else {
        json += "\"t_rpm\":\"--\",";
        json += "\"t_steer\":\"--\",";
        json += "\"t_brake\":\"--\",";
        json += "\"t_mode\":\"--\",";
        json += "\"rx_hex\":\"-- NO LINK --\",";
    }

    json += "\"tx_hex\":\""      + last_tx_hex + "\",";
    // FIX #13b: Link status flag — frontend uses this to gate display.
    json += "\"jetson_link\":"   + String(linked ? "true" : "false") + ",";
    json += "\"sys_logs\":\""    + systemLogBuffer + "\"";
    json += "}";

    systemLogBuffer = "";
    return json;
}

// ============================================================
//  WEB SERVER INIT & TASK (OVERHAULED FOR WEBSOCKETS & RACE MODE)
// ============================================================
void initWebServer() {
    WiFi.softAP(ssid, password);
    
    // Attach WebSocket listener
    ws.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len){
        if(type == WS_EVT_CONNECT){
            Serial.println("[VCS LOG] Dashboard Connected.");
        }
    });
    server.addHandler(&ws);

    // Serve HTML page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *req){ 
        req->send(200, "text/html", index_html); 
    });
    
    // Polling endpoint /data is removed, pushed via WebSocket instead
    server.begin();
    Serial.println("[VCS LOG] Pit Mode: Web & WiFi Active.");
}

void WebServerTask(void *pvParameters) {
    if (DISABLE_WEB_WIFI == 0) {
        initWebServer();
        for (;;) {
            ws.cleanupClients();
            
            // Push high-speed telemetry at 20Hz (every 50ms)
            String jsonPayload = getTelemetryJSON();
            ws.textAll(jsonPayload);
            
            vTaskDelay(50 / portTICK_PERIOD_MS); 
        }
    } else {
        // RACE MODE: HARDCODED OVERRIDE
        // Kill the Wi-Fi hardware to save battery and reduce CPU jitter
        Serial.println("[VCS LOG] Race Mode Engaged: Wi-Fi Radio Offline.");
        WiFi.mode(WIFI_OFF); 
        
        // Delete this FreeRTOS task so it consumes 0 clock cycles
        vTaskDelete(NULL); 
    }
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
#include "vcs_calibration.h"
#include "vcs_constants.h"

extern esp_adc_cal_characteristics_t adc_chars;

float smoothedSteering = 0.0f;


static int s_last_freq_hz = -1;

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
    if (!is_automatic || currentState == FAULT_STATE) {
        digitalWrite(PIN_STEER_ENA, HIGH); 
        ledcWrite(0, 0);
        s_last_freq_hz = -1; 

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
// ============================================================
//  vcs_state_machine.cpp — Rev fixes applied
//
//  CHANGES FROM ORIGINAL (fixes only — no new features):
//
//  FIX #2  — updateStateMachine() no longer takes a faults parameter.
//             It reads g_systemFaults directly. Eliminates risk of
//             callers passing a stale or wrong fault word.
//             clearFault() and clearAllFaults() added — g_systemFaults
//             was previously write-only (no way to clear a resolved fault).
//
//  FIX #3  — FAULT_STATE recovery now requires BOTH heartbeat restored
//             AND g_systemFaults == VCS_FAULT_NONE. Previously recovered
//             on heartbeat alone, so a hardware fault would clear just
//             because the Jetson reconnected.
//
//  FIX #4  — s_signalLostTime and s_lastLogTime promoted from static
//             locals inside the MANUAL_STATE case to file-scope statics.
//             Static locals inside a switch case cannot be explicitly
//             reset on state re-entry (e.g. AUTONOMOUS → MANUAL).
//             File-scope statics are resettable in initState_Machine().
//
//  FIX #9  — Removed isJetsonStopLineActive() call. That function is
//             defined in vcs_uart.cpp and encodes stop-line semantics
//             inside the UART module. The check (getTargetBrake() > 0)
//             is now inlined here where the semantic decision belongs.
//             Remove isJetsonStopLineActive() declaration from vcs_uart.h.
//
//  FIX #10 — requestSoftwareEstop() now sets a latched flag. Once set,
//             FAULT_STATE will not self-recover. Prevents the original
//             FAULT → IDLE cycling after a commanded E-stop.
//             Only a power-cycle clears the latch.
//
//  NOTE: vcs_state_machine.h must be updated to match:
//    - updateStateMachine() signature: remove uint32_t faults parameter
//    - Add declarations: clearFault(uint16_t), clearAllFaults()
// ============================================================

#include "vcs_state_machine.h"
#include "vcs_uart.h"
#include "vcs_pins.h"
#include "vcs_deadman.h"
#include "vcs_lowbrake.h"
#include "vcs_throttle.h"
#include "vcs_reverse.h"
#include "vcs_web.h"
#include "vcs_hallsensor.h"
#include "vcs_steering.h"

// Hall ISR attach/detach lives in vcs_hallsensor.cpp
extern void hall_interrupts_attach();
extern void hall_interrupts_detach();

// Brake helper lives in vcs_lowbrake.cpp / vcs_relays.cpp
extern void forceBrakeEngagement(bool engage);

static uint32_t g_systemFaults = 0;

// =========================================================
// FIX #4: Promoted static locals from MANUAL_STATE switch
// case to file scope. Static locals inside a switch case
// persist correctly in C++ but cannot be explicitly reset
// on state re-entry (e.g. AUTONOMOUS → MANUAL transitions).
// File-scope statics with clear names are resettable in
// initState_Machine().
// =========================================================
static uint32_t s_signalLostTime = 0;
static uint32_t s_lastLogTime    = 0;

// FIX #10: Latched E-stop flag. Once set by requestSoftwareEstop(),
// FAULT_STATE will not recover until power-cycle.
// Shell Eco-marathon rules require a commanded E-stop to be
// non-auto-recoverable.
static bool s_estopLatched = false;

// =========================================================
// Global state
// =========================================================
VcsState currentState      = INIT_STATE;
uint32_t dmsStartTime      = 0;
uint32_t stoppingStartTime = 0;

// =========================================================
// INIT
// =========================================================
void initState_Machine() {
    currentState      = INIT_STATE;
    dmsStartTime      = 0;
    stoppingStartTime = 0;
    // FIX #4: Explicitly reset promoted statics on re-init.
    s_signalLostTime  = 0;
    s_lastLogTime     = 0;
    // FIX #10: E-stop latch intentionally survives re-init.
    // Only a power-cycle clears it.
}

// =========================================================
// FSM TICK — call from EXACTLY ONE task (task_control).
//
// FIX #2: Signature changed — faults parameter removed.
// g_systemFaults is read directly. Update vcs_state_machine.h
// to match: void updateStateMachine();
// =========================================================
void updateStateMachine() {
    static VcsState lastState = INIT_STATE;

    // ---------------------------------------------------------
    // 1. PRIORITY SAFETY OVERRIDES
    //    FIX #2: Read g_systemFaults directly instead of the
    //    caller-supplied faults parameter that was previously
    //    passed in (and could be stale).
    //    Heartbeat loss only forces FAULT while in AUTONOMOUS;
    //    losing the link during MANUAL is non-fatal by design.
    // ---------------------------------------------------------
    if (g_systemFaults != VCS_FAULT_NONE ||
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

        case MANUAL_STATE: {
            // FIX #4: s_signalLostTime and s_lastLogTime are now
            // file-scope statics (see top of file), resettable on
            // state re-entry. Behaviour is otherwise unchanged.
            if (isDeadmanActive() && !isReverseEngaged()) {

                s_signalLostTime = 0;

                if (getANSCommandMode() == 1) {
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
                    if (millis() - s_lastLogTime >= 2000) {
                        vcs_log("WAITING: Jetson must send AUTO (mode=1)");
                        s_lastLogTime = millis();
                    }
                }
            } else {
                // 50ms grace period to prevent switch bounce resetting the timer
                if (dmsStartTime != 0) {
                    if (s_signalLostTime == 0) s_signalLostTime = millis();
                    if (millis() - s_signalLostTime > 50) {
                        dmsStartTime     = 0;
                        s_signalLostTime = 0;
                    }
                } else {
                    s_signalLostTime = 0;
                }
            }
            break;
        }

        case AUTONOMOUS_STATE:
            // Hard physical overrides → MANUAL
            if (isThrottlePedalPressed() || !isDeadmanActive()) {
                currentState = MANUAL_STATE;
                vcs_log("SAFETY: physical override -> MANUAL");
            }
            // FIX #9: Replaced isJetsonStopLineActive() with the inline
            // check it contained. Stop-line semantics belong in the state
            // machine, not in vcs_uart.cpp. Remove isJetsonStopLineActive()
            // from vcs_uart.h and vcs_uart.cpp.
            else if (isPhysicalBrakePressed() || (getTargetBrake() > 0)) {
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
            // FIX #9: Same inline check replacing isJetsonStopLineActive().
            // 3 s elapsed and stop-line cleared → resume AUTONOMOUS
            if ((millis() - stoppingStartTime >= 3000) && !(getTargetBrake() > 0)) {
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
            // FIX #3: Recovery now requires ALL three conditions:
            //   1. E-stop not latched (FIX #10)
            //   2. Heartbeat restored
            //   3. All fault bits cleared (was missing — previously
            //      recovered on heartbeat alone, so a hardware fault
            //      would clear just because the Jetson reconnected)
            if (!s_estopLatched &&
                ansHeartbeatReceived() &&
                g_systemFaults == VCS_FAULT_NONE) {
                currentState = IDLE_STATE;
            }
            break;
    }

    // ---------------------------------------------------------
    // 3. ENTRY ACTIONS (run once per state change)
    // ---------------------------------------------------------
    if (currentState != lastState) {

        // Engage brake whenever we leave a driving state
        if (!isDrivingState()) {
            forceBrakeEngagement(true);
        }

        // Hall ISR management:
        //   - Attach the first time we leave INIT (MC is up by now).
        //   - Detach on FAULT (we don't trust the hardware).
        //   - DO NOT detach on STOPPING — we still need RPM.
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
// FAULT MANAGEMENT
// FIX #2: clearFault() and clearAllFaults() added.
// g_systemFaults was previously write-only — once a fault
// bit was set via triggerFault() there was no way to clear
// it even after the condition resolved.
// =========================================================
uint32_t getSystemFaults() {
    return g_systemFaults;
}

void triggerFault(uint16_t fault_code) {
    g_systemFaults |= fault_code;
}

// FIX #2: Clear a specific fault bit. Call after verifying
// the fault condition is resolved.
void clearFault(uint16_t fault_code) {
    g_systemFaults &= ~(uint32_t)fault_code;
}

// FIX #2: Clear all fault bits. Call only after verifying
// all fault conditions are fully resolved.
void clearAllFaults() {
    g_systemFaults = VCS_FAULT_NONE;
}

// =========================================================
// ESTOP
// FIX #10: requestSoftwareEstop() now sets a latched flag.
// Once latched, FAULT_STATE will not recover automatically.
// The original code routed through FAULT which is recoverable
// — that is not a true E-stop. Power-cycle required to clear.
// =========================================================
void requestSoftwareEstop() {
    s_estopLatched = true;
    currentState   = FAULT_STATE;
    vcs_log("ESTOP latched -> FAULT_STATE (power-cycle to clear)");
}

// =========================================================
// HELPERS (unchanged)
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
void updateStateMachine();
void clearFault(uint16_t fault_code);
void clearAllFaults();
void triggerFault(uint16_t fault_code);

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

### src\main.cpp

// main.cpp
// ============================================================
//  CHANGES FROM ORIGINAL (call-site fixes only):
//
//  FIX #2 call site — updateStateMachine(getSystemFaults())
//    changed to updateStateMachine().
//    The faults parameter was removed in vcs_state_machine.cpp
//    (FIX #2) — the function now reads g_systemFaults directly.
//    Update vcs_state_machine.h: void updateStateMachine();
//
//  FIX #12 call site — broadcastVehicleTelemetry()
//    changed to broadcastVehicleTelemetry(gear).
//    FIX #12 in vcs_uart.cpp moved gear computation to the caller
//    so vcs_uart does not depend on vcs_threespeed internals.
//    Gear is computed in UITask() before the call.
//    Update vcs_uart.h: void broadcastVehicleTelemetry(uint8_t gear);
// ============================================================

#include <Arduino.h>
#include "vcs_pins.h"
#include "vcs_constants.h"
#include "vcs_state_machine.h"
#include "vcs_uart.h"
#include "vcs_throttle.h"
#include "vcs_lowbrake.h"
#include "vcs_deadman.h"
#include "vcs_relays.h"
#include "vcs_steering.h"
#include "vcs_hallsensor.h"
#include "vcs_threespeed.h"
#include "vcs_reverse.h"
#include "vcs_display.h"
#include "vcs_simulation.h"
#include "vcs_web.h"

// ============================================================
//  TASK HANDLES & FORWARD DECLARATIONS
// ============================================================
TaskHandle_t ControlTaskHandle = NULL;
static constexpr bool VCS_VERBOSE_TASK_LOGS = false;

extern void WebServerTask(void *pvParameters);

// ============================================================
//  FIX 1 — SIGNAL STALENESS GUARD
// ============================================================
constexpr uint32_t SIGNAL_TIMEOUT_MS = 100;

struct SignalFrame {
    float    value        = 0.0f;
    uint32_t timestamp_ms = 0;
    bool     valid() const {
        return (millis() - timestamp_ms) < SIGNAL_TIMEOUT_MS;
    }
};

static SemaphoreHandle_t g_signalMutex = NULL;
SignalFrame g_steerCmd;
SignalFrame g_throttleCmd;
SignalFrame g_brakeCmd;

// ============================================================
//  FIX 3 — HALL SENSOR ISR DEBOUNCE (EMI mitigation)
// ============================================================
constexpr uint32_t MIN_PULSE_WIDTH_US = 800;

static volatile uint32_t g_lastHallEdge_us = 0;
static volatile uint32_t g_hallFalseEdges  = 0;

extern void hallSensorISR_impl();

void IRAM_ATTR hallSensorISR() {
    uint32_t now = micros();
    uint32_t dt  = now - g_lastHallEdge_us;

    if (dt < MIN_PULSE_WIDTH_US) {
        g_hallFalseEdges++;
        return;
    }

    g_lastHallEdge_us = now;
    hallSensorISR_impl();
}

uint32_t getHallFalseEdgeCount() { return g_hallFalseEdges; }

// ============================================================
//  TASK BODIES
// ============================================================

void ControlTask(void *pvParameters) {
    for (;;) {
        if (isAutonomousMode()) {
            bool stale = false;
            if (xSemaphoreTake(g_signalMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                stale = !g_steerCmd.valid()    ||
                        !g_throttleCmd.valid() ||
                        !g_brakeCmd.valid();
                xSemaphoreGive(g_signalMutex);
            }
            if (stale) triggerFault(FAULT_SIGNAL_TIMEOUT);
        }

        // FIX #2 call site: removed getSystemFaults() argument.
        // updateStateMachine() now reads g_systemFaults directly.
        // Old: updateStateMachine(getSystemFaults());
        updateStateMachine();

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void CommTask() {
    handleIncomingUART();

    updateDeadman();
    updateLowBrake();
    updateThreeSpeed();
    updateReverse();
    updateUART();

    updateRelays(isAutonomousMode());
    if (VCS_VERBOSE_TASK_LOGS) {
        Serial.println(F(" -> CommTask Finished!"));
    }
}

void UITask() {
    // FIX #12 call site: gear is computed here (caller responsibility)
    // so broadcastVehicleTelemetry() does not need to depend on
    // vcs_threespeed internals. Matches the updated function signature.
    uint8_t gear = 1;   // default: normal
    if      (current_drive_mode == DRIVE_LOW)  gear = 0;
    else if (current_drive_mode == DRIVE_HIGH) gear = 2;

    // Old: broadcastVehicleTelemetry();
    broadcastVehicleTelemetry(gear);

    if (VCS_VERBOSE_TASK_LOGS) {
        Serial.println(F("   -> UITask Finished!"));
    }
}

// ============================================================
//  TASK WRAPPERS (unchanged)
// ============================================================
void ESP32_CommLoop(void *pvParameters) {
    for (;;) {
        CommTask();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void ESP32_UILoop(void *pvParameters) {
    for (;;) {
        UITask();
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void DisplayLoop(void *pvParameters) {
    for (;;) {
        updateDisplay(getMeasuredRPM(), getMeasuredSteering(), current_drive_mode);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

// ============================================================
//  SETUP (unchanged)
// ============================================================
void setup() {
    dacWrite(PIN_THROTTLE_OUT, 0);

    Serial.begin(115200);

    Serial.println(F("\n--- VCS v1.5: ESP32 38-PIN ---"));
    Serial.println(F("--- VCS v1.5 DIAGNOSTIC BOOT ---"));
    Serial.println(F("Testing modules one by one..."));

    g_signalMutex = xSemaphoreCreateMutex();
    if (!g_signalMutex) {
        Serial.println(F("[FATAL] Mutex alloc failed"));
        while (1) delay(1000);
    }

    Serial.print(F("1. State... "));
    initState_Machine();
    Serial.println(F("OK"));
    delay(500);

    Serial.print(F("2. UART... "));
    initUART();
    Serial.println(F("OK"));
    delay(500);

    Serial.print(F("3. Throttle/Brake... "));
    initThrottle();
    initLowBrake();
    Serial.println(F("OK"));
    delay(500);

    Serial.print(F("4. Safety... "));
    initDeadman();
    initRelays();
    Serial.println(F("OK"));
    delay(500);

    Serial.print(F("5. Steering... "));
    initSteering();
    Serial.println(F("OK"));
    delay(500);

    Serial.print(F("6. Sensors... "));
    initHallSensors();
    initThreeSpeed();
    initReverse();
    Serial.println(F("OK"));
    delay(500);

    Serial.print(F("7. Display... "));
    initDisplay();
    Serial.println(F("OK"));

    Serial.println(F("--- SURVIVED BOOT SEQUENCE ---"));

    xTaskCreatePinnedToCore(ControlTask,    "Control",   4096, NULL, 5, &ControlTaskHandle, 1);
    xTaskCreatePinnedToCore(ESP32_CommLoop, "Comms",     4096, NULL, 4, NULL,               0);
    xTaskCreatePinnedToCore(ESP32_UILoop,   "UI",        4096, NULL, 2, NULL,               0);
    xTaskCreatePinnedToCore(DisplayLoop,    "Display",   4096, NULL, 1, NULL,               1);
    xTaskCreatePinnedToCore(WebServerTask,  "WebServer", 8192, NULL, 1, NULL,               0);
}

// ============================================================
//  LOOP — unused; all work happens in FreeRTOS tasks
// ============================================================
void loop() {
}