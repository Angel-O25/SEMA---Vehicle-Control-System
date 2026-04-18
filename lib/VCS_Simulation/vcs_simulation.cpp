/* ==============================================================================
 * MODULE:        VCS_Simulation (v2.0) - Dual-Target Bench Tooling
 * ------------------------------------------------------------------------------
 * This file compiles to ONE of two implementations, selected automatically
 * by the board macro that the Arduino IDE sets for you:
 *
 *   TARGET A: Arduino Nano 33 BLE (nRF52840, 3.3 V)
 *             -> "Digital Twin" physics simulator (existing v1.4 behavior)
 *             -> Set SIMULATION_MODE=1 in vcs_constants.h
 *
 *   TARGET B: Arduino Nano (ATmega328P, 5 V) -- breadboard data-logging rig
 *             -> Real-hardware measurement + calibration tool for:
 *                  1. Steering potentiometer readout
 *                  2. Stepper motor calibration (pulses-per-pot-unit)
 *                  3. Odometer (Hall-sensor speed + distance)
 *                  4. Closed-loop steering output verification
 *                  5. Brake actuation calibration
 *                  6. Manual <-> Auto mode-switch relay test
 *             -> Driven from Serial: press '1'..'6' (or 'h') at 115200 baud.
 *
 * ------------------------------------------------------------------------------
 * TARGET A -- NANO 33 BLE SIMULATION SETUP
 *
 *   1. Physical connections (3.3 V logic only):
 *        Nano 33 BLE TX / D1  --->  RPi RXD / Pin 10
 *        Nano 33 BLE RX / D0  --->  RPi TXD / Pin 8
 *        Common GND between Nano and RPi.
 *   2. Power safety: disconnect the 60 V main battery and the 1500 W motor
 *      phase wires. Simulation runs on USB power.
 *   3. Peripherals: wire PIN_DMS_LEFT (D2), PIN_DMS_RIGHT (D4), and
 *      PIN_LOWBRAKE_IN (D8) each to GND through a switch. Both DMS grips
 *      must be held for motion to be authorized. (There is no single
 *      PIN_DMS_BUTTON -- old docs referring to it are stale.)
 *   4. Set '#define SIMULATION_MODE 1' in vcs_constants.h.
 *
 * ------------------------------------------------------------------------------
 * TARGET B -- ATMEGA328 BREADBOARD MEASUREMENT SETUP
 *
 *   !!! 5 V LOGIC WARNING !!!
 *   The ATmega328 Nano runs at 5 V. Do NOT wire its outputs directly to the
 *   1500 W controller, stepper driver, or optocoupler inputs expecting 3.3 V
 *   without level-shifting. For bench tests you can drive:
 *     - A standalone stepper driver that accepts 5 V logic (most common).
 *     - A 5 V relay module for the mode-switch test.
 *     - The steering potentiometer powered from 5 V (reads full 0..1023 scale).
 *   If you intend to test against the real vehicle's PCB, level-shift first.
 *
 *   Minimum wiring on the breadboard:
 *     PIN_STEER_POT  (A0)   <--  pot wiper (pot ends to 5V and GND)
 *     PIN_STEER_PUL  (D5)   -->  stepper driver PUL+
 *     PIN_STEER_DIR  (D6)   -->  stepper driver DIR+
 *     PIN_STEER_ENA  (D7)   -->  stepper driver ENA+ (active LOW)
 *     PIN_HALL_SPEED (D10)  <--  Hall speed pulse (to GND when low; pulled up)
 *     PIN_LOWBRAKE_IN (D8)  <--  brake pedal switch to GND
 *     PIN_LOWBRAKE_OUT (D3) -->  brake actuator/optocoupler/LED
 *     PIN_DMS_LEFT   (D2)   <--  dead-man switch to GND
 *     PIN_DMS_RIGHT  (D4)   <--  dead-man switch to GND
 *     PIN_RELAY_STATE (D12) -->  auto/manual relay module
 *     USB cable -- Serial is used for commands + CSV data logging.
 *
 *   Usage: open Serial Monitor (or any logger) at 115200 baud, line-ending
 *   set to Newline. Send '1'..'6' to run a test, 'h' for help.
 * ============================================================================== */

#include "vcs_simulation.h"
#include "vcs_pins.h"


// ============================================================================ 
// TARGET B: ATmega328P BREADBOARD MEASUREMENT MODE
// ============================================================================ 
#if defined(__AVR_ATmega328P__)

// --- Test-tuning constants (edit to match your test rig) ---------------------
static constexpr float        WHEEL_CIRCUMFERENCE_M = 1.85f;   // 26" wheel ~= 2.07 m; 20" ~= 1.60 m
static constexpr unsigned int STEPPER_TEST_RATE_HZ  = 1000;    // pulse rate during calibration
static constexpr unsigned int STEPPER_TEST_PULSES   = 800;     // default pulse count per calibration run
static constexpr unsigned long ODOMETER_WINDOW_MS   = 100;     // RPM averaging window
static constexpr unsigned long ODOMETER_TOTAL_MS    = 10000;   // total odometer test duration

// --- ISR-shared state (volatile, always read with interrupts masked) --------
static volatile uint32_t hall_pulse_count = 0;

// --- Module state -----------------------------------------------------------
static uint16_t last_pot_reading     = 0;
static float    last_computed_rpm    = 0.0f;
static bool     measurement_ready    = false;

// --- PCINT ISR on D10 (PB2, PCINT2) -----------------------------------------
// We enable only PCINT2 in PCMSK0, so this ISR fires only for D10 edges.
// Every edge (both rising and falling) increments the counter.
ISR(PCINT0_vect) {
    hall_pulse_count++;
}

// --- Atomic helpers for the 32-bit ISR counter ------------------------------
// AVR is 8-bit; a plain read of uint32_t is 4 non-atomic instructions.
static uint32_t readHallPulses() {
    uint32_t c;
    noInterrupts();
    c = hall_pulse_count;
    interrupts();
    return c;
}

static void resetHallPulses() {
    noInterrupts();
    hall_pulse_count = 0;
    interrupts();
}

// --- Forward decls ----------------------------------------------------------
static void printHelp();
static void testPotReadout();
static void testStepperCalibration();
static void testOdometer();
static void testSteeringClosedLoop();
static void testBrakeActuation();
static void testModeSwitch();
static void handleSerialCommand();

// --- One-time init (called lazily on first updateSimulatedPhysics) ----------
static void initMeasurementMode() {
    // Pin directions
    pinMode(PIN_STEER_POT,    INPUT);
    pinMode(PIN_STEER_PUL,    OUTPUT);
    pinMode(PIN_STEER_DIR,    OUTPUT);
    pinMode(PIN_STEER_ENA,    OUTPUT);
    pinMode(PIN_HALL_SPEED,   INPUT_PULLUP);
    pinMode(PIN_LOWBRAKE_IN,  INPUT_PULLUP);
    pinMode(PIN_LOWBRAKE_OUT, OUTPUT);
    pinMode(PIN_DMS_LEFT,     INPUT_PULLUP);
    pinMode(PIN_DMS_RIGHT,    INPUT_PULLUP);
    pinMode(PIN_RELAY_STATE,  OUTPUT);

    // Safe initial states
    digitalWrite(PIN_STEER_ENA,    HIGH);  // stepper driver disabled (ENA is active-low)
    digitalWrite(PIN_STEER_PUL,    LOW);
    digitalWrite(PIN_STEER_DIR,    LOW);
    digitalWrite(PIN_LOWBRAKE_OUT, LOW);
    digitalWrite(PIN_RELAY_STATE,  LOW);

    // Enable PCINT on D10 for hall pulse counting
    noInterrupts();
    PCICR  |= (1 << PCIE0);     // enable PCINT0..7 (PORTB) group
    PCMSK0 |= (1 << PCINT2);    // mask in only PCINT2 (D10 / PB2)
    hall_pulse_count = 0;
    interrupts();

    // Serial is the data logger and command surface
    if (!Serial) Serial.begin(115200);
    delay(50);
    Serial.println();
    Serial.println(F("# ================================================"));
    Serial.println(F("# VCS Measurement Mode -- ATmega328 breadboard rig"));
    Serial.println(F("# ================================================"));
    printHelp();

    measurement_ready = true;
}

static void printHelp() {
    Serial.println(F("# Commands:"));
    Serial.println(F("#   1 -- Steering pot readout (continuous, 5 s)"));
    Serial.println(F("#   2 -- Stepper calibration (pulses-per-pot-unit)"));
    Serial.println(F("#   3 -- Odometer / Hall-sensor speed (10 s)"));
    Serial.println(F("#   4 -- Closed-loop steering to commanded target"));
    Serial.println(F("#   5 -- Brake actuation calibration"));
    Serial.println(F("#   6 -- Manual<->Auto mode-switch test"));
    Serial.println(F("#   h -- this help"));
    Serial.println(F("# Output is CSV; lines beginning with '#' are comments."));
}

// --- Test 1: Potentiometer readout ------------------------------------------
static void testPotReadout() {
    Serial.println(F("# Test 1 -- Steering pot readout"));
    Serial.println(F("# t_ms,raw_adc,pct_of_range,scaled_comm_units"));

    const unsigned long t_start = millis();
    while (millis() - t_start < 5000UL) {
        uint16_t raw = analogRead(PIN_STEER_POT);
        last_pot_reading = raw;

        // Map raw ADC onto the communication-protocol steering space
        long scaled = map((long)raw,
                          STEER_POT_MIN, STEER_POT_MAX,
                          COMM_STEER_LEFT, COMM_STEER_RIGHT);
        scaled = constrain(scaled, (long)COMM_STEER_LEFT, (long)COMM_STEER_RIGHT);

        float pct = 100.0f * (float)(raw - STEER_POT_MIN) /
                             (float)(STEER_POT_MAX - STEER_POT_MIN);

        Serial.print(millis() - t_start);
        Serial.print(',');
        Serial.print(raw);
        Serial.print(',');
        Serial.print(pct, 1);
        Serial.print(',');
        Serial.println(scaled);
        delay(50);
    }
    Serial.println(F("# Test 1 complete."));
}

// --- Test 2: Stepper calibration --------------------------------------------
// Drives a fixed number of pulses in one direction, then the reverse, and
// reports the resulting pot delta. Use the ratio to set your steering gain.
static void testStepperCalibration() {
    Serial.println(F("# Test 2 -- Stepper calibration"));
    Serial.print(F("# pulses="));        Serial.print(STEPPER_TEST_PULSES);
    Serial.print(F(", rate_hz="));       Serial.println(STEPPER_TEST_RATE_HZ);
    Serial.println(F("# direction,pulse_idx,pot_raw"));

    const unsigned int half_period_us = 1000000UL / (STEPPER_TEST_RATE_HZ * 2UL);

    digitalWrite(PIN_STEER_ENA, LOW);  // enable driver (active-low)
    delay(10);

    for (int dir = 0; dir < 2; ++dir) {
        digitalWrite(PIN_STEER_DIR, dir ? HIGH : LOW);
        delay(5);

        uint16_t pot_before = analogRead(PIN_STEER_POT);
        Serial.print(F("# dir=")); Serial.print(dir);
        Serial.print(F(" pot_before=")); Serial.println(pot_before);

        for (unsigned int i = 0; i < STEPPER_TEST_PULSES; ++i) {
            digitalWrite(PIN_STEER_PUL, HIGH);
            delayMicroseconds(half_period_us);
            digitalWrite(PIN_STEER_PUL, LOW);
            delayMicroseconds(half_period_us);

            // Log every 50th pulse so we don't drown Serial at 1 kHz
            if ((i % 50) == 0) {
                Serial.print(dir); Serial.print(',');
                Serial.print(i);   Serial.print(',');
                Serial.println(analogRead(PIN_STEER_POT));
            }
        }

        delay(50);
        uint16_t pot_after = analogRead(PIN_STEER_POT);
        int16_t  delta     = (int16_t)pot_after - (int16_t)pot_before;

        Serial.print(F("# dir=")); Serial.print(dir);
        Serial.print(F(" pot_after="));  Serial.print(pot_after);
        Serial.print(F(" delta="));      Serial.println(delta);

        if (delta != 0) {
            Serial.print(F("# pulses_per_pot_unit="));
            Serial.println((float)STEPPER_TEST_PULSES / (float)abs(delta), 3);
        } else {
            Serial.println(F("# pulses_per_pot_unit=N/A (pot did not move - check wiring/mechanical coupling)"));
        }
        delay(500);
    }

    digitalWrite(PIN_STEER_ENA, HIGH); // disable driver
    Serial.println(F("# Test 2 complete."));
}

// --- Test 3: Odometer (Hall speed + distance) -------------------------------
static void testOdometer() {
    Serial.println(F("# Test 3 -- Odometer / Hall speed"));
    Serial.print(F("# wheel_circumference_m=")); Serial.println(WHEEL_CIRCUMFERENCE_M, 3);
    Serial.print(F("# transitions_per_mech_rev="));
    Serial.println((unsigned int)HALL_TRANSITIONS_REV * (unsigned int)MOTOR_POLE_PAIRS);
    Serial.println(F("# t_ms,pulses_total,pulses_window,rpm,speed_kmh,distance_m"));

    const float transitions_per_rev =
        (float)HALL_TRANSITIONS_REV * (float)MOTOR_POLE_PAIRS;

    resetHallPulses();
    unsigned long t_start  = millis();
    unsigned long t_window = t_start;
    uint32_t      last_count = 0;
    float         total_distance_m = 0.0f;

    while (millis() - t_start < ODOMETER_TOTAL_MS) {
        if (millis() - t_window >= ODOMETER_WINDOW_MS) {
            uint32_t count       = readHallPulses();
            uint32_t delta       = count - last_count;
            last_count           = count;

            float revs_window    = (float)delta / transitions_per_rev;
            float rpm            = revs_window * (60000.0f / (float)ODOMETER_WINDOW_MS);
            float dist_window_m  = revs_window * WHEEL_CIRCUMFERENCE_M;
            total_distance_m    += dist_window_m;
            float speed_kmh      = (dist_window_m / (ODOMETER_WINDOW_MS / 1000.0f)) * 3.6f;

            last_computed_rpm = rpm;

            Serial.print(millis() - t_start);     Serial.print(',');
            Serial.print(count);                  Serial.print(',');
            Serial.print(delta);                  Serial.print(',');
            Serial.print(rpm, 1);                 Serial.print(',');
            Serial.print(speed_kmh, 2);           Serial.print(',');
            Serial.println(total_distance_m, 3);

            t_window = millis();
        }
    }
    Serial.println(F("# Test 3 complete."));
}

// --- Test 4: Closed-loop steering output test -------------------------------
// Commands a target pot value and drives the stepper with a simple P loop,
// logging pot vs target until within tolerance (or timeout).
static void testSteeringClosedLoop() {
    Serial.println(F("# Test 4 -- Closed-loop steering"));
    Serial.println(F("# Sweeping targets: LEFT -> CENTER -> RIGHT -> CENTER"));
    Serial.println(F("# t_ms,target,pot,error,dir"));

    const uint16_t targets[] = {
        (uint16_t)(STEER_POT_MIN + 100),
        (uint16_t)((STEER_POT_MIN + STEER_POT_MAX) / 2),
        (uint16_t)(STEER_POT_MAX - 100),
        (uint16_t)((STEER_POT_MIN + STEER_POT_MAX) / 2)
    };
    const int16_t tolerance     = 10;
    const unsigned long settle_timeout_ms = 4000;
    const unsigned int half_period_us = 1000000UL / (STEPPER_TEST_RATE_HZ * 2UL);

    digitalWrite(PIN_STEER_ENA, LOW);
    delay(10);

    for (uint8_t t = 0; t < sizeof(targets) / sizeof(targets[0]); ++t) {
        uint16_t target = targets[t];
        Serial.print(F("# target=")); Serial.println(target);

        unsigned long t0 = millis();
        while (millis() - t0 < settle_timeout_ms) {
            uint16_t pot = analogRead(PIN_STEER_POT);
            int16_t  err = (int16_t)target - (int16_t)pot;

            if (abs(err) <= tolerance) {
                Serial.print(F("# settled in ")); Serial.print(millis() - t0);
                Serial.println(F(" ms"));
                break;
            }

            bool dir = (err > 0);
            digitalWrite(PIN_STEER_DIR, dir ? HIGH : LOW);

            // Issue one pulse per loop iteration
            digitalWrite(PIN_STEER_PUL, HIGH);
            delayMicroseconds(half_period_us);
            digitalWrite(PIN_STEER_PUL, LOW);
            delayMicroseconds(half_period_us);

            static unsigned long last_log = 0;
            if (millis() - last_log >= 20) {
                Serial.print(millis() - t0); Serial.print(',');
                Serial.print(target);        Serial.print(',');
                Serial.print(pot);           Serial.print(',');
                Serial.print(err);           Serial.print(',');
                Serial.println(dir ? 'R' : 'L');
                last_log = millis();
            }
        }
        delay(250);
    }

    digitalWrite(PIN_STEER_ENA, HIGH);
    Serial.println(F("# Test 4 complete."));
}

// --- Test 5: Brake actuation calibration ------------------------------------
// Pulses the brake output and cross-logs the physical brake-input switch,
// so you can validate the input debounce, output timing, and signal path.
static void testBrakeActuation() {
    Serial.println(F("# Test 5 -- Brake actuation"));
    Serial.println(F("# Will assert brake output for 500 ms, idle for 500 ms, 4 cycles."));
    Serial.println(F("# Also logs physical brake switch input continuously."));
    Serial.println(F("# t_ms,cmd_out,phys_sw,latency_ms"));

    for (uint8_t cycle = 0; cycle < 4; ++cycle) {
        // Assert
        unsigned long t_assert = millis();
        digitalWrite(PIN_LOWBRAKE_OUT, HIGH);
        unsigned long first_sw_low = 0;

        while (millis() - t_assert < 500UL) {
            bool sw = (digitalRead(PIN_LOWBRAKE_IN) == LOW);
            if (sw && first_sw_low == 0) {
                first_sw_low = millis();
            }
            Serial.print(millis() - t_assert); Serial.print(',');
            Serial.print(1);                   Serial.print(',');
            Serial.print(sw ? 1 : 0);          Serial.print(',');
            Serial.println(first_sw_low ? (int)(first_sw_low - t_assert) : -1);
            delay(20);
        }

        // Release
        unsigned long t_release = millis();
        digitalWrite(PIN_LOWBRAKE_OUT, LOW);
        while (millis() - t_release < 500UL) {
            bool sw = (digitalRead(PIN_LOWBRAKE_IN) == LOW);
            Serial.print(500 + (millis() - t_release)); Serial.print(',');
            Serial.print(0);                            Serial.print(',');
            Serial.print(sw ? 1 : 0);                   Serial.print(',');
            Serial.println(-1);
            delay(20);
        }
    }

    Serial.println(F("# Test 5 complete."));
}

// --- Test 6: Manual <-> Auto mode switching ---------------------------------
// The autonomy lockout: both DMS grips must be held (LOW, since INPUT_PULLUP)
// AND the organizer relay must be in AUTO for the system to consider itself
// authorized. This test toggles the relay and reports the combined state.
static void testModeSwitch() {
    Serial.println(F("# Test 6 -- Manual<->Auto mode switch"));
    Serial.println(F("# Toggling PIN_RELAY_STATE every 1 s for 10 s."));
    Serial.println(F("# Hold both DMS grips during the AUTO half to see AUTHORIZED."));
    Serial.println(F("# t_ms,relay,dms_L,dms_R,authorized"));

    unsigned long t_start = millis();
    bool relay_state = false;
    unsigned long t_last_toggle = t_start;

    while (millis() - t_start < 10000UL) {
        if (millis() - t_last_toggle >= 1000UL) {
            relay_state = !relay_state;
            digitalWrite(PIN_RELAY_STATE, relay_state ? HIGH : LOW);
            t_last_toggle = millis();
        }

        bool dms_l = (digitalRead(PIN_DMS_LEFT)  == LOW);   // pressed = LOW
        bool dms_r = (digitalRead(PIN_DMS_RIGHT) == LOW);
        bool authorized = relay_state && dms_l && dms_r;

        Serial.print(millis() - t_start); Serial.print(',');
        Serial.print(relay_state ? 1 : 0); Serial.print(',');
        Serial.print(dms_l ? 1 : 0);       Serial.print(',');
        Serial.print(dms_r ? 1 : 0);       Serial.print(',');
        Serial.println(authorized ? 1 : 0);
        delay(100);
    }

    digitalWrite(PIN_RELAY_STATE, LOW);
    Serial.println(F("# Test 6 complete."));
}

// --- Serial command dispatcher ---------------------------------------------
static void handleSerialCommand() {
    if (!Serial.available()) return;
    int c = Serial.read();
    switch (c) {
        case '1': testPotReadout();          break;
        case '2': testStepperCalibration();  break;
        case '3': testOdometer();            break;
        case '4': testSteeringClosedLoop();  break;
        case '5': testBrakeActuation();      break;
        case '6': testModeSwitch();          break;
        case 'h': case 'H': case '?':
                  printHelp();               break;
        case '\r': case '\n': case ' ':      break;  // ignore whitespace
        default:
            Serial.print(F("# Unknown command: "));
            Serial.println((char)c);
            printHelp();
    }
}

// --- Public interface (matches vcs_simulation.h) ----------------------------
// In measurement mode the "physics update" tick becomes:
//   (a) lazy init on first call, and
//   (b) poll Serial for a test command, and
//   (c) keep a fresh pot reading so the getters return real data.
void updateSimulatedPhysics(int pulse_freq, bool direction) {
    (void)pulse_freq;
    (void)direction;

    if (!measurement_ready) {
        initMeasurementMode();
    }
    handleSerialCommand();
    last_pot_reading = analogRead(PIN_STEER_POT);
}

float getSimulatedSteering() {
    // Return the most recent real pot reading, scaled into COMM_STEER_* units
    // so callers see the same range as the simulator version.
    long scaled = map((long)last_pot_reading,
                      STEER_POT_MIN, STEER_POT_MAX,
                      COMM_STEER_LEFT, COMM_STEER_RIGHT);
    scaled = constrain(scaled, (long)COMM_STEER_LEFT, (long)COMM_STEER_RIGHT);
    return (float)scaled;
}

float getSimulatedRPM() {
    // Returns the RPM last computed by testOdometer(). Between odometer runs
    // this value is stale - for continuous RPM, call testOdometer periodically
    // or refactor the window logic into updateSimulatedPhysics.
    return last_computed_rpm;
}


// ============================================================================ 
// TARGET A: Nano 33 BLE -- DIGITAL TWIN PHYSICS SIMULATOR (unchanged v1.4)
// ============================================================================ 
#else

#include "vcs_throttle.h"   // current_pwm_duty (what vcs_throttle last wrote)

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

#endif  // __AVR_ATmega328P__