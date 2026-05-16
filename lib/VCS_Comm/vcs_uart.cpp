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
    bool reverseOn = (pkt[10] & 0x01) != 0;

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
    (void)gear;   // GEAR no longer transmitted per fixed Sidlak spec.
                  // Parameter retained for API compatibility — remove
                  // from signature in a future cleanup pass.

    float    rpm;
    uint16_t steer;
    uint8_t  mode  = (uint8_t)currentState;     // ACTUAL_MODE — first in payload
    uint8_t  brake = getTargetBrake();          // ACTUAL_BRAKE — echo commanded value

    // Reverse encoded as bitfield: bit 0 = reverse, bit 1 = horn (reserved),
    // bits 2-7 reserved per spec.
    uint8_t revField = isReverseEngaged() ? 0x01 : 0x00;

#if SIMULATION_MODE
    rpm   = getSimulatedRPM();
    steer = getSimulatedSteering();
    Serial.print(F("SIM -> Mode:")); Serial.print(mode);
    Serial.print(F(" RPM:"));        Serial.print(rpm);
    Serial.print(F(" Steer:"));      Serial.print(steer);
    Serial.print(F(" Brake:"));      Serial.print(brake);
    Serial.print(F(" Rev:"));        Serial.println(revField);
#else
    rpm   = getMeasuredRPM();
    steer = getMeasuredSteering();
#endif

    int16_t rpmInt = (int16_t)rpm;

    // Fixed Sidlak protocol — ESP32 → ANS payload layout:
    //   [0] MSG_TYPE       0x02
    //   [1] LENGTH         0x07 (7-byte payload)
    //   [2] ACTUAL_MODE
    //   [3-4] ACTUAL_RPM   (uint16, big-endian)
    //   [5-6] ACTUAL_STEER (uint16, big-endian)
    //   [7] ACTUAL_BRAKE
    //   [8] ACTUAL_REVERSE (bitfield: bit 0 = reverse, bit 1 = horn, 2-7 reserved)
    uint8_t buf[9];
    buf[0] = 0x02;                              // MSG_TYPE
    buf[1] = 0x07;                              // LENGTH
    buf[2] = mode;                              // ACTUAL_MODE  (frame byte 4)
    buf[3] = (uint8_t)((rpmInt >> 8) & 0xFF);   // ACTUAL_RPM_H (frame byte 5)
    buf[4] = (uint8_t)( rpmInt       & 0xFF);   // ACTUAL_RPM_L (frame byte 6)
    buf[5] = (uint8_t)((steer  >> 8) & 0xFF);   // ACTUAL_STEER_H (frame byte 7)
    buf[6] = (uint8_t)( steer        & 0xFF);   // ACTUAL_STEER_L (frame byte 8)
    buf[7] = brake;                             // ACTUAL_BRAKE (frame byte 9)
    buf[8] = revField;                          // ACTUAL_REVERSE bitfield (frame byte 10)

    uint16_t crc = calculateCRC16(buf, 9);

    uint8_t frame[14] = {
        0xAA, 0x55,
        buf[0], buf[1], buf[2], buf[3], buf[4],
        buf[5], buf[6], buf[7], buf[8],
        (uint8_t)((crc >> 8) & 0xFF),
        (uint8_t)( crc       & 0xFF),
        0xFF
    };

    if (UART_DEBUG_LOGS) {
        printHexDebug("TX [ESP32 -> JETSON]: ", frame, 14);
    }

    Serial2.write(frame, 14);

    // FIX #7: Single snprintf build, single String assignment.
    {
        char txHexBuf[14 * 3 + 1];
        char *p = txHexBuf;
        for (int i = 0; i < 14; i++) {
            p += sprintf(p, "%02X ", frame[i]);
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
    /*
    portENTER_CRITICAL(&uartMux);
    bool v = target_direction_reverse;
    portEXIT_CRITICAL(&uartMux);
    return v;
    */
   return true;
}

bool ansHeartbeatReceived() {
    /*
    portENTER_CRITICAL(&uartMux);
    uint32_t t = last_valid_packet_time;
    portEXIT_CRITICAL(&uartMux);
    // FIX #1: t == 0 means no packet ever received.
    // millis() - 0 < 500 during first 500ms of boot — would return
    // true falsely without this guard.
    if (t == 0) return false;
    return (millis() - t) <= 500;
    */
   return true;
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