#include "vcs_uart.h"
#include "vcs_hallsensor.h"
#include "vcs_steering.h"
#include "vcs_simulation.h"
#include "vcs_state_machine.h"
#include "vcs_pins.h"
#include "vcs_reverse.h"
#include "vcs_web.h"

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
            Serial.print(F("UART CRC mismatch: calc="));
            Serial.print(calcCRC, HEX);
            Serial.print(F(" recv="));
            Serial.println(recvCRC, HEX);
            continue;
        }

        // Print the valid 14-byte packet to your PC Serial Monitor
        printHexDebug("RX [JETSON -> ESP32]: ", pkt, 14);
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

#if SIMULATION_MODE == 0

    // Debug Telemetry
uint8_t txDebug[14] = {
        0xAA, 0x55, buf[0], buf[1], buf[2], buf[3], buf[4], 
        buf[5], buf[6], buf[7], buf[8], 
        (uint8_t)((crc >> 8) & 0xFF), (uint8_t)(crc & 0xFF), 0xFF
    };
    printHexDebug("TX [ESP32 -> JETSON]: ", txDebug, 14);
    // ----------------------
    
    Serial2.write((uint8_t)0xAA);
    Serial2.write((uint8_t)0x55);
    Serial2.write(buf, 9);
    Serial2.write((uint8_t)((crc >> 8) & 0xFF));
    Serial2.write((uint8_t)( crc       & 0xFF));
    Serial2.write((uint8_t)0xFF);
#endif
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