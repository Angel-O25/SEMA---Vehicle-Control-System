#include "vcs_uart.h"
#include "vcs_hallsensor.h"
#include "vcs_steering.h"
#include "vcs_simulation.h"
#include "vcs_state_machine.h"
#include "vcs_pins.h"
#include "vcs_reverse.h"
#include "vcs_web.h"

// =========================================================
// SERIAL PORT SELECTION
// =========================================================
#if defined(NANO_33_BLE)
    #define ANS_SERIAL Serial1
#elif defined(ESP32_VCS)
    #define ANS_SERIAL Serial2   // UART2: RX=16, TX=17
#else
    #define ANS_SERIAL Serial
#endif

// =========================================================
// RX FSM STATES
// NOTE: WAIT_SEQ removed — Pi packet has no sequence byte.
//       Format: [AA][55][Type][Len][Payload...][CRC_H][CRC_L][FF]
// =========================================================
enum UartState {
    WAIT_START1,
    WAIT_START2,
    WAIT_TYPE,
    WAIT_LEN,
    WAIT_PAYLOAD,
    WAIT_CRC1,
    WAIT_CRC2,
    WAIT_END
};

static UartState rxState      = WAIT_START1;
static uint8_t   rxBuffer[64];
static uint8_t   rxIndex      = 0;
static uint8_t   expectedLength = 0;

// =========================================================
// SHARED TARGET VARIABLES
// Protected by uartMux — read from web server task safely.
// =========================================================
static portMUX_TYPE uartMux = portMUX_INITIALIZER_UNLOCKED;

static volatile uint8_t  target_mode              = 0;
static volatile float    target_rpm               = 0.0f;
static volatile uint16_t target_steering          = 500;
static volatile uint8_t  target_brake             = 0;
static volatile bool     target_direction_reverse = false;
static volatile uint32_t last_valid_packet_time   = 0;

// =========================================================
// FUNCTION PROTOTYPES
// =========================================================
uint16_t calculateCRC16(uint8_t *data, uint8_t length);
void     processCommand(uint8_t msgType, uint8_t *payload, uint8_t length);

// =========================================================
// INIT
// =========================================================
void initUART() {
#if defined(ESP32_VCS)
    ANS_SERIAL.begin(115200, SERIAL_8N1, 16, 17);  // RX=16, TX=17
#else
    ANS_SERIAL.begin(115200);
#endif
    portENTER_CRITICAL(&uartMux);
    last_valid_packet_time = millis();
    portEXIT_CRITICAL(&uartMux);
}

// =========================================================
// UPDATE UART — SINGLE FSM PARSER ONLY
// Call this from your main loop or UART task.
// =========================================================
void updateUART() {
    while (ANS_SERIAL.available() > 0) {
        uint8_t byte = ANS_SERIAL.read();

        switch (rxState) {

            case WAIT_START1:
                if (byte == 0xAA) rxState = WAIT_START2;
                break;

            case WAIT_START2:
                rxState = (byte == 0x55) ? WAIT_TYPE : WAIT_START1;
                break;

            case WAIT_TYPE:
                rxBuffer[0] = byte;   // store msg type
                rxState = WAIT_LEN;
                break;

            case WAIT_LEN:
                // Guard: payload > 60 would overflow rxBuffer[64]
                // (need room for type[0]+len[1]+payload+crc_h+crc_l = len+4 bytes)
                if (byte > 60) {
                    rxState = WAIT_START1;
                    break;
                }
                rxBuffer[1]    = byte;
                expectedLength = byte;
                rxIndex        = 2;
                rxState        = (expectedLength > 0) ? WAIT_PAYLOAD : WAIT_CRC1;
                break;

            case WAIT_PAYLOAD:
                rxBuffer[rxIndex++] = byte;
                if (rxIndex >= expectedLength + 2) rxState = WAIT_CRC1;
                break;

            case WAIT_CRC1:
                rxBuffer[expectedLength + 2] = byte;
                rxState = WAIT_CRC2;
                break;

            case WAIT_CRC2:
                rxBuffer[expectedLength + 3] = byte;
                rxState = WAIT_END;
                break;

            case WAIT_END:
                if (byte == 0xFF) {
                    uint16_t receivedCRC = ((uint16_t)rxBuffer[expectedLength + 2] << 8)
                                         | rxBuffer[expectedLength + 3];

                    // CRC is calculated over Type(1) + Len(1) + Payload(N) = N+2 bytes
                    uint16_t calculatedCRC = calculateCRC16(rxBuffer, expectedLength + 2);

                    if (receivedCRC == calculatedCRC) {
                        // Payload starts at rxBuffer[2]
                        processCommand(rxBuffer[0], &rxBuffer[2], expectedLength);

                        portENTER_CRITICAL(&uartMux);
                        last_valid_packet_time = millis();
                        portEXIT_CRITICAL(&uartMux);
                    } else {
                        Serial.print(F("CRC ERROR: calc="));
                        Serial.print(calculatedCRC, HEX);
                        Serial.print(F(" recv="));
                        Serial.println(receivedCRC, HEX);
                    }
                }
                rxState = WAIT_START1;
                break;
        }
    }
}

// =========================================================
// PROCESS COMMAND
// msgType 0x01: Control packet from Pi
//   Payload (7 bytes): Mode(1) RPM(2,signed) Steer(2) Brake(1) Rev(1)
// =========================================================
void processCommand(uint8_t msgType, uint8_t *payload, uint8_t length) {
    if (msgType == 0x01 && length >= 7) {
        uint8_t  new_mode    = payload[0];
        int16_t  raw_rpm     = (int16_t)((payload[1] << 8) | payload[2]);
        uint16_t raw_steer   = (uint16_t)((payload[3] << 8) | payload[4]);
        uint8_t  raw_brake   = payload[5];
        bool     raw_reverse = (payload[6] == 1);

        portENTER_CRITICAL(&uartMux);
        target_mode              = new_mode;
        target_rpm               = (float)constrain(raw_rpm,   COMM_SPEED_MIN, COMM_SPEED_MAX);
        target_steering          = constrain(raw_steer, COMM_STEER_LEFT, COMM_STEER_RIGHT);
        target_brake             = constrain(raw_brake, COMM_BRAKE_MIN,  COMM_BRAKE_MAX);
        target_direction_reverse = raw_reverse;
        portEXIT_CRITICAL(&uartMux);
    }
}

// =========================================================
// CRC16 (Modbus / A001 polynomial)
// =========================================================
uint16_t calculateCRC16(uint8_t *data, uint8_t length) {
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
// Packet: [AA][55][0x02][0x07][RPM_H][RPM_L][STEER_H][STEER_L]
//         [STATE][GEAR][REV][CRC_H][CRC_L][FF]  = 14 bytes
// CRC over bytes 0..8 (Type+Len+Payload)
// =========================================================
void broadcastVehicleTelemetry() {
    uint8_t telBuffer[9];

    float    rpm;
    uint16_t steer;
    uint8_t  state = (uint8_t)currentState;
    uint8_t  gear  = 1;
    uint8_t  rev   = isReverseEngaged() ? 1 : 0;

#if defined(ESP32_VCS)
    gear = (current_drive_mode == DRIVE_LOW)  ? 0 :
           (current_drive_mode == DRIVE_HIGH) ? 2 : 1;
#else
    if      (digitalRead(PIN_SPEED_SW_LOW)  == LOW) gear = 0;
    else if (digitalRead(PIN_SPEED_SW_HIGH) == LOW) gear = 2;
#endif

#if SIMULATION_MODE
    rpm   = getSimulatedRPM();
    steer = getSimulatedSteering();
    Serial.print(F("SIM -> RPM:")); Serial.print(rpm);
    Serial.print(F(" Steer:"));    Serial.print(steer);
    Serial.print(F(" Gear:"));     Serial.print(gear);
    Serial.print(F(" Rev:"));      Serial.println(rev);
#else
    rpm   = getMeasuredRPM();
    steer = getMeasuredSteering();
#endif

    int16_t rpmInt = (int16_t)rpm;

    // Build buffer: [Type][Len][RPM_H][RPM_L][STEER_H][STEER_L][STATE][GEAR][REV]
    telBuffer[0] = 0x02;                   // msg type
    telBuffer[1] = 0x07;                   // payload length
    telBuffer[2] = (rpmInt >> 8)  & 0xFF;
    telBuffer[3] =  rpmInt        & 0xFF;
    telBuffer[4] = (steer  >> 8)  & 0xFF;
    telBuffer[5] =  steer         & 0xFF;
    telBuffer[6] = state;
    telBuffer[7] = gear;
    telBuffer[8] = rev;

    // CRC over all 9 bytes (Type + Len + Payload)
    uint16_t crc = calculateCRC16(telBuffer, 9);

    // Debug hex dump (all modes)
    Serial.print(F("TEL HEX: AA 55 "));
    for (int i = 0; i < 9; i++) {
        if (telBuffer[i] < 0x10) Serial.print(F("0"));
        Serial.print(telBuffer[i], HEX);
        Serial.print(F(" "));
    }
    if (((crc >> 8) & 0xFF) < 0x10) Serial.print(F("0"));
    Serial.print((crc >> 8) & 0xFF, HEX);
    Serial.print(F(" "));
    if ((crc & 0xFF) < 0x10) Serial.print(F("0"));
    Serial.print(crc & 0xFF, HEX);
    Serial.println(F(" FF"));

    // Binary write (live mode only)
#if SIMULATION_MODE == 0
    ANS_SERIAL.write(0xAA);
    ANS_SERIAL.write(0x55);
    ANS_SERIAL.write(telBuffer, 9);
    ANS_SERIAL.write((crc >> 8) & 0xFF);
    ANS_SERIAL.write( crc        & 0xFF);
    ANS_SERIAL.write(0xFF);
#endif
}

// =========================================================
// LEGACY sendTelemetry() — kept for compatibility
// Uses a 4-byte int32 RPM (different from broadcastVehicleTelemetry)
// =========================================================
void sendTelemetry(float rpm, uint16_t steer, float volt, uint8_t state) {
    uint8_t  telBuffer[11];
    uint16_t voltScaled = (uint16_t)(volt * 100);
    int32_t  rpmInt     = (int32_t)rpm;

    telBuffer[0]  = 0x02;
    telBuffer[1]  = 9;
    telBuffer[2]  = (rpmInt >> 24) & 0xFF;
    telBuffer[3]  = (rpmInt >> 16) & 0xFF;
    telBuffer[4]  = (rpmInt >>  8) & 0xFF;
    telBuffer[5]  =  rpmInt        & 0xFF;
    telBuffer[6]  = (steer  >>  8) & 0xFF;
    telBuffer[7]  =  steer         & 0xFF;
    telBuffer[8]  = (voltScaled >> 8) & 0xFF;
    telBuffer[9]  =  voltScaled       & 0xFF;
    telBuffer[10] = state;

    uint16_t crc = calculateCRC16(telBuffer, 11);

    ANS_SERIAL.write(0xAA);
    ANS_SERIAL.write(0x55);
    ANS_SERIAL.write(telBuffer, 11);
    ANS_SERIAL.write((crc >> 8) & 0xFF);
    ANS_SERIAL.write( crc        & 0xFF);
    ANS_SERIAL.write(0xFF);
}

// =========================================================
// GETTERS — thread-safe reads for web server task
// =========================================================
uint8_t  getANSCommandMode()    {
    portENTER_CRITICAL(&uartMux);
    uint8_t v = target_mode;
    portEXIT_CRITICAL(&uartMux);
    return v;
}

float    getTargetRPM()         {
    portENTER_CRITICAL(&uartMux);
    float v = target_rpm;
    portEXIT_CRITICAL(&uartMux);
    return v;
}

uint16_t getTargetSteering()    {
    portENTER_CRITICAL(&uartMux);
    uint16_t v = target_steering;
    portEXIT_CRITICAL(&uartMux);
    return v;
}

uint8_t  getTargetBrake()       {
    portENTER_CRITICAL(&uartMux);
    uint8_t v = target_brake;
    portEXIT_CRITICAL(&uartMux);
    return v;
}

bool     getANSReverseCommand() {
    portENTER_CRITICAL(&uartMux);
    bool v = target_direction_reverse;
    portEXIT_CRITICAL(&uartMux);
    return v;
}

bool ansHeartbeatReceived() {
    portENTER_CRITICAL(&uartMux);
    uint32_t t = last_valid_packet_time;
    portEXIT_CRITICAL(&uartMux);
    return (millis() - t) <= 500;   // 500ms — tolerates 10 missed packets at 20Hz
}

// =========================================================
// UTILITY
// =========================================================
uint8_t calculateChecksum(const String &payload) {
    uint8_t checksum = 0;
    for (unsigned int i = 0; i < payload.length(); i++)
        checksum ^= payload[i];
    return checksum;
}