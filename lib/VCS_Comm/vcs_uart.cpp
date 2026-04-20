#include "vcs_uart.h"
#include "vcs_hallsensor.h"
#include "vcs_steering.h"
#include "vcs_simulation.h"
#include "vcs_state_machine.h"
#include "vcs_pins.h"
#include "vcs_reverse.h"

    #ifdef NANO_33_BLE
        #define ANS_SERIAL Serial1
    #else
        #define ANS_SERIAL Serial
    #endif

enum UartState { 
    WAIT_START1, WAIT_START2, WAIT_TYPE, WAIT_LEN, WAIT_SEQ, 
    WAIT_PAYLOAD, WAIT_CRC1, WAIT_CRC2, WAIT_END 
};

static UartState rxState = WAIT_START1;
static uint8_t rxBuffer[64];
static uint8_t rxIndex = 0;
static uint8_t expectedLength = 0;

static uint8_t target_mode = 0;
static float target_rpm = 0.0f;
static uint16_t target_steering = 500;
static uint8_t target_brake = 0;
static uint32_t last_valid_packet_time = 0;
static bool target_direction_reverse = false; 

// Function Prototypes
uint16_t calculateCRC16(uint8_t *data, uint8_t length);
void processCommand(uint8_t msgType, uint8_t *payload, uint8_t length);

void initUART() {
    ANS_SERIAL.begin(115200); 
    last_valid_packet_time = millis();
}

void updateUART() {
    while (ANS_SERIAL.available() > 0) {
        uint8_t byte = ANS_SERIAL.read();

        switch (rxState) {
            case WAIT_START1:
                if (byte == 0xAA) rxState = WAIT_START2;
                break;
                
            case WAIT_START2:
                if (byte == 0x55) rxState = WAIT_TYPE;
                else rxState = WAIT_START1;
                break;
                
            case WAIT_TYPE:
                rxBuffer[0] = byte;
                rxState = WAIT_LEN;
                break;
                
            case WAIT_LEN:
                // Bounds check BEFORE we commit to parsing this packet.
                // rxBuffer is 64 bytes and we need room for:
                //   [0]=type  [1]=len  [2..len+1]=payload  [len+2]=CRC hi
                //   [len+3]=CRC lo
                // so the maximum safe payload length is 60. A bit-flipped or
                // hostile LEN=0xFF would otherwise walk past the buffer and
                // the WAIT_PAYLOAD exit condition (rxIndex is uint8_t) would
                // never trigger.
                if (byte > 60) {
                    rxState = WAIT_START1;
                    break;
                }
                rxBuffer[1] = byte;
                expectedLength = byte;
                rxState = WAIT_SEQ;
                break;

            case WAIT_SEQ:
                rxState = (expectedLength > 0) ? WAIT_PAYLOAD : WAIT_CRC1;
                rxIndex = 2; 
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
                    uint16_t receivedCRC = (rxBuffer[expectedLength + 2] << 8) | rxBuffer[expectedLength + 3];
                    uint16_t calculatedCRC = calculateCRC16(rxBuffer, expectedLength + 2);
                    
                    if (receivedCRC == calculatedCRC) {
                        processCommand(rxBuffer[0], &rxBuffer[2], expectedLength);
                        last_valid_packet_time = millis();
                    }
                }
                rxState = WAIT_START1;
                break;
        }
    }
}


void processCommand(uint8_t msgType, uint8_t *payload, uint8_t length) {
    if (msgType == 0x01 && length >= 7) { 
        target_mode = payload[0];
        
        int16_t raw_speed = (int16_t)((payload[1] << 8) | payload[2]);
        target_rpm = (float)constrain(raw_speed, COMM_SPEED_MIN, COMM_SPEED_MAX);
        
        uint16_t raw_steer = (uint16_t)((payload[3] << 8) | payload[4]);
        target_steering = constrain(raw_steer, COMM_STEER_LEFT, COMM_STEER_RIGHT);
        
        target_brake = constrain(payload[5], COMM_BRAKE_MIN, COMM_BRAKE_MAX);
        
        target_direction_reverse = (payload[6] == 1); 
    }
}

bool getANSReverseCommand() { 
    return target_direction_reverse; 
}

uint16_t calculateCRC16(uint8_t *data, uint8_t length) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

uint8_t getANSCommandMode() { return target_mode; }
float getTargetRPM() { return target_rpm; }
uint16_t getTargetSteering() { return target_steering; }
uint8_t getTargetBrake() { return target_brake; }

bool ansHeartbeatReceived() {
    return (millis() - last_valid_packet_time) <= 200;
}

void sendTelemetry(float rpm, uint16_t steer, float volt, uint8_t state) {
    uint8_t telBuffer[12];
    uint16_t voltScaled = (uint16_t)(volt * 100);
    int32_t rpmInt = (int32_t)rpm; 

    telBuffer[0] = 0x02; 
    telBuffer[1] = 9;    
    
    telBuffer[2] = (rpmInt >> 24) & 0xFF;
    telBuffer[3] = (rpmInt >> 16) & 0xFF;
    telBuffer[4] = (rpmInt >> 8) & 0xFF;
    telBuffer[5] = rpmInt & 0xFF;
    telBuffer[6] = (steer >> 8) & 0xFF;
    telBuffer[7] = steer & 0xFF;
    telBuffer[8] = (voltScaled >> 8) & 0xFF;
    telBuffer[9] = voltScaled & 0xFF;
    telBuffer[10] = state;

    uint16_t crc = calculateCRC16(telBuffer, 11);

    ANS_SERIAL.write(0xAA);
    ANS_SERIAL.write(0x55);
    ANS_SERIAL.write(telBuffer, 11);
    ANS_SERIAL.write((crc >> 8) & 0xFF);
    ANS_SERIAL.write(crc & 0xFF);
    ANS_SERIAL.write(0xFF);
}

void broadcastVehicleTelemetry() {
    uint8_t telBuffer[9];
    
    float rpm;
    uint16_t steer;
    uint8_t state = (uint8_t)currentState;
    
    uint8_t gear = 1;
    if (digitalRead(PIN_SPEED_SW_LOW) == LOW) gear = 0;
    else if (digitalRead(PIN_SPEED_SW_HIGH) == LOW) gear = 2;
    
    uint8_t rev = isReverseEngaged() ? 1 : 0;

    #if SIMULATION_MODE
        rpm = getSimulatedRPM();
        steer = getSimulatedSteering();
        
        // SECURITY FIX: F() macro applied to prevent SRAM exhaustion
        Serial.print(F("SIM DATA -> RPM: ")); Serial.print(rpm);
        Serial.print(F(" | Steer: ")); Serial.print(steer);
        Serial.print(F(" | Gear: ")); Serial.print(gear);
        Serial.print(F(" | Rev: ")); Serial.println(rev);
    #else
        rpm = getMeasuredRPM();
        steer = getMeasuredSteering();
    #endif

    int16_t rpmInt = (int16_t)rpm; 

    telBuffer[0] = 0x02;
    telBuffer[1] = 0x07;
    
    telBuffer[2] = (rpmInt >> 8) & 0xFF;
    telBuffer[3] = rpmInt & 0xFF;
    telBuffer[4] = (steer >> 8) & 0xFF;
    telBuffer[5] = steer & 0xFF;
    telBuffer[6] = state;
    telBuffer[7] = gear;
    telBuffer[8] = rev;

    uint16_t crc = calculateCRC16(telBuffer, 9);

    // =========================================================
    // UNIVERSAL HEX PRINT: Executes in ALL modes
    // =========================================================
    Serial.print(F("HEX PAYLOAD: AA 55 "));
    for(int i = 0; i < 9; i++) {
        if(telBuffer[i] < 0x10) Serial.print(F("0")); // Add leading zero
        Serial.print(telBuffer[i], HEX); 
        Serial.print(F(" "));
    }
    
    if(((crc >> 8) & 0xFF) < 0x10) Serial.print(F("0"));
    Serial.print((crc >> 8) & 0xFF, HEX); Serial.print(F(" "));
    
    if((crc & 0xFF) < 0x10) Serial.print(F("0"));
    Serial.print(crc & 0xFF, HEX); 
    Serial.println(F(" FF"));

    // =========================================================
    // RAW BINARY WRITE: Only executes in LIVE mode
    // =========================================================
    #if SIMULATION_MODE == 0
        ANS_SERIAL.write(0xAA);
        ANS_SERIAL.write(0x55);
        ANS_SERIAL.write(telBuffer, 9);
        ANS_SERIAL.write((crc >> 8) & 0xFF);
        ANS_SERIAL.write(crc & 0xFF);
        ANS_SERIAL.write(0xFF);
    #endif
}

uint8_t calculateChecksum(const String& payload) {
    uint8_t checksum = 0;
    for (unsigned int i = 0; i < payload.length(); i++) {
        checksum ^= payload[i];
    }
    return checksum;
}