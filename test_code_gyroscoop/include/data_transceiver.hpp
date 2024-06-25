#ifndef R2D2_DATA_TRANSCEIVER_HPP
#define R2D2_DATA_TRANSCEIVER_HPP

#include <Arduino.h>
#include <LoRa.h>
#include <SPI.h>

#include <deque>
#include <vector>

#include "FreeRTOS.h"
#include "queue.h"

namespace sen {

const uint8_t SUB_LORA_ADDRESS = 0x33;
const uint8_t LAND_LORA_ADDRESS = 0x0B;

class DataTransceiver {
public:
    DataTransceiver( const uint8_t SUB_LORA_ADDRESS, const uint8_t LAND_LORA_ADDRESS );

    void sendBytes( std::deque bytes );
    uint8_t generateInstructionHeader( Instruction_t inst, uint8_t n_bytes );
    uint8_t generateUpdateHeader( uint8_t data_id, uint8_t n_bytes );
    uint8_t generateSensHeader( Sensor_t sensor, uint8_t n_bytes );

private:
    void run();
    static void staticRun();
    void passMessages();
    void writeMessage( const std::vector<uint8_t>& bytes );

    int miso_pin;
    int mosi_pin;
    int sck_pin;
    int nss_pin;
    int rst_pin;
    int dio0_pin;
};
}  // namespace sen

#endif  // R2D2_DATA_TRANSCEIVER_HPP
