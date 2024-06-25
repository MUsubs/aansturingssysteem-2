#include "data_transceiver.hpp"

namespace sen {
// public
DataTransceiver::DataTransceiver( uint8_t SUB_LORA_ADDRESS, uint8_t LAND_LORA_ADDRESS ) :
    SUB_LORA_ADDRESS( SUB_LORA_ADDRESS ), LAND_LORA_ADDRESS( LAND_LORA_ADDRESS ) {
}

void DataTransceiver::sendBytes( int bytes ) {
}
int DataTransceiver::generateInstructionHeader( int inst, int n_bytes ) {
    return 0;
}
int DataTransceiver::generateUpdateHeader( int data_id, int n_bytes ) {
    return 0;
}
int DataTransceiver::generateSensHeader( int sensor, int n_bytes ) {
    return 0;
}

// private
void DataTransceiver::run() {
}
void DataTransceiver::staticRun() {
}

void DataTransceiver::passMessages() {
    int packetSize = LoRa.parsePacket();
    std::vector<byte> loraMessage;
        for ( auto iterator = begin( vector ); iterator != end( vector ); iterator++ ) {
            MessageInterpreter.byteReceived(iterator);
        } MessageInterpreter.messageDone();
}

void DataTransceiver::writeMessage( const int &bytes ) {
    LoRa.beginPacket();
    for ( auto iterator = begin( vector ); iterator != end( vector ); iterator++ ) {
        LoRa.write( iterator );
    }
    LoRa.endPacket();
}

}  // namespace sen
