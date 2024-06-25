#include "data_transceiver.hpp"

namespace sen {
// public
DataTransceiver::DataTransceiver( const uint8_t sub_address, const uint8_t land_address, bool is_sub ) :
    sub_address( sub_address ), land_address( land_address ), is_sub( is_sub ) {
}

void DataTransceiver::sendBytes( std::vector<uint8_t>& bytes ) {
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
    byte recipient = 0;
    if ( packetSize > 0 ) {
        recipient = LoRa.read();
    } else {
        return;
    }
    if ( recipient != ( is_sub ? sub_address : land_address ) ) {
        for ( int i = 0; i < packetSize; i++ ) {
            LoRa.read();
        }
        return;
    }
    for ( int i = 0; i < packetSize; i++ ) {
        MessageInterpreter.byteReceived( LoRa.read() );
    }
    MessageInterpreter.messageDone();
}

void DataTransceiver::writeMessage( const std::vector<uint8_t>& bytes ) {
    LoRa.beginPacket();
    LoRa.write( is_sub ? sub_address : land_address );
    for ( const auto& b : bytes ) {
        LoRa.write( b );
    }
    LoRa.endPacket();
}

}  // namespace sen
