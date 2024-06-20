#include "dummyControl.hpp"

namespace asn {

DummyControl::DummyControl( TravelControl& travelControl ) :
    travelControl( travelControl ) {
}

void DummyControl::main() {
    Serial.println("start dummy");
    for(;;){
        // Serial.println("start cycle");
        travelControl.updateCurPos( 0.26, 0.99 );
        // Serial.println("end cycle");
    }
}

}  // namespace asn
