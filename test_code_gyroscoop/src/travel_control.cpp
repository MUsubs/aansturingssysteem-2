#include "travel_control.hpp"

namespace asn {

TravelControl::TravelControl( MotorControl &motorControl,
                              SteerControl &steerControl ) :
    motorControl( motorControl ), steerControl( steerControl ) {
}

void TravelControl::calculateRotation(const float cur_x, const float cur_z) {
    float angle = acos( (((cur_x - prev_x) * (dest_x - cur_x)) +
                        ((cur_z - prev_z) *  (dest_z - cur_z))) /
                            ( sqrt( pow((cur_x - prev_x), 2) + pow((cur_z - prev_z), 2)) *
                              sqrt( pow((dest_x - cur_x), 2) + pow((dest_z - cur_z), 2))) );
    angle = angle * ( 180 / ( atan( 1 ) * 4 ) );
    Serial.println(angle);
    steerControl.setSetpoint( angle );
}

void TravelControl::stop() {
    motorControl.move( motorControl.direction_t::STOP );
}

void TravelControl::newDest() {
}

void TravelControl::updateCurPos() {
}

}  // namespace asn

// angle = cos^-1((a_x * b_x) + (a_z * b_z) / (sqrt((a_x)^2 + (a_z)^2) *
// sqrt((b_x)^2 + (b_z)^2)))