#include "travel_control.hpp"

namespace asn {

TravelControl::TravelControl( MotorControl &motorControl,
                              SteerControl &steerControl ) :
    motorControl( motorControl ), steerControl( steerControl ) {
}

void TravelControl::calculateRotation() {
//     float angle = acos( ( prev_x * dest_x ) +
//                         ( prev_z * dest_z ) /
//                             ( sqrt( ( prev_x ) ^ 2 + ( prev_z ) ^ 2 ) *
//                               sqrt( ( dest_x ) ^ 2 + ( dest_z ) ^ 2 ) ) );
//     angle = angle * ( 180 / ( atan( 1 ) * 4 ) );
//     steerControl.setSetpoint( angle );
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