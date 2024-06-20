#include "steer_control.hpp"

namespace asn {

SteerControl::SteerControl( Mpu6050 &mpu, MotorControl &motorControl ) :
    mpu( mpu ), motorControl( motorControl ) {
}


void SteerControl::setSetpoint( float s ) {
    setpoint = s;
}

float SteerControl::highPassFilter( float current_value,
                                    float previous_value ) {
    return alpha * ( previous_value + current_value - alpha * previous_value );
}

void SteerControl::PID() {
    float gyro_z = mpu.getCurrent_z();
    float current_z = highPassFilter( gyro_z, previous_z );

    error = setpoint - current_z;
    error_sum += error * dt;
    error_div = ( error - error_prev ) / dt;
    steer_action = ( kp * error + ki * error_sum + kd * error_div );
    error_prev = error;

    pos_prev = steer_action;
    previous_z = current_z;

    if ( round( mpu.getCurrent_z() ) < steer_action ) {
        Serial.println( "LEFT" );
        motorControl.move( motorControl.direction_t::LEFT );
        vTaskDelay( 50 );
    } else if ( round( mpu.getCurrent_z() ) > steer_action ) {
        Serial.println( "RIGHT" );
        motorControl.move( motorControl.direction_t::RIGHT );
        vTaskDelay( 50 );
    } else if ( round( mpu.getCurrent_z() ) == steer_action ) {
        Serial.println( "DONE" );
        // motorControl.move( motorControl.direction_t::FORWARD );
        vTaskDelay( 50 );
    }
}

void SteerControl::main() {
    Serial.println( "SteerControl main" );
    for ( ;; ) {
        PID();
    }
}

}  // namespace asn
