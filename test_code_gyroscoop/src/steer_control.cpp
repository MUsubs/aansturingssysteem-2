#include "steer_control.hpp"

namespace asn {

SteerControl::SteerControl( Mpu6050 &mpu, MotorControl &motorControl, Kalman &kalmanFilter) :
    mpu( mpu ), motorControl( motorControl ), kalmanFilter( kalmanFilter ) {
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
        motorControl.move( motorControl.direction_t::LEFT );
        delay( 50 );
    } else if ( round( mpu.getCurrent_z() ) > steer_action ) {
        motorControl.move( motorControl.direction_t::RIGHT );
        delay( 50 );
    }
}

void SteerControl::kalman() {
    currentTime = millis();

    steer_action = kalmanFilter.getAngle( mpu.getCurrent_z(), mpu.getAcc_z(),
                                    ( currentTime - prevTime ) / 1000 );

    prevTime = currentTime;
}

void SteerControl::setUpSteerControl(){
    mpu.setUpGyro();
    kalmanFilter.setAngle( 0.0f );
    kalmanFilter.setQangle( 0.001f );
    kalmanFilter.setQbias( 0.0067f );
    kalmanFilter.setRmeasure( 0.075f );
    prevTime = millis();
}

}  // namespace asn
