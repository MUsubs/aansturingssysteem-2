#include "mpu6050.hpp"

namespace asn {

Mpu6050::Mpu6050( MPU6050 &mpu, Kalman &kalmanFilter ) :
    mpu( mpu ), kalmanFilter( kalmanFilter ) {
}

void Mpu6050::setUpGyro() {
    Wire.begin();
    byte status = mpu.begin();
    delay( 1000 );
    mpu.calcOffsets();
    kalmanFilter.setAngle( 0.0f );
    kalmanFilter.setQangle( 0.001f );
    kalmanFilter.setQbias( 0.0067f );
    kalmanFilter.setRmeasure( 0.075f );
    prevTime = millis();
}
float Mpu6050::getSetpoint() {
    return setpoint;
}

void Mpu6050::setSetpoint( float s ) {
    setpoint = s;
}

float Mpu6050::getCurrent_z() {
    mpu.update();
    return mpu.getAngleZ();
}

void Mpu6050::kalman() {
    currentTime = millis();

    output = kalmanFilter.getAngle( mpu.getAngleZ(), mpu.getAccZ(),
                                    ( currentTime - prevTime ) / 1000 );

    prevTime = currentTime;
}

}  // namespace asn