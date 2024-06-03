#include "Wire.h"
#include "Mpu6050.hpp"

VarSpeedServo my_servo;
MPU6050 mpu( Wire );
Mpu6050 gyro( my_servo, mpu );
unsigned long timer = 0;

void setup() {
    Serial.begin( 9600 );
    Wire.begin();
    gyro.setGyroUp();
}

void loop() {

    if ( Serial.available() > 0 ) {
      String inputString = Serial.readStringUntil( '\n' );
      gyro.setSetpoint( inputString.toFloat() );
    }
    
    // my_servo.write( gyro.PID(), 30 );
    my_servo.write( gyro.PID(), 30 );

    delay( 5 );
}