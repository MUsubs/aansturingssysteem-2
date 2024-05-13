#ifndef _MPU6050_HPP
#define _MPU6050_HPP

#include <Wire.h>
#include <Arduino.h>

class Mpu6050{
private:
    float x_as, y_as, z_as, roll, pitch, output;
    const int mpu_addr1 = 0x68;

public:
    Mpu6050();
    void gyroscoopSetup();
    float getAngle(int as);
	
};

#endif