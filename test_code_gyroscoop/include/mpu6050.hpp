#ifndef R2D2_MPU6050_HPP
#define R2D2_MPU6050_HPP

#include <MPU6050_light.h>
#include <Arduino.h>
#include "Kalman.h"
#include "Wire.h"

namespace asn
{

    class Mpu6050
    {
    public:
        Mpu6050(MPU6050 &mpu, Kalman &kalmanFilter);
        void setUpGyro();
        float getCurrent_z();
        void kalman();

    private:
        MPU6050 &mpu;
        Kalman &kalmanFilter;
        float output = 0;
        unsigned long currentTime = 0;
        float prevTime = 0;
        float filteredAngle = 0;
        int int_count = 0;
    };

}
#endif // R2D2_MPU6050_HPP
