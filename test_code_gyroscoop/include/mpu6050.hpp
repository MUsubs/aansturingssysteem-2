#ifndef R2D2_MPU6050_HPP
#define R2D2_MPU6050_HPP

#include <MPU6050_light.h>
#include <Arduino.h>
#include "Wire.h"

namespace asn
{
    /**
     * @class Class Mpu6050 mpu6050.hpp
     * @brief A class that controls the mpu6050 gyroscope
     */

    class Mpu6050
    {
    public:
        Mpu6050(MPU6050 &mpu);
        /**
         * @brief Starts the setup sequence of the mpu6050
         * @details Starts the mpu6050 and applies a delay of 1 second so the mpu6050 can calibrate itself.
         * The parameters of the kalman filter are also setup here.
         */
        void setUpGyro();
        /**
         * @brief Returns the current z angle of the mpu6050
         * @return The Z-angle as calculated by the mpu6050
         */
        float getCurrent_z();

        float getAcc_z();
        
        

    private:
        MPU6050 &mpu;
        float output = 0;
        unsigned long currentTime = 0;
        float prevTime = 0;
        float filteredAngle = 0;
        int int_count = 0;
    };

}
#endif // R2D2_MPU6050_HPP
