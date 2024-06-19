#include "mpu6050.hpp"

namespace asn
{

    Mpu6050::Mpu6050(VarSpeedServo &my_servo, MPU6050 &mpu,
                     Kalman &kalmanFilter) : my_servo(my_servo), mpu(mpu), kalmanFilter(kalmanFilter)
    {
    }

    float Mpu6050::highPassFilter(float current_value, float previous_value)
    {
        return alpha * (previous_value + current_value - alpha * previous_value);
    }

    void Mpu6050::setUpGyro()
    {
        Wire.begin();
        my_servo.attach(9);
        byte status = mpu.begin();
        delay(1000);
        mpu.calcOffsets();
        kalmanFilter.setAngle(0.0f);
        kalmanFilter.setQangle(0.001f);
        kalmanFilter.setQbias(0.0067f);
        kalmanFilter.setRmeasure(0.075f);
        prevTime = millis();
    }

    float Mpu6050::PID()
    {
        mpu.update();
        float gyro_z = mpu.getAngleZ();
        float current_z = highPassFilter(gyro_z, previous_z);

        error = setpoint - current_z;
        error_sum += error * dt;
        error_div = (error - error_prev) / dt;
        servo_pos = (kp * error + ki * error_sum + kd * error_div);
        error_prev = error;

        pos_prev = servo_pos;
        previous_z = current_z;

        return servo_pos;
    }

    float Mpu6050::getSetpoint()
    {
        return setpoint;
    }

    float Mpu6050::getServo_pos()
    {
        return servo_pos;
    }

    void Mpu6050::setSetpoint(float s)
    {
        setpoint = s;
    }

    float Mpu6050::getCurrent_z()
    {
        mpu.update();
        return mpu.getAngleZ();
    }

    void Mpu6050::kalman()
    {
        currentTime = millis();

        output = kalmanFilter.getAngle(mpu.getAngleZ(), mpu.getAccZ(),
                                       (currentTime - prevTime) / 1000);

        prevTime = currentTime;
    }

} // namespace asn