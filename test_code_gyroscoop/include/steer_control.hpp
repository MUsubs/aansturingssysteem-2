#ifndef R2D2_STEERCONTROL_HPP
#define R2D2_STEERCONTROL_HPP

#include "mpu6050.hpp"
#include "motor_control.hpp"
#include "Kalman.h"

namespace asn
{
/**
 * @class Class steer_control.hpp
 * @brief A class that controls the steering of the submarine.
 */
    class SteerControl
    {
    public:
        SteerControl(Mpu6050 &mpu, MotorControl &motorControl, Kalman &kalmanFilter);
        /**
         * @brief Sets the setpoint to a given value.
         * @param s The parameter to set the setpoint to.
         */
        void setSetpoint(float s);
        /**
         * @brief Implements a PID to reduce measurement noise.
         */
        void PID();
        /**
         * @brief Initializes SteerControl and Kalman variables and starts the setup for the mpu6050.
         */
        void setUpSteerControl();
        void main();

    private:
    /**
     * @brief Implements a high-pass filter to reduce measurement noise.
     * @param current_value The currently held value of the filter.
     * @param previous_value The previously held value of the filter.
     * @return Returns a filtered value.
     */
        float highPassFilter(float current_value, float previous_value);
        /**
         * @brief Applies Kalman filter to the steering action.
         */
        void kalman();


        Mpu6050 &mpu;
        MotorControl &motorControl;
        Kalman &kalmanFilter;
        float steer_action;
        float previous_z = 0.0;
        float pos_prev = 0.0;
        double setpoint = 0.0;
        double error = 0.0;
        double error_sum = 0.0;
        double error_prev = 0.0;
        double error_div = 0.0;
        const double kp = 0.725;
        const double ki = 1.02;
        const double kd = 0.01;
        const double dt = 0.1;
        const float alpha = 0.8;
        unsigned long currentTime = 0;
        float prevTime = 0;
    };

} // namespace asn
#endif // R2D2_STEERCONTROL_HPP