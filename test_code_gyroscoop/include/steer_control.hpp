#ifndef R2D2_STEERCONTROL_HPP
#define R2D2_STEERCONTROL_HPP

#include "mpu6050.hpp"
#include "motor_control.hpp"

namespace asn
{
/**
 * @class Class SteerControl.hpp
 * @brief A class that controls the steering of the submarine
 */
    class SteerControl
    {
    public:
        SteerControl(Mpu6050 &mpu, MotorControl &motorControl);
        /**
         * @brief Sets the setpoint to a given value.
         * @param s The parameter to set the setpoint to.
         */
        void setSetpoint(float s);
        /**
         * @brief Implements a PID to reduce noise.
         */
        void PID();

    private:
    /**
     * @Laurens650 YEET
     */
        float highPassFilter(float current_value, float previous_value);

        Mpu6050 &mpu;
        MotorControl &motorControl;
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
    };

} // namespace asn
#endif // R2D2_STEERCONTROL_HPP