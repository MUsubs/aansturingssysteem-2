#ifndef R2D2_MOTORCONTROL_HPP
#define R2D2_MOTORCONTROL_HPP

#include <Arduino.h>

#include "motor.hpp"

namespace asn
{

    class MotorControl
    {
    public:
        MotorControl(uint8_t *pins);
        enum direction_t
        {
            LEFT,
            RIGHT,
            FORWARD,
            BACKWARD,
            UP,
            DOWN,
            STOP
        };
        void move(direction_t direction);

    private:
        Motor motor;
    };

} // namespace asn

#endif // R2D2_MOTORCONTROL_HPP
