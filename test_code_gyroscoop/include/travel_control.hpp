#ifndef R2D2_TRAVELCONTROL_HPP
#define R2D2_TRAVELCONTROL_HPP

#include <cmath>
#include "motor_control.hpp"
#include "steer_control.hpp"

namespace asn {

class TravelControl
{
public:
    TravelControl(MotorControl &motorControl, SteerControl &steerControl);
    void calculateRotation(const float cur_x, const float cur_z);
    void stop();
    void newDest();
    void updateCurPos();

private:
    MotorControl &motorControl;
    SteerControl &steerControl;
    float dest_x = 0.64;
    float dest_y;
    float dest_z = 0.12;
    float prev_x = 0.23;
    float prev_y;
    float prev_z = 0.99;
    float goal_direction;
};

} // namespace asn
#endif // R2D2_TRAVELCONTROL_HPP