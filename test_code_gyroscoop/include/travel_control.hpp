#ifndef R2D2_TRAVELCONTROL_HPP
#define R2D2_TRAVELCONTROL_HPP

#include <cmath>
#include "motor_control.hpp"
#include "steer_control.hpp"

namespace asn {

class TravelControl
{
public:
    TravelControl();
    void calculateRotation();
    void stop();
    void newDest();
    void updateCurPos();

private:
    float dest_x;
    float dest_y;
    float dest_z;
    float prev_x;
    float prev_y;
    float prev_z;
    float goal_direction;
};

} // namespace asn
#endif // R2D2_TRAVELCONTROL_HPP