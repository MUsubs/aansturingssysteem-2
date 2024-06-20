#ifndef R2D2_TRAVELCONTROL_HPP
#define R2D2_TRAVELCONTROL_HPP

#include <cmath>
#include "motor_control.hpp"
#include "steer_control.hpp"

namespace asn {

/**
 * @class Class travel_control.hpp
 * @brief A control class thaat controls the submarine travel.
 */
class TravelControl
{
public:
    TravelControl(MotorControl &motorControl, SteerControl &steerControl);
     /**
     * @brief Calculates the needed rotation.
     * @details Calculates the rotation in degrees that the submarine
     * must make in order to get on course to the destination.
     * @param cur_x The current x-coordinate of the submarine.
     * @param cur_z The current z-coordinate of the submarine.
     */
    void calculateRotation(const float cur_x, const float cur_z);
    /**
     * @brief Stops the submarine.
     */
    void stop();
    void newDest();
    void updateCurPos(const float cur_x, const float cur_z);
    void main();

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