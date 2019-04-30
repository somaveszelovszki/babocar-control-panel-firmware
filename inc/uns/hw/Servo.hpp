#pragma once

#include <uns/bsp/tim.hpp>

namespace uns {
namespace hw {
/* @brief Controls a servo motor using a timer configured in PWM mode.
 **/
class Servo {
private:
    tim_handle_t * const htim;  // The handle for the timer used for PWM generation.
    const uint32_t chnl;        // The timer channel used for PWM generation.

    const angle_t angle_min; // The minimum angle.
    const angle_t angle_max; // The maximum angle.
    angle_t _angle;          // The current angle.

public:
    /* @brief Constructor - sets timer handle, timer channel, minimum and maximum angles.
     * @param _htim The handle for the timer used for PWM generation.
     * @param _chnl The timer channel used for PWM generation.
     * @param _angle_min The minimum angle.
     * @param _angle_max The maximum angle.
     **/
    Servo(tim_handle_t * const _htim, tim_channel_t _chnl, angle_t _angle_min, angle_t _angle_max)
        : htim(_htim)
        , chnl(_chnl)
        , angle_min(_angle_min)
        , angle_max(_angle_max) {}

    /* @brief Writes angle to the servo motor - converts value to PWM duty, and writes PWM pin.
     * @param _ang The angle to write.
     **/
    void write(angle_t _ang);

    /* @brief Gets servo angle.
     * @returns The servo angle.
     **/
    angle_t getAngle() const {
        return this->_angle;
    }
};
} // namespace hw
} // namespace uns
