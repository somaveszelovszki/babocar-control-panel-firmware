#pragma once

#include <uns/bsp/tim.hpp>

namespace uns {
namespace hw {

/* @brief Controls a DC motor using a timer configured in PWM mode.
 **/
class DC_Motor {
public:

    /* @brief Constructor - sets timer handle and timer channel.
     * @param _htim The handle for the timer used for PWM generation.
     * @param _chnlFwd The timer channel used for forward PWM generation.
     * @param _chnlBwd The timer channel used for backward PWM generation.
     **/
    DC_Motor(tim_handle_t *_htim, tim_channel_t _chnlFwd, tim_channel_t _chnlBwd);

    /* @brief Writes PWM value to the DC motor - writes PWM pin.
     * @param _pwm The PWM value to write.
     * @returns Status indicating operation success (operation fails when 'forceStopActive' flag is set).
     **/
    Status write(float32_t duty);

    void forceStop();

private:
    tim_handle_t *htim;             // The handle for the timer used for PWM generation.
    const tim_channel_t chnlFwd;    // The timer channel used for forward PWM generation.
    const tim_channel_t chnlBwd;    // The timer channel used for backward PWM generation.

    int32_t pwm;                    // The current PWM value.
    int32_t errCntr;
    bool forceStopActive;

};
} // namespace hw
} // namespace uns
