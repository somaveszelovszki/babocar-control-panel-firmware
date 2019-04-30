#pragma once

#include <uns/hw/Servo.hpp>
#include <uns/config/cfg_car.hpp>

namespace uns {
namespace hw {

/* @brief Controls a steering servo motor using a timer configured in PWM mode.
 **/
class SteeringServo : private Servo {
private:
    const angle_t angle_mid;     // The middle position angle.

public:
    /* @brief Constructor - initializes timer handle, channel, middle servo angle and delta maximum wheel angle.
     * @param _htim The handle for the timer used for PWM generation.
     * @param _chnl The timer channel used for PWM generation.
     * @param _angle_mid The middle servo angle.
     * @param _wheelAngle_d_max The maximum delta wheel angle.
     **/
    SteeringServo(tim_handle_t * const _htim, tim_channel_t _chnl, angle_t _angle_mid, angle_t _wheelAngle_d_max)
        : Servo(_htim, _chnl, _angle_mid - _wheelAngle_d_max / cfg::SERVO_WHEEL_TR, _angle_mid + _wheelAngle_d_max / cfg::SERVO_WHEEL_TR)
        , angle_mid(_angle_mid) {}

    /* @brief Writes wheel angle - converts value to servo angle and writes it to the PWM pin.
     * @param wheelAngle The wheel angle to write.
     **/
    void writeWheelAngle(angle_t wheelAngle) {
        this->write(this->angle_mid + wheelAngle / cfg::SERVO_WHEEL_TR);
    }

    /* @brief Writes the middle angle to the servo, thus positioning the wheels middle.
     **/
    void positionMiddle() {
        this->write(this->angle_mid);
    }

    /* @brief Gets wheel angle - converts servo angle to wheel angle and returns it.
     * @returns The wheel angle.
     **/
    angle_t getWheelAngle() const {
        return (this->getAngle() - this->angle_mid) * cfg::SERVO_WHEEL_TR;
    }
};
} // namespace hw
} // namespace uns
