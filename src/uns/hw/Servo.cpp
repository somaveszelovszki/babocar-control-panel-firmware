#include "util/debug.hpp"
#include <uns/bsp/tim.hpp>
#include <uns/hw/Servo.hpp>
#include <uns/util/debug.hpp>
#include <uns/util/numeric.hpp>
#include <uns/util/unit_utils.hpp>

using namespace uns;

namespace {
constexpr uint32_t PWM_DUTY_0 = 800;        // PWM duty for 0 radians (timer period is 20000)
constexpr uint32_t PWM_DUTY_PI = 2200;      // PWM duty for PI radians (timer period is 20000)
} // namespace

void hw::Servo::write(radian_t _ang) {
    _ang = uns::clamp(_ang, this->angle_min, this->angle_max);

    if (this->_angle != _ang) {
        this->_angle = _ang;
        uint32_t pwm = uns::map(this->_angle.get(), 0.0f, PI.get(), PWM_DUTY_0, PWM_DUTY_PI);
        uns::writePWM(this->htim, this->chnl, pwm);
    }
}
