#include <config/cfg_track.hpp>
#include "util/debug.hpp"
#include <uns/hw/DC_Motor.hpp>
#include <uns/util/debug.hpp>
#include <uns/util/numeric.hpp>
#include <uns/config/cfg_car.hpp>
#include <uns/CarProps.hpp>
#include <uns/bsp/it.hpp>

using namespace uns;

extern uint32_t rcRecvInSpeed;
extern CarProps car;

namespace {

constexpr int32_t PWM_PERIOD = 16 * 20;

constexpr int32_t chnl2_pwm(const int32_t fwdPwm) {
    return PWM_PERIOD - fwdPwm;
}

constexpr int32_t PWM_FWD_MAX = static_cast<int32_t>(PWM_PERIOD * 0.85f);
constexpr int32_t PWM_BWD_MAX = chnl2_pwm(PWM_FWD_MAX);
constexpr int32_t PWM_STOP = (PWM_FWD_MAX + PWM_BWD_MAX) / 2;

constexpr int32_t ERROR_LIMIT = 3;
}

hw::DC_Motor::DC_Motor(tim_handle_t *_htim, tim_channel_t _chnlFwd, tim_channel_t _chnlBwd)
    : htim(_htim)
    , chnlFwd(_chnlFwd)
    , chnlBwd(_chnlBwd)
    , pwm(PWM_STOP)
    , errCntr(0)
    , forceStopActive(false) {}

Status hw::DC_Motor::write(float32_t duty) {
    Status status = forceStopActive ? Status::BUSY : Status::OK;

    if (isOk(status)) {
        if (cfg::USE_SAFETY_ENABLE_SIGNAL) {
            int32_t recvPwm = static_cast<int32_t>(rcRecvInSpeed);

            // after a given number of errors, stops motor
            if (recvPwm < 900 || recvPwm > 2100) {
                if (++this->errCntr >= ERROR_LIMIT) {
                    recvPwm = 1500;
                }   // else: does not change motor pwm value
            } else {
                this->errCntr = 0;

                if (recvPwm > 1200 && recvPwm < 1800) {
                    recvPwm = 1500;
                }

                if (recvPwm <= 1500) {
                    this->pwm = uns::map(recvPwm, 1500L, 1000L, PWM_STOP, PWM_BWD_MAX);
                } else {
                    this->pwm = uns::map(duty, -1.0f, 1.0f, PWM_BWD_MAX, PWM_FWD_MAX);
                }
            }
        } else {
            this->pwm = uns::map(duty, -1.0f, 1.0f, PWM_BWD_MAX, PWM_FWD_MAX);
        }

    } else {
        this->pwm = PWM_STOP;
    }

    this->pwm = uns::incarcerate(this->pwm, PWM_BWD_MAX, PWM_FWD_MAX);
    const int32_t pwm2 = chnl2_pwm(this->pwm);

    uns::enterCritical();
    uns::writePWM(this->htim, this->chnlFwd, static_cast<uint32_t>(this->pwm));
    uns::writePWM(this->htim, this->chnlBwd, static_cast<uint32_t>(pwm2));
    uns::exitCritical();

    return status;
}

void hw::DC_Motor::forceStop() {
    static constexpr speed_t DEADBAND(mm_per_sec(), 50.0f);

    // TODO
    if (car.speed() > DEADBAND) {
        this->write(-1.0f);
        uns::blockingDelay(time_t::from<milliseconds>(1000.0f));
    } else if (car.speed() < -DEADBAND) {
        this->write(0.2f);
        uns::blockingDelay(time_t::from<milliseconds>(1000.0f));
    }
    this->write(0.0f);
    forceStopActive = true;
}
