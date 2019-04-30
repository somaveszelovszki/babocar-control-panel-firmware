#pragma once

#include <uns/base/Runnable.hpp>

namespace uns {
/* @brief PI controller implementation - used for controlling speed.
 **/
class PI_Controller : public Runnable {
public:
    PI_Controller()
        : Runnable()
        , Ti(time_t::ZERO())
        , Kc(0.0f)
        , b0(0.0f)
        , b1(0.0f)
        , prevError(0.0f)
        , outMin(0.0f)
        , outMax(0.0f)
        , desired(speed_t::ZERO())
        , output(0.0f) {}

    /* @brief Constructor - sets period time and term weights.
     **/
    PI_Controller(uns::time_t period_, uns::time_t Ti_, float32_t Kc_, float32_t outMin_, float32_t outMax_)
        : Runnable(period_)
        , Ti(Ti_)
        , Kc(Kc_)
        , b0(Kc_ * (1 + Ti_ / period_))
        , b1(-Kc_)
        , prevError(0.0f)
        , outMin(outMin_)
        , outMax(outMax_)
        , desired(speed_t::ZERO())
        , output(0.0f) {}

    /* @brief Initializes sensor.
     * @returns Status indicating operation success.
     **/
    Status initialize();

    void setKc(float32_t Kc_) {
        this->Kc = Kc_;
        this->b0 = Kc_ * (1 + this->Ti / this->period);
        this->b1 = -Kc_;
    }

    void setParams(const PI_Controller& other) {
        this->setKc(other.Kc);
    }

    /* @brief Gets output.
     * @returns The output.
     **/
    float32_t getOutput() const {
        return this->output;
    }

    void setDesired(speed_t desired_);

    /* @brief Updates output according to current and previous errors.
     * @note If an error occurs (see result status), output will not be updated!
     * @note This is an internal method, use MEASURE(meas) macro instead!
     * @param measured The current measured speed.
     * @returns Status indicating operation success.
     **/
    Status run(speed_t measured);

private:
    time_t Ti;
    float32_t Kc;   // Weight of the proportional term.
    float32_t b0, b1;
    float32_t prevError;
    float32_t outMin;
    float32_t outMax;
    speed_t desired;      // The desired value.
    float32_t output;       // The output - updated in every cycle, holds the output value until the next update.
};
} // namespace uns
