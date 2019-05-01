#pragma once

#include <uns/base/Runnable.hpp>

namespace uns {
/* @brief PID controller implementation - used for controlling speed.
 **/
class PID_Controller : public Runnable {
public:

    struct Params {
        float32_t Kp;   // Weight of the proportional term.
        float32_t Ki;   // Weight of the integral term.
        float32_t Kd;   // Weight of the derivative term.
    };

    PID_Controller()
        : Runnable()
        , params{ 0.0f, 0.0f, 0.0f }
        , outMin(0.0f)
        , outMax(0.0f)
        , desired(0.0f)
        , measured(0.0f)
        , output(0.0f)
        , integral(0.0f)
        , maxIntegralRate(0.0f)
        , maxIntegral(0.0f) {}

    explicit PID_Controller(millisecond_t _period)
        : Runnable(_period)
        , params{ 0.0f, 0.0f, 0.0f }
        , outMin(0.0f)
        , outMax(0.0f)
        , desired(0.0f)
        , measured(0.0f)
        , output(0.0f)
        , integral(0.0f)
        , maxIntegralRate(0.0f)
        , maxIntegral(0.0f) {}

    /* @brief Constructor - sets period time and term weights.
     * @param _Kp The weight of the proportional term.
     * @param _Ki The weight of the integral term.
     * @param _Kd The weight of the derivative term.
     **/
    PID_Controller(millisecond_t _period, float32_t _Kp, float32_t _Ki, float32_t _Kd, float32_t _outMin, float32_t _outMax, float32_t _maxIntegralRate)
        : Runnable(_period)
        , params{ _Kp, _Ki, _Kd }
        , outMin(_outMin)
        , outMax(_outMax)
        , desired(0.0f)
        , measured(0.0f)
        , output(0.0f)
        , integral(0.0f)
        , maxIntegralRate(_maxIntegralRate)
        , maxIntegral(abs(_outMax - _outMin) * _maxIntegralRate) {}

    /* @brief Initializes sensor.
     * @returns Status indicating operation success.
     **/
    Status initialize();

    void setParams(const Params& _params, float32_t _maxIntegralRate);

    void setParams(const PID_Controller& other);

    /* @brief Gets output.
     * @returns The output.
     **/
    float32_t getOutput() const {
        return this->output;
    }

    /* @brief Updates output according to current and previous errors.
     * @note If an error occurs (see result status), output will not be updated!
     * @note This is an internal method, use MEASURE(meas) macro instead!
     * @param _measured The current measured value.
     * @returns Status indicating operation success.
     **/
    Status run(float32_t _measured);

    Params params;          // The controller parameters.
    float32_t outMin;
    float32_t outMax;
    float32_t desired;      // The desired value.

private:
    /* @brief Calculates error.
     * @returns The error.
     **/
    float32_t getError() const {
        return this->desired - this->measured;
    }

    float32_t measured;     // The measured value.
    float32_t output;       // The output - updated in every cycle, holds the output value until the next update.
    float32_t integral;     // The integral error.
    float32_t maxIntegralRate;
    float32_t maxIntegral;
};
} // namespace uns
