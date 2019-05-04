#pragma once

#include <uns/base/Runnable.hpp>
#include <uns/util/unit_utils.hpp>

namespace uns {
/* @brief PID controller implementation.
 **/
template <typename T_meas, typename T_out>
class PID_Controller : public Runnable {
public:
    /* @brief Constructor - sets period time and term weights.
     **/
    PID_Controller(millisecond_t Ts, millisecond_t Ti, millisecond_t Td, float32_t Kc, const T_out& outMin, const T_out& outMax)
        : Runnable(period)
        , desired(0.0f)
        , b0(Kc * (1 + period / Ti + Td / period))
        , b1(-Kc * (1 + 2 * Td / Ts))
        , b2(Kc * Td / Ts)
        , ek1(0.0f)
        , ek2(0.0f)
        , outMin(outMin)
        , outMax(outMax)
        , output(0.0f) {}

    void setParams(const PID_Controller& other) {
        this->b0 = other.b0;
        this->b1 = other.b1;
        this->b2 = other.b2;
    }

    /* @brief Gets output.
     * @returns The output.
     **/
    T_out getOutput() const {
        return this->output;
    }

    /* @brief Updates output according to current and previous errors.
     * @note If an error occurs (see result status), output will not be updated!
     * @note This is an internal method, use MEASURE(meas) macro instead!
     * @param measured The current measured speed.
     * @returns Status indicating operation success.
     **/
    Status run(const T_meas& measured) {
        this->updateTimeDiff();

        float32_t ek = uns::get_value(this->desired - measured);

        this->output = this->output + this->b0 * ek + this->b1 * this->ek1 + this->b2 * this->ek2;
        this->output = uns::clamp(this->output, this->outMin, this->outMax);
        this->ek2 = this->ek1;
        this->ek1 = ek;

        return Status::OK;
    }

    T_meas desired;     // The desired value.

private:
    float32_t b0, b1, b2;
    float32_t ek1, ek2;
    float32_t outMin;
    float32_t outMax;
    T_out output;       // The output - updated in every cycle, holds the output value until the next update.
};
} // namespace uns
