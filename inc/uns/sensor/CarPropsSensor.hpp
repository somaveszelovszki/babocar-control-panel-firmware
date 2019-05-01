#pragma once

#include <uns/sensor/Sensor.hpp>
#include <uns/sensor/Filter.hpp>
#include <uns/CarProps.hpp>

namespace uns {

constexpr uint32_t INCREMENT_PER_WHEEL_ROT = 46344; // Increments per wheel rotation - measured value.
constexpr float32_t SPEED_COMPLIANCE_RATE = 1.1f;
constexpr m_per_sec_t SPEED_DEAD_BAND = m_per_sec_t(1.0f);

constexpr float32_t SPEED_WEIGHT_NEW = 2.0f;

class SpeedFilter : FilterBase<speed_t> {
public:
    const speed_t& update(const speed_t& measuredValue);

private:
    speed_t prevDiff;
};

class CarPropsSensor : public Sensor<CarProps> {
public:
    /* @brief Constructor - sets timer handle and counter resolution, initializes previous value.
     * @param _htim The handle for the timer used in encoder mode.
     **/
    CarPropsSensor()
        : Sensor(millisecond_t(0.0f))
        , speed_updated(false) {}

    /* @brief Initializes sensor - measures encoder offset and calibrates gyroscope.
     * @returns Status indicating operation success.
     **/
    Status initialize();

    /* @brief Measures and stores speed.
     * @returns Status indicating operation success.
     **/
    Status run(angle_t d_angle);

    void updateMeas(const CarProps& car) {
        this->meas = car;
    }

private:

    SpeedFilter speedFilter;  // Filter for the speed. Drops fast changes.

    speed_t newSpeed;
    bool speed_updated;
};

} // namespace uns
