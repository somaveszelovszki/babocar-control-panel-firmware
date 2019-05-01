#pragma once

#include <uns/hw/HC_SR04_Ultrasonic.hpp>
#include <uns/sensor/Sensor.hpp>
#include <uns/sensor/Filter.hpp>

namespace uns {

constexpr float32_t COMPLIANCE_RATE = 0.3f;         // Compliance for new measurement - relative to previous measurement.
constexpr meter_t DEADBAND = centimeter_t(10.0f);   // Deadband for new measurement.

class DistanceSensor : public Sensor<distance_t> {
public:
    DistanceSensor(millisecond_t timeout, const gpio_pin_struct& _trigger, const gpio_pin_struct& _echo)
        : Sensor(timeout)
        , sonar(_trigger, _echo)
        , filter(COMPLIANCE_RATE, DEADBAND) {}

    /* @brief Initializes sensor.
     * @returns Status indicating operation success.
     **/
    Status initialize();

    /* @brief Measures, stores and filters distance.
     * @returns Status indicating operation success.
     **/
    Status run();

    /* @brief Called when ECHO signal has been received. Calculates distance by the difference of the TRIGGER and the ECHO signals.
     **/
    void onEchoReceived() {
        this->sonar.onEchoReceived();
    }

private:
    hw::HC_SR04_Ultrasonic sonar;   // The ultrasonic distance sensor.
    BounceFilter<distance_t, 2> filter; // Filter for the measured distances. Drops positive and negative peaks.
};

} // namespace uns
