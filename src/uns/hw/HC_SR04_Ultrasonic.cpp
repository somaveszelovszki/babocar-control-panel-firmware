#include <uns/hw/HC_SR04_Ultrasonic.hpp>
#include <uns/bsp/tim.hpp>
#include <uns/bsp/gpio.hpp>
#include <uns/util/numeric.hpp>
#include <uns/config/cfg_board.hpp>
#include <uns/util/debug.hpp>

using namespace uns;

namespace {
static constexpr uint32_t US_ROUNDTRIP_CM = 57; // Microseconds (uS) it takes sound to travel round-trip 1cm (2cm total), uses integer to save compiled code space.

static constexpr distance_t MAX_DIST(millimeters(), 200);

distance_t echoTimeToDist(uns::time_t echoTime) {
    const uint32_t echoTime_us = static_cast<uint32_t>(echoTime.get<microseconds>());
    return distance_t::from<centimeters>(max(echoTime_us / US_ROUNDTRIP_CM, (echoTime_us ? static_cast<uint32_t>(1) : 0)));
}

uns::time_t distToEchoTime(const distance_t& dist) {
    return uns::time_t::from<microseconds>(dist.get<centimeters>() * US_ROUNDTRIP_CM);
}

static constexpr time_t ECHO_OVERHEAD = time_t(microseconds(), 450);    // Overhead before echo.

} // namespace

hw::HC_SR04_Ultrasonic::HC_SR04_Ultrasonic(const gpio_pin_struct& _trigger, const gpio_pin_struct& _echo)
    : trigger(_trigger)
    , echo(_echo)
    , busy(false) {}

void hw::HC_SR04_Ultrasonic::initialize() {
    GPIO_WritePin(this->trigger, PinState::RESET);
    this->busy = false;
}

void hw::HC_SR04_Ultrasonic::startMeasurement() {
    // if no echo signal has been received, sets measured value to zero and clears busy flag
//    if (this->busy && (uns::getTimerCounter(cfg::tim_System) - this->lastStartTime > distToEchoTime(MAX_DIST))) {
    if (this->busy && (uns::getExactTime() - this->lastStartTime > distToEchoTime(MAX_DIST))) {
        this->distance = MAX_DIST;
        this->busy = false;
    }

    // if sensor is not busy, starts next measurement by sending out the trigger impulse
    if (!this->busy) {
        this->lastStartTime = uns::getExactTime();
        //this->lastStartTime_us = uns::getTime().get<microseconds>() + uns::getTimerCounter(cfg::tim_System);
        GPIO_WritePin(this->trigger, PinState::RESET);
        while(uns::getExactTime() - this->lastStartTime < time_t::from<microseconds>(4)) {}
        GPIO_WritePin(this->trigger, PinState::SET);
        while(uns::getExactTime() - this->lastStartTime < time_t::from<microseconds>(14)) {}
        GPIO_WritePin(this->trigger, PinState::RESET);
        this->busy = true;
    }

    while(GPIO_ReadPin(this->echo) != PinState::SET) {
        if (uns::getExactTime() - this->lastStartTime > time_t::from<milliseconds>(500)){
            break;
        }
    }
    const time_t echoTime = uns::getExactTime();
    this->distance = echoTime > this->lastStartTime ? echoTimeToDist(echoTime - this->lastStartTime) : distance_t::from<millimeters>(0);
    this->busy = false;
}

void hw::HC_SR04_Ultrasonic::onEchoReceived() {
    const time_t echoTime = uns::getExactTime();
    this->distance = echoTime > this->lastStartTime - ECHO_OVERHEAD ? echoTimeToDist(echoTime - this->lastStartTime - ECHO_OVERHEAD) : MAX_DIST;
    this->busy = false;
}
