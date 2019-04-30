#include <uns/config/cfg_car.hpp>
#include <uns/sensor/CarPropsSensor.hpp>
#include <uns/util/debug.hpp>
#include <uns/util/unit_utils.hpp>

using namespace uns;

const speed_t& SpeedFilter::update(const speed_t& measuredValue) {
//    const speed_t prevFiltered = this->filteredValue;
//    const speed_t expected = this->filteredValue + this->prevDiff;
//
//    this->filteredValue = uns::avg(expected, measuredValue);
//    this->prevDiff = this->filteredValue - prevFiltered;
    this->filteredValue = measuredValue;
    return this->filteredValue;
}

Status CarPropsSensor::initialize() {
    this->meas.orientation_ = uns::PI_2;
    return Status::OK;
}

Status CarPropsSensor::run(angle_t d_angle) {
    static constexpr angular_velocity_t deadBand(deg_per_sec(), 2.0f);

    uns::time_t d_time = this->updateTimeDiff();
    Status status = Status::OK;

    const distance_t d_dist = this->meas.speed() * d_time;
    this->meas.orientation_ += d_angle / 2;
    this->meas.pos_.X += d_dist * uns::cos(this->meas.orientation_);
    this->meas.pos_.Y += d_dist * uns::sin(this->meas.orientation_);
    this->meas.orientation_ += d_angle / 2;
    this->meas.orientation_ = uns::normalize360(this->meas.orientation_);

    return status;
}
