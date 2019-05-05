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
    this->meas = CarProps();
    return Status::OK;
}

Status CarPropsSensor::run(angle_t d_angle) {
    static constexpr deg_per_sec_t deadBand = deg_per_sec_t(2.0f);

    millisecond_t d_time = this->updateTimeDiff();
    Status status = Status::OK;

    const meter_t d_dist = this->meas.speed * d_time;
    this->meas.pose.angle += d_angle / 2;
    this->meas.pose.pos.X += d_dist * uns::cos(this->meas.pose.angle);
    this->meas.pose.pos.Y += d_dist * uns::sin(this->meas.pose.angle);
    this->meas.pose.angle = uns::normalize360(this->meas.pose.angle + d_angle / 2);

    return status;
}
