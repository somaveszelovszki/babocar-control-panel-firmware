#include <uns/sensor/DistanceSensor.hpp>

using namespace uns;

Status DistanceSensor::initialize() {
    this->sonar.initialize();
    return Status::OK;
}

Status DistanceSensor::run() {
    this->meas = this->filter.update(this->sonar.getDistance());
    this->sonar.startMeasurement();
    return Status::OK;
}
