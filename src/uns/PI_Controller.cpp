#include <uns/PI_Controller.hpp>
#include <uns/util/numeric.hpp>

using namespace uns;

Status PI_Controller::initialize() {
    this->desired = speed_t::ZERO();
    this->output = this->prevError = 0.0f;
    return Status::OK;
}

void PI_Controller::setDesired(speed_t desired_) {
    static constexpr speed_t DEAD_BAND(m_per_sec(), 0.2f);
    this->desired = abs(desired_) > DEAD_BAND ? desired_ : speed_t::ZERO();
}

Status PI_Controller::run(speed_t measured) {
    this->updateTimeDiff();

    float32_t error = (this->desired - measured).get<m_per_sec>();

    this->output = this->output + b0 * error + b1 * this->prevError;
    this->output = uns::incarcerate(this->output, this->outMin, this->outMax);
    this->prevError = error;

    return Status::OK;
}
