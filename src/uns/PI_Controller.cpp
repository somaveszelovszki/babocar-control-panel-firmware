#include <uns/PI_Controller.hpp>
#include <uns/util/unit_utils.hpp>

using namespace uns;

Status PI_Controller::initialize() {
    this->desired = m_per_sec_t::ZERO();
    this->output = this->prevError = 0.0f;
    return Status::OK;
}

void PI_Controller::setDesired(m_per_sec_t desired_) {
    static constexpr m_per_sec_t DEAD_BAND(0.2f);
    this->desired = uns::abs(desired_) > DEAD_BAND ? desired_ : m_per_sec_t::ZERO();
}

Status PI_Controller::run(m_per_sec_t measured) {
    this->updateTimeDiff();

    float32_t error = (this->desired - measured).get();

    this->output = this->output + b0 * error + b1 * this->prevError;
    this->output = uns::clamp(this->output, this->outMin, this->outMax);
    this->prevError = error;

    return Status::OK;
}
