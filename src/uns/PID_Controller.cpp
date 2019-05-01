#include <uns/PID_Controller.hpp>
#include <uns/util/numeric.hpp>

using namespace uns;

Status PID_Controller::initialize() {
    this->desired = this->measured = this->output = this->integral = 0.0f;
    return Status::OK;
}

void PID_Controller::setParams(const Params& _params, float32_t _maxIntegralRate) {
    this->params = _params;
    this->maxIntegralRate = _maxIntegralRate;
    this->maxIntegral = abs(this->outMax - this->outMin) * _maxIntegralRate;
}

void PID_Controller::setParams(const PID_Controller& other) {
    this->setParams(other.params, other.maxIntegralRate);
}

Status PID_Controller::run(float32_t _measured) {
    millisecond_t d_time = this->updateTimeDiff();

    float32_t prevErr = this->getError();
    this->measured = _measured;
    float32_t err = this->getError();

    Status status;
    int32_t d_time_ms = d_time.get();
    if (d_time_ms > 0) {
        float32_t derivative = (err - prevErr) / d_time_ms;
        this->integral += err * d_time_ms;
        this->integral = uns::clamp(this->integral, -this->maxIntegral, this->maxIntegral);
        this->output = err * this->params.Kp + this->integral * this->params.Ki + derivative * this->params.Kd;
        this->output = uns::clamp(this->output, this->outMin, this->outMax);
        status = Status::OK;
    } else {
        status = Status::ERROR;
    }

    return status;
}
