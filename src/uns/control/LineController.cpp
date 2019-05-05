#include <uns/control/LineController.hpp>
#include <uns/util/debug.hpp>
#include <uns/util/unit_utils.hpp>

namespace uns {

LineController::LineController(meter_t _wheelBase, meter_t _sensorWheelDist)
    : wheelBase(_wheelBase)
    , projectedWheelBase(_sensorWheelDist + _wheelBase) {}

Status LineController::run(m_per_sec_t speed, meter_t baseline, const Line& line) {

    static constexpr float32_t KSI_COEFF = SQRT_2;
    static constexpr m_per_sec_t MIN_SPEED = mm_per_sec_t(5);

    if (uns::abs(speed) < MIN_SPEED) {
        this->output = radian_t::ZERO();
    } else {

        // The car should correct itself in this distance.
//        millimeter_t adjustDistance(static_cast<mm_per_sec_t>(speed).get() * 0.5f + 500);

        const millimeter_t adjustDist = centimeter_t(50) + speed * second_t(0.5f);
        const second_t adjustTime = adjustDist / speed;
//        float32_t adjustTime = adjustDistance.get() / static_cast<mm_per_sec_t>(speed).get();

        // The time constant that the system needs.
        float32_t T_const = (adjustTime.get() * KSI_COEFF) / 3;
        float32_t w0 = 1.0f / T_const;

        // Update poles = (-)Re +- j*Im
        float32_t s_re = -w0 * KSI_COEFF;
        float32_t s_im = w0 * std::sqrt(1 - KSI_COEFF * KSI_COEFF);

        // Handle for variables.
        float32_t v = speed.get();
        float32_t L = this->projectedWheelBase.get();

        // Gains of the position and orientation.
        float32_t K_location = (-L / (v * v)) * (s_re * s_re + s_im * s_im);
        float32_t K_orientation = (L / v) * ((2 * s_re) - v * K_location);

        // Feedback components. Baseline, position, orientation.
        float32_t ub = K_location * baseline.get();
        float32_t up = K_location * static_cast<meter_t>(line.pos_front).get();
        float32_t ud = -K_orientation * line.angle.get();

        // The wheel turn at the sensor
        const radian_t servoInFi = uns::clamp(radian_t(ub - up - ud), -uns::PI_2, uns::PI_2);

        //result = atan(tan(servoInFi)*(this->wheelbase/(this->projectedWheelBase)));
        this->output = servoInFi * (this->wheelBase / this->projectedWheelBase);   // nearly equal to atan(tan(servoInFi)*(this->wheelbase/(this->projectedWheelBase)))
    }

    return Status::OK;
}
} // namespace uns
