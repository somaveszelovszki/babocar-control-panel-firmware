

#include <uns/LineController.hpp>
#include <uns/util/debug.hpp>
#include <uns/util/unit_utils.hpp>


using namespace uns;



void LineController::_Test(){

    m_per_sec_t v = cm_per_sec_t(5);
    meter_t linePos = centimeter_t(10.6f);
    radian_t lineAngle = radian_t::ZERO();
    angle_t u = GetControlSignal(v, linePos, lineAngle);

}

LineController::LineController(meter_t _wheelBase, meter_t _sensorWheelDist)
    : wheelBase(_wheelBase)
    , sensorWheelDist(_sensorWheelDist)
    , projectedWheelBase(_sensorWheelDist + _wheelBase) {}

radian_t LineController::GetControlSignal(m_per_sec_t currentVelocity, meter_t currentLinePosition, radian_t currentLineOrientation) {

    const m_per_sec_t MIN_SPEED = mm_per_sec_t(5);
    angle_t result;
    if (uns::abs(currentVelocity) < MIN_SPEED) {
        result = radian_t::ZERO();
    } else {

        // The car should correct itself in this distance.
        millimeter_t adjustDistance(static_cast<mm_per_sec_t>(currentVelocity).get() * 0.5f + 500);

        // The car should correct itself in this time.
        float32_t adjustTime = adjustDistance.get() / static_cast<mm_per_sec_t>(currentVelocity).get();

        // The time constant that the system needs.
        float32_t T_const = (adjustTime * xi_coeff) / 3;
        float32_t w0 = 1.0f / T_const;

        // Update poles = (-)Re +- j*Im
        float32_t s_re = -w0 * xi_coeff;
        float32_t s_im = w0 * std::sqrt(1 - xi_coeff * xi_coeff);

        // Handle for variables.
        float32_t v = currentVelocity.get();
        float32_t L = this->projectedWheelBase.get();

        // Gains of the position and orientation.
        float32_t K_location = (-L / (v * v)) * (s_re * s_re + s_im * s_im);
        float32_t K_orientation = (L / v) * ((2 * s_re) - v * K_location);

        // Feedback components. Baseline, position, orientation.
        float32_t ub = K_location * this->baseline.get();
        float32_t up = K_location * currentLinePosition.get();
        float32_t ud = -K_orientation * currentLineOrientation.get();

        // The wheel turn at the sensor.
        radian_t servoInFi(ub - up - ud);

        // The wheel turn at the front axle. (Don't let servoInFi be over +-90degrees.)
        servoInFi = uns::clamp(servoInFi, -uns::PI_2, uns::PI_2);

        //result = atan(tan(servoInFi)*(this->wheelbase/(this->projectedWheelBase)));
        result = servoInFi * (this->wheelBase / this->projectedWheelBase);   // nearly equal to atan(tan(servoInFi)*(this->wheelbase/(this->projectedWheelBase)))
    }

    return result;
}




void LineController::UpdateBaseline(distance_t newBaseLine){
    this->baseline = newBaseLine;
}
