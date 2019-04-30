

#include <uns/LineController.hpp>
#include <uns/util/debug.hpp>
#include <uns/util/unit_utils.hpp>


using namespace uns;



void LineController::_Test(){

	speed_t v = speed_t::from<mm_per_sec>(50);
    distance_t linePos = distance_t::from<millimeters>(106);
    angle_t lineAngle = angle_t::from<radians>(-0.0);
	angle_t u = GetControlSignal(v, linePos, lineAngle);

}

LineController::LineController(distance_t _wheelBase, distance_t _sensorWheelDist)
    : wheelBase(_wheelBase)
    , sensorWheelDist(_sensorWheelDist)
    , projectedWheelBase(_sensorWheelDist + _wheelBase) {}

angle_t LineController::GetControlSignal(speed_t currentVelocity, distance_t currentLinePosition, angle_t currentLineOrientation){

    const speed_t MIN_SPEED(mm_per_sec(), 5);
    angle_t result;
    if (abs(currentVelocity) < MIN_SPEED) {
        result = angle_t::from<radians>(0.0f);
    } else {

        // The car should correct itself in this distance.
        distance_t adjustDistance = distance_t::from<millimeters>(((currentVelocity.get<mm_per_sec>())*0.5f + 500));

        // The car should correct itself in this time.
        float32_t adjustTime = adjustDistance.get<millimeters>()/(float32_t)currentVelocity.get<mm_per_sec>();

        // The time constant that the system needs.
        float32_t T_const = (adjustTime*xi_coeff)/3;
        float32_t w0 = 1.0f / T_const;

        // Update poles = (-)Re +- j*Im
        float32_t s_re = -w0*xi_coeff;
        float32_t s_im = w0*std::sqrt(1-xi_coeff*xi_coeff);

        // Handle for variables.
        float32_t v = currentVelocity.get<mm_per_sec>() / 1000.0f;
        float32_t L = this->projectedWheelBase.get<millimeters>() / 1000.0f;

        // Gains of the position and orientation.
        float32_t K_location = (-L / (v*v)) * (s_re*s_re + s_im*s_im);
        float32_t K_orientation = (L / v) * ((2*s_re) - v*K_location);

        // Feedback components. Baseline, position, orientation.
        float32_t ub = K_location*this->baseline.get<millimeters>() / 1000.0f;
        float32_t up = K_location*currentLinePosition.get<millimeters>() / 1000.0f;
        float32_t ud = -K_orientation*currentLineOrientation.get<radians>();

        // The wheel turn at the sensor.
        angle_t servoInFi = angle_t::from<radians>(ub - up - ud);

        // The wheel turn at the front axle. (Don't let servoInFi be over +-90degrees.)
        servoInFi = uns::incarcerate(servoInFi, -uns::PI_2, uns::PI_2);

        //result = atan(tan(servoInFi)*(this->wheelbase/(this->projectedWheelBase)));
        result = servoInFi * (this->wheelBase / this->projectedWheelBase);   // nearly equal to atan(tan(servoInFi)*(this->wheelbase/(this->projectedWheelBase)))
    }

	return result;
}




void LineController::UpdateBaseline(distance_t newBaseLine){
	this->baseline = newBaseLine;
}










