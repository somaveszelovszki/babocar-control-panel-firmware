#pragma once

#include <uns/util/types.hpp>
#include <uns/util/debug.hpp>
#include <math.h>


namespace uns {


class LineController {

public:

	LineController(meter_t _wheelBase, meter_t _sensorWheelDist);


	radian_t GetControlSignal(m_per_sec_t currentVelocity, meter_t currentLinePosition, radian_t currentLineOrientation);


	void _Test();

private:

	static constexpr float32_t xi_coeff = 0.9f; // TODO 0.7

	void UpdateBaseline(distance_t newBaseLine);

	// From config.
	meter_t wheelBase;						// Distance between the two axles.
	meter_t sensorWheelDist;			// Distance between the first axle and front led row.
	meter_t projectedWheelBase ; 			// The original wheel base + the sensor distance.

	meter_t baseline;  					// The controller wants to keep the line at this point of the sensor.

};
} // namespace uns
