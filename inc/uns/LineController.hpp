#pragma once

#include <uns/util/types.hpp>
#include <uns/util/debug.hpp>
#include <math.h>


namespace uns {


class LineController {

public:

	LineController(distance_t _wheelBase, distance_t _sensorWheelDist);


	angle_t GetControlSignal(speed_t currentVelocity, distance_t followedLinePosition, angle_t currentLineOrientation);


	void _Test();

private:

	static constexpr float32_t xi_coeff = 0.9f; // TODO 0.7

	void UpdateBaseline(distance_t newBaseLine);

	// From config.
	distance_t wheelBase;						// Distance between the two axles.
	distance_t sensorWheelDist;			// Distance between the first axle and front led row.
	distance_t projectedWheelBase ; 			// The original wheel base + the sensor distance.

	distance_t baseline;  					// The controller wants to keep the line at this point of the sensor.

};
} // namespace uns
