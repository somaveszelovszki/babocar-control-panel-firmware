#pragma once

#include <uns/util/units.hpp>
#include <uns/Line.hpp>

namespace uns {

class LineController {
public:
	LineController(meter_t _wheelBase, meter_t _sensorWheelDist);

	Status run(m_per_sec_t speed, meter_t baseline, const Line& line);

	radian_t getOutput() const {
	    return this->output;
	}
private:
	const meter_t wheelBase;          // Distance between the two axles.
	const meter_t projectedWheelBase; // The original wheel base + the sensor distance.
	radian_t output;
};
} // namespace uns
