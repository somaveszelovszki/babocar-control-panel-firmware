#pragma once

#include <micro/utils/units.hpp>

namespace cfg {

constexpr micro::meter_t CAR_FRONT_REAR_PIVOT_DIST      = micro::millimeter_t(269.5f);
constexpr micro::meter_t CAR_PIVOT_LENGTH               = micro::millimeter_t(263);
constexpr micro::meter_t CAR_FRONT_REAR_SENSOR_ROW_DIST = micro::millimeter_t(447);
constexpr micro::meter_t OPTO_SENSOR_FRONT_WHEEL_DIST   = micro::millimeter_t(107.0f);

constexpr micro::radian_t WHEEL_MAX_DELTA             = micro::degree_t(26);
constexpr micro::radian_t DIST_SENSOR_SERVO_MAX_DELTA = micro::degree_t(45);
constexpr float DIST_SENSOR_SERVO_TRANSFER_RATE       = 1.0f;
constexpr bool DIST_SENSOR_SERVO_ENABLED              = false;
constexpr micro::meter_t MIN_TURN_RADIUS              = micro::centimeter_t(40);

} // namespace cfg
