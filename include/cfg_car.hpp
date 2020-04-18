#pragma once

#include <micro/utils/units.hpp>
#include <micro/math/unit_utils.hpp>

namespace cfg {

constexpr micro::meter_t  CAR_FRONT_REAR_PIVOT_DIST      = micro::millimeter_t(0);
constexpr micro::meter_t  CAR_PIVOT_LENGTH               = micro::millimeter_t(0);
constexpr micro::meter_t  CAR_FRONT_REAR_SENSOR_ROW_DIST = micro::millimeter_t(0);
constexpr micro::meter_t  OPTO_SENSOR_FRONT_WHEEL_DIST   = micro::millimeter_t(107.0f);

constexpr micro::radian_t FRONT_WHEEL_MAX_DELTA          = micro::degree_t(20);
constexpr micro::radian_t REAR_WHEEL_MAX_DELTA           = micro::degree_t(20);
constexpr micro::radian_t DIST_SENSOR_SERVO_MAX_DELTA    = micro::degree_t(45);

#define SERIAL_DEBUG_ENABLED true

} // namespace cfg
