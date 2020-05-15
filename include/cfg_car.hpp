#pragma once

#include <micro/utils/units.hpp>

namespace cfg {

constexpr micro::meter_t  CAR_FRONT_REAR_PIVOT_DIST       = micro::millimeter_t(0);
constexpr micro::meter_t  CAR_PIVOT_LENGTH                = micro::millimeter_t(0);
constexpr micro::meter_t  CAR_FRONT_REAR_SENSOR_ROW_DIST  = micro::millimeter_t(0);
constexpr micro::meter_t  OPTO_SENSOR_FRONT_WHEEL_DIST    = micro::millimeter_t(107.0f);

constexpr micro::radian_t WHEEL_MAX_DELTA                 = micro::degree_t(20);
constexpr micro::radian_t DIST_SENSOR_SERVO_MAX_DELTA     = micro::degree_t(45);
constexpr float           DIST_SENSOR_SERVO_TRANSFER_RATE = 1.0f;
constexpr bool            DIST_SENSOR_SERVO_ENABLED       = false;

constexpr bool            USE_SAFETY_ENABLE_SIGNAL        = true;
constexpr bool            INDICATOR_LEDS_ENABLED          = true;
constexpr uint8_t         REDUCED_LINE_DETECT_SCAN_RADIUS = 10;

} // namespace cfg
