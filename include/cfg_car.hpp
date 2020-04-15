#pragma once

#include <micro/utils/units.hpp>
#include <micro/math/unit_utils.hpp>

namespace cfg {

constexpr micro::meter_t  CAR_FRONT_REAR_PIVOT_DIST      = micro::millimeter_t(0);      // TODO Distance between front and rear pivots.
constexpr micro::meter_t  CAR_PIVOT_LENGTH               = micro::millimeter_t(0);      // TODO Length of front pivot (wheel to wheel).
constexpr micro::meter_t  CAR_FRONT_REAR_SENSOR_ROW_DIST = micro::millimeter_t(0);      // TODO Distance between front and rear sensor rows.

constexpr float           SERVO_WHEEL_TRANSFER_RATE    = 1.0f; // TODO Servo-to-wheel transfer rate.
constexpr micro::radian_t FRONT_SERVO_WHEEL_MAX_DELTA  = micro::degree_t(25.0f);           // Front wheel max delta angle.
constexpr micro::radian_t REAR_SERVO_WHEEL_MAX_DELTA   = micro::degree_t(25.0f);           // Rear wheel max delta angle.
constexpr micro::radian_t DIST_SERVO_OFFSET            = micro::degree_t(90.0f);           // Front distance sensor servo servo middle position angle.
constexpr micro::radian_t DIST_SERVO_MAX_DELTA         = micro::degree_t(45.0f);           // Front distance sensor servo max delta angle.

constexpr micro::meter_t  OPTO_SENSOR_FRONT_WHEEL_DIST = micro::millimeter_t(107.0f);     // Distance between front sensors and front wheels.

#define SERIAL_DEBUG_ENABLED true

} // namespace cfg
