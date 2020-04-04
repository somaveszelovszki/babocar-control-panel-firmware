#pragma once

#include <micro/utils/units.hpp>
#include <micro/math/unit_utils.hpp>

namespace cfg {

constexpr micro::meter_t  CAR_FRONT_REAR_PIVOT_DIST    = micro::millimeter_t(268);      // Distance between front and rear pivots.
constexpr micro::meter_t  CAR_PIVOT_CENTER_DIST        = CAR_FRONT_REAR_PIVOT_DIST / 2; // Distance between pivots and car center.
constexpr micro::meter_t  CAR_FRONT_PIVOT_LENGTH       = micro::millimeter_t(257);      // Length of front pivot (wheel to wheel).
constexpr micro::meter_t  CAR_REAR_PIVOT_LENGTH        = micro::millimeter_t(257);      // Length of rear pivot (wheel to wheel).

constexpr micro::meter_t  CAR_OPTO_FRONT_PIVOT_DIST    = micro::centimeter_t(11.0f);                            // Distance between optical sensor row and front pivot.
constexpr micro::meter_t  CAR_OPTO_CENTER_DIST         = CAR_PIVOT_CENTER_DIST + CAR_OPTO_FRONT_PIVOT_DIST;     // Distance between optical sensor row and car center.
constexpr micro::meter_t  CAR_OPTO_REAR_PIVOT_DIST     = CAR_FRONT_REAR_PIVOT_DIST + CAR_OPTO_FRONT_PIVOT_DIST; // Distance between optical sensor row and rear pivot.

constexpr float           SERVO_WHEEL_TRANSFER_RATE    = 1.0f; // TODO Servo-to-wheel transfer rate.
constexpr micro::radian_t FRONT_SERVO_WHEEL_MAX_DELTA  = micro::degree_t(25.0f);           // Front wheel max delta angle.
constexpr micro::radian_t REAR_SERVO_WHEEL_MAX_DELTA   = micro::degree_t(25.0f);           // Rear wheel max delta angle.
constexpr micro::radian_t DIST_SERVO_OFFSET            = micro::degree_t(90.0f);           // Front distance sensor servo servo middle position angle.
constexpr micro::radian_t DIST_SERVO_MAX_DELTA         = micro::degree_t(45.0f);           // Front distance sensor servo max delta angle.

constexpr micro::meter_t  OPTO_SENSOR_FRONT_WHEEL_DIST = micro::millimeter_t(107.0f);     // Distance between front sensors and front wheels.

#define SERIAL_DEBUG_ENABLED true

} // namespace cfg
