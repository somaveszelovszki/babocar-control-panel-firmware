#pragma once

#include <micro/utils/units.hpp>
#include <micro/math/unit_utils.hpp>

namespace cfg {

constexpr micro::meter_t CAR_FRONT_REAR_PIVOT_DIST    = micro::millimeter_t(268);      // Distance between front and rear pivots.
constexpr micro::meter_t CAR_PIVOT_CENTER_DIST        = CAR_FRONT_REAR_PIVOT_DIST / 2; // Distance between pivots and car center.
constexpr micro::meter_t CAR_FRONT_PIVOT_LENGTH       = micro::millimeter_t(257);      // Length of front pivot (wheel to wheel).
constexpr micro::meter_t CAR_REAR_PIVOT_LENGTH        = micro::millimeter_t(257);      // Length of rear pivot (wheel to wheel).

constexpr micro::meter_t CAR_OPTO_FRONT_PIVOT_DIST    = micro::centimeter_t(11.0f);    // Distance between optical sensor row and front pivot.
constexpr micro::meter_t CAR_OPTO_CENTER_DIST         = CAR_PIVOT_CENTER_DIST + CAR_OPTO_FRONT_PIVOT_DIST; // Distance between optical sensor row and car center.

constexpr uint32_t        FRONT_SERVO_PWM_0           = 550;                              // Front servo PWM for 0 degrees.
constexpr uint32_t        FRONT_SERVO_PWM_180         = 2600;                             // Front servo PWM for 180 degrees.
constexpr micro::radian_t FRONT_SERVO_OFFSET          = micro::degree_t(90.0f);           // Front servo middle position angle.
constexpr micro::radian_t FRONT_SERVO_WHEEL_MAX_DELTA = micro::degree_t(25.0f);           // Front wheel max delta angle.
constexpr float           FRONT_SERVO_WHEEL_TR        = micro::avg(30.0f, 19.0f) / 40.0f; // Front servo-to-wheel transfer rate.

constexpr uint32_t        REAR_SERVO_PWM_0            = 580;                              // Rear servo PWM for 0 degrees.
constexpr uint32_t        REAR_SERVO_PWM_180          = 2565;                             // Rear servo PWM for 180 degrees.
constexpr micro::radian_t REAR_SERVO_OFFSET           = micro::degree_t(90.0f);           // Rear servo middle position angle.
constexpr micro::radian_t REAR_SERVO_WHEEL_MAX_DELTA  = micro::degree_t(25.0f);           // Rear wheel max delta angle.
constexpr float           REAR_SERVO_WHEEL_TR         = micro::avg(30.0f, 19.0f) / 40.0f; // Rear servo-to-wheel transfer rate.

constexpr uint32_t        DIST_SERVO_PWM_0            = 680;                              // Front distance sensor servo PWM for 0 degrees.
constexpr uint32_t        DIST_SERVO_PWM_180          = 2500;                             // Front distance sensor servo PWM for 180 degrees.
constexpr micro::radian_t DIST_SERVO_OFFSET           = micro::degree_t(90.0f);           // Front distance sensor servo servo middle position angle.
constexpr micro::radian_t DIST_SERVO_MAX_DELTA        = micro::degree_t(45.0f);           // Front distance sensor servo max delta angle.

constexpr micro::meter_t OPTO_SENSOR_FRONT_WHEEL_DIST = micro::millimeter_t(107.0f);     // Distance between front sensors and front wheels.

#define SERIAL_DEBUG_ENABLED true

} // namespace cfg
