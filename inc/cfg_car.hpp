#pragma once

#include <micro/utils/units.hpp>

namespace cfg {

constexpr micro::meter_t CAR_FRONT_REAR_PIVOT_DIST      = micro::millimeter_t(268);      // Distance between front and rear pivots.
constexpr micro::meter_t CAR_PIVOT_CENTER_DIST          = CAR_FRONT_REAR_PIVOT_DIST / 2; // Distance between pivots and car center.
constexpr micro::meter_t CAR_FRONT_PIVOT_LENGTH         = micro::millimeter_t(257);      // Length of front pivot (wheel to wheel).
constexpr micro::meter_t CAR_REAR_PIVOT_LENGTH          = micro::millimeter_t(257);      // Length of rear pivot (wheel to wheel).

constexpr micro::meter_t CAR_OPTO_FRONT_PIVOT_DIST      = micro::centimeter_t(11.0f);    // Distance between optical sensor row and front pivot.
constexpr micro::meter_t CAR_OPTO_CENTER_DIST           = CAR_PIVOT_CENTER_DIST + CAR_OPTO_FRONT_PIVOT_DIST; // Distance between optical sensor row and car center.

constexpr uint32_t        FRONT_SERVO_PWM_0             = 1000;                          // Front servo PWM for 0 degrees.
constexpr uint32_t        FRONT_SERVO_PWM_180           = 2000;                          // Front servo PWM for 180 degrees.
constexpr micro::radian_t FRONT_SERVO_OFFSET            = micro::degree_t(103.0f);       // Front servo middle position angle.
constexpr micro::radian_t FRONT_SERVO_WHEEL_MAX_DELTA   = micro::degree_t(23.0f);        // Front wheel max delta angle.
constexpr float           FRONT_SERVO_WHEEL_TR          = 16.5f / 40.0f;                 // Front servo-to-wheel transfer rate.

constexpr uint32_t        REAR_SERVO_PWM_0              = 1000;                          // Rear servo PWM for 0 degrees.
constexpr uint32_t        REAR_SERVO_PWM_180            = 2000;                          // Rear servo PWM for 180 degrees.
constexpr micro::radian_t REAR_SERVO_OFFSET             = micro::degree_t(95.0f);        // Rear servo middle position angle.
constexpr micro::radian_t REAR_SERVO_WHEEL_MAX_DELTA    = micro::degree_t(23.0f);        // Rear wheel max delta angle.
constexpr float           REAR_SERVO_WHEEL_TR           = 16.5f / 40.0f;                 // Rear servo-to-wheel transfer rate.

constexpr uint32_t        DIST_SERVO_PWM_0              = 1000;                          // Front distance sensor servo PWM for 0 degrees.
constexpr uint32_t        DIST_SERVO_PWM_180            = 2000;                          // Front distance sensor servo PWM for 180 degrees.
constexpr micro::radian_t DIST_SERVO_OFFSET             = micro::degree_t(100.0f);       // Front distance sensor servo servo middle position angle.
constexpr micro::radian_t DIST_SERVO_MAX_DELTA          = micro::degree_t(45.0f);        // Front distance sensor servo max delta angle.

constexpr micro::meter_t OPTO_SENSOR_FRONT_WHEEL_DIST = micro::millimeter_t(107.0f);     // Distance between front sensors and front wheels.

constexpr micro::microsecond_t DC_MOTOR_T_ELECTRICAL = micro::microsecond_t(484.0f);

constexpr micro::microsecond_t DC_MOTOR_CONTROLLER_DEFAULT_Ti = DC_MOTOR_T_ELECTRICAL;
constexpr float                DC_MOTOR_CONTROLLER_DEFAULT_Kc = 1.0f;

constexpr float FRONT_LINE_CONTROLLER_DEFAULT_P_slow = 1.8f;
constexpr float FRONT_LINE_CONTROLLER_DEFAULT_D_slow = 1700.0f;

constexpr float FRONT_LINE_CONTROLLER_DEFAULT_P_fast = 0.15f;
constexpr float FRONT_LINE_CONTROLLER_DEFAULT_D_fast = 50.0f;

constexpr float REAR_LINE_CONTROLLER_DEFAULT_P  = 1.0f;
constexpr float REAR_LINE_CONTROLLER_DEFAULT_D  = 1.0f;

} // namespace cfg
