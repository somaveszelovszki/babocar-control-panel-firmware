#pragma once

#include <micro/utils/units.hpp>

namespace cfg {

constexpr micro::millimeter_t CAR_PIVOT_DIST_FRONT_REAR = micro::millimeter_t(273);      // Distance between front and rear pivots.
constexpr micro::millimeter_t CAR_PIVOT_DIST_MID        = CAR_PIVOT_DIST_FRONT_REAR / 2; // Distance between pivot and car middle.
constexpr micro::millimeter_t CAR_PIVOT_LENGTH_FRONT    = micro::millimeter_t(250);      // Length of front pivot (wheel to wheel).
constexpr micro::millimeter_t CAR_PIVOT_LENGTH_REAR     = micro::millimeter_t(250);      // Length of front pivot (wheel to wheel).
constexpr micro::millimeter_t CAR_PIVOT_FRONT_DIST      = micro::millimeter_t(72);       // Distance between car front and front pivot.
constexpr micro::millimeter_t CAR_PIVOT_REAR_DIST       = micro::millimeter_t(72);       // Distance between car rear and rear pivot.

constexpr micro::millimeter_t CAR_WHEEL_CIRC = micro::millimeter_t(329);    // Wheel circumference.

constexpr uint8_t NUM_OPTO_FRONT = 32;                                  // Number of front opto-sensors.
constexpr uint8_t NUM_OPTO_REAR  = 32;                                  // Number of rear opto-sensors.
constexpr uint8_t NUM_OPTO       = NUM_OPTO_FRONT + NUM_OPTO_REAR;      // Number of opto-sensors.

constexpr micro::millimeter_t DIST_BTW_OPTOS = micro::millimeter_t(7.742f); // Distance between the line detector optical sensors.

// Maximum position of the front optical sensor line.
// Leftmost sensor's position:  -MAX_POS_OPTO_FRONT
// Middle position:             0
// Rightmost sensor's position: MAX_POS_OPTO_FRONT
constexpr micro::millimeter_t MAX_POS_OPTO_FRONT = DIST_BTW_OPTOS * NUM_OPTO_FRONT / 2;

// Maximum position of the rear optical sensor line.
// Leftmost sensor's position:  -MAX_POS_OPTO_REAR
// Middle position:             0
// Rightmost sensor's position: MAX_POS_OPTO_REAR
constexpr micro::millimeter_t MAX_POS_OPTO_REAR = DIST_BTW_OPTOS * NUM_OPTO_REAR / 2;

constexpr micro::millimeter_t DIST_BTW_OPTO_ROWS = micro::centimeter_t(21.3f);   // Distance between the two line detector optical sensor rows.

constexpr micro::radian_t FRONT_SERVO_OFFSET          = micro::degree_t(103.0f); // Front servo middle position angle.
constexpr micro::radian_t FRONT_SERVO_WHEEL_MAX_DELTA = micro::degree_t(23.0f);  // Front wheel max delta angle.
constexpr float           FRONT_SERVO_WHEEL_TR        = 16.5f / 40.0f;           // Front servo-to-wheel transfer rate.

constexpr micro::radian_t REAR_SERVO_OFFSET           = micro::degree_t(98.0f);  // Rear servo middle position angle.
constexpr micro::radian_t REAR_SERVO_WHEEL_MAX_DELTA  = micro::degree_t(23.0f);  // Rear wheel max delta angle.
constexpr float           REAR_SERVO_WHEEL_TR         = 16.5f / 40.0f;           // Rear servo-to-wheel transfer rate.

constexpr micro::radian_t DIST_SERVO_OFFSET           = micro::degree_t(100.0f); // Front distance sensor servo servo middle position angle.
constexpr micro::radian_t DIST_SERVO_MAX_DELTA        = micro::degree_t(45.0f);  // Front distance sensor servo max delta angle.

constexpr micro::millimeter_t OPTO_SENSOR_FRONT_WHEEL_DIST = micro::millimeter_t(107.0f); // Distance between front sensors and front wheels.

constexpr uint32_t ROTARY_PERIOD_MS = 10;

constexpr micro::microsecond_t DC_MOTOR_T_ELECTRICAL = micro::microsecond_t(484.0f);

constexpr micro::microsecond_t DC_MOTOR_CONTROLLER_DEFAULT_Ti = DC_MOTOR_T_ELECTRICAL;
constexpr float                DC_MOTOR_CONTROLLER_DEFAULT_Kc = 1.0f;

constexpr float FRONT_LINE_CONTROLLER_DEFAULT_P_slow = 2.0f;
constexpr float FRONT_LINE_CONTROLLER_DEFAULT_D_slow = 1700.0f;

constexpr float FRONT_LINE_CONTROLLER_DEFAULT_P_fast = 0.1f;
constexpr float FRONT_LINE_CONTROLLER_DEFAULT_D_fast = 100.0f;

constexpr float REAR_LINE_CONTROLLER_DEFAULT_P  = 1.0f;
constexpr float REAR_LINE_CONTROLLER_DEFAULT_D  = 1.0f;

} // namespace cfg
