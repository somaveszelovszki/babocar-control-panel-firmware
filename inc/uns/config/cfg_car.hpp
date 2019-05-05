#pragma once

#include <uns/util/units.hpp>

namespace uns {
namespace cfg {

constexpr millimeter_t CAR_PIVOT_DIST_FRONT_REAR = millimeter_t(200);   // Distance between front and rear pivots.
constexpr distance_t CAR_PIVOT_DIST_MID(CAR_PIVOT_DIST_FRONT_REAR / 2); // Distance between pivot and car middle.
constexpr millimeter_t CAR_PIVOT_LENGTH = millimeter_t(80);             // Length of front and rear pivots (middle to wheel).
constexpr millimeter_t CAR_PIVOT_FRONT_DIST = millimeter_t(72);         // Distance between car front and front pivot.
constexpr millimeter_t CAR_PIVOT_REAR_DIST = millimeter_t(72);          // Distance between car rear and rear pivot.
constexpr millimeter_t CAR_WIDTH = 2 * CAR_PIVOT_LENGTH;
constexpr millimeter_t CAR_LENGTH = CAR_PIVOT_FRONT_DIST + CAR_PIVOT_DIST_FRONT_REAR + CAR_PIVOT_REAR_DIST;

constexpr millimeter_t CAR_WHEEL_CIRC = millimeter_t(329);              // Wheel circumference.

constexpr uint8_t NUM_OPTO_FRONT = 32;                                  // Number of front opto-sensors.
constexpr uint8_t NUM_OPTO_BACK  = 16;                                  // Number of back opto-sensors.
constexpr uint8_t NUM_OPTO = NUM_OPTO_FRONT + NUM_OPTO_BACK;            // Number of opto-sensors.

constexpr millimeter_t DIST_BTW_OPTOS = millimeter_t(7.742f);           // Distance between the line detector optical sensors.

// Maximum position of the front optical sensor line.
// Leftmost sensor's position:  -MAX_POS_OPTO_FRONT
// Middle position:             0
// Rightmost sensor's position: MAX_POS_OPTO_FRONT
constexpr millimeter_t MAX_POS_OPTO_FRONT = DIST_BTW_OPTOS * NUM_OPTO_FRONT / 2;

// Maximum position of the back optical sensor line.
// Leftmost sensor's position:  -MAX_POS_OPTO_BACK
// Middle position:             0
// Rightmost sensor's position: MAX_POS_OPTO_BACK
constexpr millimeter_t MAX_POS_OPTO_BACK = DIST_BTW_OPTOS * NUM_OPTO_BACK / 2;

constexpr millimeter_t DIST_BTW_OPTO_ROWS = millimeter_t(121);          // Distance between the two line detector optical sensor rows.

constexpr radian_t SERVO_MID = degree_t(74.0f);                         // Servo middle position angle.
constexpr radian_t WHEEL_MAX_DELTA = degree_t(25.0f);                   // Wheel max delta angle.
constexpr float32_t SERVO_WHEEL_TR = 25.0f / 55.0f;                     // Servo-to-wheel transfer rate.

constexpr millimeter_t WHEEL_BASE = millimeter_t(250.0f);               // Distance between left and right wheels.
constexpr millimeter_t WHEEL_LED_DIST = millimeter_t(110.0f);           // Distance between front sensors and front wheels.

constexpr uint32_t ROTARY_PERIOD_MS = 10;

constexpr microsecond_t DC_MOTOR_T_ELECTRICAL = microsecond_t(484.0f);

} // namespace cfg
} // namespace uns
