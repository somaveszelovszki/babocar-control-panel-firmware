#pragma once

#include <uns/util/units.hpp>

namespace uns {
namespace cfg {

constexpr distance_t CAR_PIVOT_DIST_FRONT_REAR(millimeters(), 200);       // distance between front and rear pivots
constexpr distance_t CAR_PIVOT_DIST_MID(CAR_PIVOT_DIST_FRONT_REAR / 2);   // distance between pivot and car middle
constexpr distance_t CAR_PIVOT_LENGTH(millimeters(), 80);                 // length of front and rear pivots (middle to wheel)
constexpr distance_t CAR_PIVOT_FRONT_DIST(millimeters(), 72);             // distance between car front and front pivot
constexpr distance_t CAR_PIVOT_REAR_DIST(millimeters(), 72);              // distance between car rear and rear pivot
constexpr distance_t CAR_WIDTH(2 * CAR_PIVOT_LENGTH);
constexpr distance_t CAR_LENGTH(CAR_PIVOT_FRONT_DIST + CAR_PIVOT_DIST_FRONT_REAR + CAR_PIVOT_REAR_DIST);

constexpr distance_t CAR_WHEEL_CIRC(millimeters(), 329);                // Wheel circumference.

constexpr uint8_t NUM_OPTO_FRONT = 32;                                  // Number of front opto-sensors.
constexpr uint8_t NUM_OPTO_BACK  = 16;                                  // Number of back opto-sensors.
constexpr uint8_t NUM_OPTO = NUM_OPTO_FRONT + NUM_OPTO_BACK;            // Number of opto-sensors.

constexpr distance_t DIST_BTW_OPTOS(micrometers(), 7742);                 // Distance between the line detector optical sensors.

// Maximum position of the front optical sensor line.
// Leftmost sensor's position:  -MAX_POS_OPTO_FRONT
// Middle position:             0
// Rightmost sensor's position: MAX_POS_OPTO_FRONT
constexpr distance_t MAX_POS_OPTO_FRONT = DIST_BTW_OPTOS * NUM_OPTO_FRONT / 2;

// Maximum position of the back optical sensor line.
// Leftmost sensor's position:  -MAX_POS_OPTO_BACK
// Middle position:             0
// Rightmost sensor's position: MAX_POS_OPTO_BACK
constexpr distance_t MAX_POS_OPTO_BACK = DIST_BTW_OPTOS * NUM_OPTO_BACK / 2;

constexpr distance_t DIST_BTW_OPTO_ROWS(millimeters(), 121);              // Distance between the two line detector optical sensor rows.

constexpr angle_t SERVO_MID(degrees(), 74.0f);                            // Servo middle position angle.
constexpr angle_t WHEEL_MAX_DELTA(degrees(), 25.0f);                      // Wheel max delta angle.
constexpr float32_t SERVO_WHEEL_TR = 25.0f / 55.0f;                     // Servo-to-wheel transfer rate.

constexpr distance_t WHEEL_BASE(millimeters(), 250.0f);                 // Distance between left and right wheels.
constexpr distance_t WHEEL_LED_DIST(millimeters(), 110.0f);             // Distance between front sensors and front wheels.

constexpr uint32_t ROTARY_PERIOD_MS = 10;

constexpr bool USE_SAFETY_ENABLE_SIGNAL = true;
constexpr bool INDICATOR_LEDS_ENABLED = true;
constexpr bool START_SIGNAL_ENABLED = false;

static constexpr time_t DC_MOTOR_T_ELECTRICAL(microseconds(), 484.0f);

} // namespace cfg
} // namespace uns
