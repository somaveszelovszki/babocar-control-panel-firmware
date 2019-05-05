#pragma once

#include <uns/Point2.hpp>

namespace uns {

struct Pose {

    /* @brief Position of the car relative to its start position.
    **/
    Point2<meter_t> pos;

    /* @brief Car's orientation.
    @note Orientation is relative to the X axis!
     **/
    radian_t angle;
};

/* @brief Car properties.
 **/
struct CarProps  {
    /* @brief Default constructor - initializes fields.
     **/
    CarProps()
        : pose{ { meter_t(0.0f), meter_t(0.0f) }, PI_2 }
        , speed(0.0f) {}

    Pose pose;          // The current pose (position and orientation) of the car.
    m_per_sec_t speed;  // The current speed of the car.
};
} // namespace uns
