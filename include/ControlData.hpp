#pragma once

#include <micro/utils/Line.hpp>

namespace micro {

/* @brief Stores data that is sent to the ControlTask.
 **/
struct ControlData {
    m_per_sec_t speed;
    millisecond_t rampTime;
    bool directControl = false;
    bool rearServoEnabled = true;

    // line control
    Line baseline;
    millimeter_t offset;
    radian_t angle; // not used

    // direct control
    radian_t frontWheelAngle;
    radian_t rearWheelAngle;
};

} // namespace micro
