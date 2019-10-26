#pragma once

#include <micro/utils/Line.hpp>

namespace micro {

/* @brief Stores data that is sent to the ControlTask.
 **/
struct ControlData {
public:
    m_per_sec_t speed;
    Line baseline;
    millimeter_t baselineOffset;
    radian_t baselineAngle;
};

} // namespace micro
