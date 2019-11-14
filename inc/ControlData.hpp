#pragma once

#include <micro/utils/Line.hpp>

namespace micro {

/* @brief Stores data that is sent to the ControlTask.
 **/
struct ControlData {
    m_per_sec_t speed;
    Line baseline;
};

} // namespace micro
