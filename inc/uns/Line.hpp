#pragma once

#include <uns/util/units.hpp>

namespace uns {

/* @brief Stores data of a detected line.
 **/
struct Line {
    static const distance_t INVALID_POS;    // Invalid line position.
    static const angle_t INVALID_ANGLE;     // Invalid line angle.

    distance_t pos; // The line position at the front sensor line (relative to car vertical middle axis).
    angle_t angle;  // The line angle (relative to car forward angle)
};

/* @brief Defines line placement.
 **/
enum class LinePlacement : uint8_t {
    CENTER  = 0,    // Center position.
    LEFT    = 1,    // Left position.
    RIGHT   = 2     // Right position.
};

} // namespace uns
