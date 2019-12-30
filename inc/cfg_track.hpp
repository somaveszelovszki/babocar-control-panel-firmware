#pragma once

#include <micro/panel/lines.h>
#include <micro/utils/units.hpp>

namespace cfg {

constexpr uint8_t MAX_NUM_LAB_SEGMENTS = 20;    // Maximum number of segments in the labyrinth.

constexpr uint8_t MAX_NUM_CROSSING_SEGMENTS_SIDE = 3;
constexpr uint8_t MAX_NUM_CROSSING_SEGMENTS      = MAX_NUM_CROSSING_SEGMENTS_SIDE * 2;

constexpr micro::meter_t MIN_JUNCTION_LENGTH = micro::centimeter_t(20);

} // namespace cfg
