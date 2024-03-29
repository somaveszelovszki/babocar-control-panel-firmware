#pragma once

#include <micro/utils/units.hpp>

namespace cfg {

#define TEST_TRACK  1
#define RACE_TRACK  2
#define TRACK       RACE_TRACK

constexpr uint8_t         MAX_NUM_LABYRINTH_JUNCTIONS    = 25;
constexpr uint8_t         MAX_NUM_LABYRINTH_SEGMENTS     = 50;
constexpr uint8_t         MAX_NUM_CROSSING_SEGMENTS_SIDE = 3;
constexpr uint8_t         MAX_NUM_CROSSING_SEGMENTS      = MAX_NUM_CROSSING_SEGMENTS_SIDE * 2;
constexpr micro::meter_t  MIN_JUNCTION_LENGTH            = micro::centimeter_t(20);
constexpr uint8_t         NUM_RACE_LAPS                  = 6;
constexpr uint8_t         MAX_NUM_RACE_SEGMENTS          = 25;
constexpr micro::radian_t MAX_TARGET_LINE_ANGLE          = micro::degree_t(18);

} // namespace cfg
