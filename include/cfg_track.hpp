#pragma once

#include <micro/utils/units.hpp>

namespace cfg {

#define TEST_TRACK  1
#define RACE_TRACK  2
#define TRACK       RACE_TRACK

constexpr uint8_t         MAX_NUM_LABYRINTH_SEGMENTS     = 25;
constexpr uint8_t         NUM_LABYRINTH_GATE_SEGMENTS    = 15;
constexpr uint8_t         MAX_NUM_CROSSING_SEGMENTS_SIDE = 3;
constexpr uint8_t         MAX_NUM_CROSSING_SEGMENTS      = MAX_NUM_CROSSING_SEGMENTS_SIDE * 2;
constexpr micro::meter_t  MIN_JUNCTION_LENGTH            = micro::centimeter_t(20);
constexpr uint8_t         NUM_RACE_LAPS                  = 6;
constexpr micro::radian_t MAX_TARGET_LINE_ANGLE          = micro::degree_t(18);

enum class ProgramState : uint8_t {
    // Start states
    INVALID              = 0,
    WaitStartSignal      = 1,

    // Labyrinth states
    NavigateLabyrinth    = 2,
    LaneChange           = 3,

    // RaceTrack states
    ReachSafetyCar       = 4,
    FollowSafetyCar      = 5,
    OvertakeSafetyCar    = 6,
    Race                 = 7,
    Race_segFast2        = 8,
    Race_segFast3        = 9,
    Race_segFast4        = 10,
    Finish               = 11,
    Test                 = 12
};

} // namespace cfg
