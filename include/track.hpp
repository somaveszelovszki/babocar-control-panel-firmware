#pragma once

#include <micro/container/vec.hpp>
#include <micro/utils/CarProps.hpp>
#include <micro/utils/ControlData.hpp>
#include <micro/utils/LinePattern.hpp>

#include <cfg_track.hpp>

#include <functional>

using TrackSpeeds = micro::vec<micro::m_per_sec_t, 20>;

struct AccelerationRamps {
    micro::millisecond_t fast1;
    micro::millisecond_t fast2;
    micro::millisecond_t fast3;
    micro::millisecond_t fast4;
};

struct BrakeOffsets {
    micro::meter_t slow1;
    micro::meter_t slow2;
    micro::meter_t slow3;
    micro::meter_t slow4;
};

struct RaceTrackInfo;
class LabyrinthGraph;

struct TrackSegment {
    bool isFast;
    micro::meter_t length;
    std::function<bool(const micro::CarProps&, const RaceTrackInfo&, const micro::LinePattern&)> hasBecomeActive;
    std::function<micro::ControlData(const micro::CarProps&, const RaceTrackInfo&, const micro::MainLine&)> getControl;
};

using TrackSegments = micro::vec<TrackSegment, 20>;

extern TrackSpeeds trackSpeeds[cfg::NUM_RACE_LAPS + 1];
extern AccelerationRamps accelerationRamps[cfg::NUM_RACE_LAPS + 1];
extern BrakeOffsets brakeOffsets[cfg::NUM_RACE_LAPS + 1];
extern const TrackSegments trackSegments;

LabyrinthGraph buildLabyrinthGraph();
