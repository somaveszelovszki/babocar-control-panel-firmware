#pragma once

#include <micro/container/vec.hpp>
#include <micro/utils/CarProps.hpp>
#include <micro/utils/ControlData.hpp>
#include <micro/utils/LinePattern.hpp>

#include <cfg_track.hpp>

#include <functional>

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

typedef micro::vec<TrackSegment, 20> TrackSegments;

extern const TrackSegments testTrackSegments;
extern const TrackSegments raceTrackSegments;

LabyrinthGraph buildTestLabyrinthGraph();
LabyrinthGraph buildRaceLabyrinthGraph();

#if TRACK == RACE_TRACK

#define trackSegments           raceTrackSegments
#define buildLabyrinthGraph()   buildRaceLabyrinthGraph()

#elif TRACK == TEST_TRACK

#define trackSegments           testTrackSegments
#define buildLabyrinthGraph()   buildTestLabyrinthGraph()

#else
#error "TRACK must be set to either TEST_TRACK or RACE_TRACK"
#endif
