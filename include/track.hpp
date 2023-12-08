#pragma once

#include <micro/utils/Line.hpp>

#include <cfg_track.hpp>
#include <RaceTrackController.hpp>

class LabyrinthGraph;

#if TRACK == RACE_TRACK || COMPILE_ALL_TRACKS

class RaceLapTrackSectionProvider : public ILapTrackSectionProvider {
public:
	LapTrackSections operator()(const size_t lap) override;
	~RaceLapTrackSectionProvider() = default;

private:
	micro::OrientedLine lastLineGradient_;
};

extern OverridableLapTrackSectionProvider raceLapTrackSectionProvider;
LabyrinthGraph buildRaceLabyrinthGraph();

#endif // TRACK == RACE_TRACK || COMPILE_ALL_TRACKS

#if TRACK == TEST_TRACK || COMPILE_ALL_TRACKS

class TestLapTrackSectionProvider : public ILapTrackSectionProvider {
public:
	LapTrackSections operator()(const size_t lap) override;
	~TestLapTrackSectionProvider() = default;

private:
	micro::OrientedLine lastLineGradient_;
};

extern OverridableLapTrackSectionProvider testLapTrackSectionProvider;
LabyrinthGraph buildTestLabyrinthGraph();

#endif // TRACK == TEST_TRACK || COMPILE_ALL_TRACKS

#if TRACK == RACE_TRACK

#define lapTrackSectionProvider raceLapTrackSectionProvider
#define buildLabyrinthGraph()   buildRaceLabyrinthGraph()

#elif TRACK == TEST_TRACK

#define lapTrackSectionProvider testLapTrackSectionProvider
#define buildLabyrinthGraph()   buildTestLabyrinthGraph()

#endif // TRACK == TEST_TRACK

