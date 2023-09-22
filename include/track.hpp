#pragma once

#include <cfg_track.hpp>
#include <RaceTrackController.hpp>

class LabyrinthGraph;

#if TRACK == TEST_TRACK || COMPILE_ALL_TRACKS

RaceTrackSections buildTestTrackSections();
LabyrinthGraph buildTestLabyrinthGraph();

#endif // TRACK == TEST_TRACK || COMPILE_ALL_TRACKS

#if TRACK == RACE_TRACK || COMPILE_ALL_TRACKS

RaceTrackSections buildRaceTrackSections();
LabyrinthGraph buildRaceLabyrinthGraph();

#endif // TRACK == RACE_TRACK || COMPILE_ALL_TRACKS

#if TRACK == RACE_TRACK

#define buildTrackSections()  buildRaceTrackSections()
#define buildLabyrinthGraph() buildRaceLabyrinthGraph()

#elif TRACK == TEST_TRACK

#define buildTrackSections()  buildTestTrackSections()
#define buildLabyrinthGraph() buildTestLabyrinthGraph()

#endif // TRACK == TEST_TRACK

