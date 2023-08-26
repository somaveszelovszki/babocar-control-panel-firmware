#pragma once

#include <cfg_track.hpp>
#include <RaceTrackController.hpp>

class LabyrinthGraph;

RaceTrackSections buildTestTrackSections();
LabyrinthGraph buildTestLabyrinthGraph();

RaceTrackSections buildRaceTrackSections();
LabyrinthGraph buildRaceLabyrinthGraph();

#if TRACK == RACE_TRACK

#define buildTrackSections()  buildRaceTrackSections()
#define buildLabyrinthGraph() buildRaceLabyrinthGraph()

#elif TRACK == TEST_TRACK

#define buildTrackSections()  buildTestTrackSections()
#define buildLabyrinthGraph() buildTestLabyrinthGraph()

#endif // TRACK == TEST_TRACK

