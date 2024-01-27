#include <cfg_track.hpp>

#if TRACK == RACE_TRACK || COMPILE_ALL_TRACKS

#include <LabyrinthGraph.hpp>
#include <track.hpp>

using namespace micro;

LabyrinthGraph buildRaceLabyrinthGraph() {
    LabyrinthGraph graph;
    return graph;
}

#endif // TRACK == RACE_TRACK || COMPILE_ALL_TRACKS
