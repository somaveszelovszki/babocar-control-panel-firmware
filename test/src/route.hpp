#pragma once

#include "cfg_track.hpp"
#include <micro/container/vector.hpp>

#include <LabyrinthGraph.hpp>

struct RouteConnection {
    char junction{'\0'};
    JunctionDecision decision;

    RouteConnection& operator=(const RouteConnection&) = default;
};

void checkRoute(
    const LabyrinthGraph& graph,
    const char prevJunction,
    const Segment::Id& src,
    const micro::set<Segment::Id, cfg::MAX_NUM_LABYRINTH_SEGMENTS> destSegments,
    const micro::set<Segment::Id, cfg::MAX_NUM_LABYRINTH_SEGMENTS> forbiddenSegments,
    const micro::set<char, cfg::MAX_NUM_LABYRINTH_SEGMENTS> forbiddenJunctions,
    const bool allowBackwardNavigation,
    const micro::vector<RouteConnection, cfg::MAX_NUM_LABYRINTH_SEGMENTS>& expectedConnections);