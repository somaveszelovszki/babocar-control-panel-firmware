#pragma once

#include <LabyrinthGraph.hpp>
#include <cfg_track.hpp>

#include <micro/container/vector.hpp>

struct RouteConnection {
    char junction{'\0'};
    JunctionDecision decision;

    RouteConnection& operator=(const RouteConnection&) = default;
};

void checkRoute(
    const LabyrinthGraph& graph, const char prevJunction, const Segment::Id& src,
    const Segment::Id& dest, const char lastJunction, const JunctionIds& forbiddenJunctions,
    const bool allowBackwardNavigation,
    const micro::vector<RouteConnection, cfg::MAX_NUM_LABYRINTH_SEGMENTS>& expectedConnections);
