#pragma once

#include <micro/container/vector.hpp>

#include <LabyrinthGraph.hpp>

struct RouteConnection {
    char junction{'\0'};
    JunctionDecision decision;

    RouteConnection& operator=(const RouteConnection&) = default;
};

typedef micro::vector<RouteConnection, 50> RouteConnections;

void checkRoute(
    const LabyrinthGraph& graph,
    const char prevJunction,
    const Segment::Id& src,
    const Segment::Id& dest,
    const bool allowBackwardNavigation,
    const RouteConnections& expectedConnections);