#pragma once

#include <micro/container/vec.hpp>

#include <LabyrinthGraph.hpp>

struct RouteConnection {
    const Junction *junction;
    JunctionDecision decision;

    RouteConnection() = default;
    
    RouteConnection(const Junction *junction, const JunctionDecision& decision)
        : junction(junction)
        , decision(decision) {}

    RouteConnection& operator=(const RouteConnection&) = default;
};

typedef micro::vec<RouteConnection, 50> RouteConnections;

void checkRoute(const Connection& prevConn, const Segment& src, const Segment& dest, const RouteConnections& expectedConnections);