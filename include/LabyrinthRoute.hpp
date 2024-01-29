#pragma once

#include <utility>

#include <micro/container/set.hpp>
#include <micro/container/vector.hpp>

#include <LabyrinthGraph.hpp>

struct LabyrinthRoute {
    const Segment* startSeg;
    const Segment* destSeg;
    micro::vector<const Connection*, cfg::MAX_NUM_LABYRINTH_SEGMENTS> connections;

    explicit LabyrinthRoute(const Segment *currentSeg = nullptr);

    void push_front(const Connection& c);
    void push_back(const Connection& c);

    void pop_front();

    const Connection* firstConnection() const;
    const Connection* lastConnection() const;

    void reset();
    
    static bool isForwardConnection(const Connection& prevConn, const Segment& currentSeg, const Connection& newConn);

    static LabyrinthRoute create(
        const Connection& prevConn,
        const Segment& currentSeg,
        const Segment& destSeg,
        const JunctionIds& forbiddenJunctions,
        const bool allowBackwardNavigation);
};
