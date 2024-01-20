#pragma once

#include <utility>

#include <micro/container/set.hpp>
#include <micro/container/vector.hpp>

#include <LabyrinthGraph.hpp>

struct LabyrinthRoute {
    static constexpr uint32_t MAX_LENGTH = 2 * cfg::MAX_NUM_LABYRINTH_SEGMENTS;
    const Segment* startSeg;
    const Segment* destSeg;
    micro::vector<const Connection*, MAX_LENGTH> connections;

    explicit LabyrinthRoute(const Segment *currentSeg = nullptr);

    void push_front(const Connection& c);
    void push_back(const Connection& c);

    void pop_front();

    const Connection* firstConnection() const;
    const Connection* lastConnection() const;

    void reset(const Segment& currentSeg);

    static bool isForwardConnection(const Connection& prevConn, const Segment& currentSeg, const Connection& newConn);

    static LabyrinthRoute create(
        const Connection& prevConn,
        const Segment& currentSeg,
        const Segment& destSeg,
        const micro::set<uint8_t, cfg::MAX_NUM_LABYRINTH_SEGMENTS>& forbiddenJunctions,
        const bool allowBackwardNavigation);
};
