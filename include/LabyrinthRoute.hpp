#pragma once

#include <LabyrinthGraph.hpp>

#include <utility>

struct LabyrinthRoute {
    static constexpr uint32_t MAX_LENGTH = 2 * cfg::MAX_NUM_LABYRINTH_SEGMENTS;
    const Segment* startSeg;
    const Segment* destSeg;
    micro::vec<const Connection*, MAX_LENGTH> connections;

    explicit LabyrinthRoute(const Segment *currentSeg = nullptr);

    void push_front(const Connection& c);
    void push_back(const Connection& c);

    void pop_front();

    const Connection* firstConnection() const;
    const Connection* lastConnection() const;

    void reset(const Segment& currentSeg);

    static bool isNewConnectionValid(const Connection& prevConn, const Segment& currentSeg, const Connection& newConn);

    static LabyrinthRoute create(const Connection& prevConn, const Segment& currentSeg, const Segment& destSeg);
};
