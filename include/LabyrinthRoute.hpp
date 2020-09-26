#pragma once

#include <LabyrinthGraph.hpp>

struct Route {
    static constexpr uint32_t MAX_LENGTH = 2 * cfg::NUM_LABYRINTH_SEGMENTS;
    const Segment *startSeg;
    micro::vec<const Connection*, MAX_LENGTH> connections;

    explicit Route(const Segment& currentSeg) : startSeg(&currentSeg) {}

    void push_front(const Connection& c);
    void push_back(const Connection& c);

    const Connection* nextConnection();

    const Connection* lastConnection() const;

    void reset(const Segment& currentSeg);
};

Route createRoute(const Connection& prevConn, const Segment& currentSeg, const Segment& destSeg);
