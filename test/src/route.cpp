#include "route.hpp"

#include <LabyrinthRoute.hpp>
#include <algorithm>
#include <gtest/gtest.h>
#include <micro/test/utils.hpp>

using namespace micro;

void checkRoute(
    const LabyrinthGraph& graph, const char prevJunction, const Segment::Id& src,
    const Segment::Id& dest, const char lastJunction, const JunctionIds& forbiddenJunctions,
    const bool allowBackwardNavigation,
    const micro::vector<RouteConnection, cfg::MAX_NUM_LABYRINTH_SEGMENTS>& expectedConnections) {
    const auto* currentSeg = graph.findSegment(src);
    ASSERT_NE(nullptr, currentSeg);

    const auto* destSeg = graph.findSegment(dest);
    ASSERT_NE(nullptr, destSeg);

    const auto* lastJunc = graph.findJunction(lastJunction);
    ASSERT_NE(nullptr, lastJunc);

    const auto prevConn =
        std::find_if(currentSeg->edges.begin(), currentSeg->edges.end(),
                     [prevJunction](const auto& c) { return c->junction->id == prevJunction; });
    ASSERT_NE(currentSeg->edges.end(), prevConn);

    auto route = LabyrinthRoute::create(**prevConn, *currentSeg, *destSeg, *lastJunc,
                                        forbiddenJunctions, allowBackwardNavigation);
    ASSERT_EQ(expectedConnections.size(), route.connections.size());

    const Segment* seg = currentSeg;
    for (uint32_t i = 0; i < expectedConnections.size(); ++i) {
        EXPECT_EQ(seg, route.startSeg);
        const Connection* nextConn = route.firstConnection();
        EXPECT_EQ(expectedConnections[i].junction, nextConn->junction->id);
        const auto decision = nextConn->getDecision(*(seg = nextConn->getOtherSegment(*seg)));
        EXPECT_EQ_JUNCTION_DECISION(expectedConnections[i].decision, decision);
        route.pop_front();
    }
}
