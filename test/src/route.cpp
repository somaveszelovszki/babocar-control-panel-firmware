#include <algorithm>

#include <gtest/gtest.h>
#include <micro/test/utils.hpp>

#include <LabyrinthRoute.hpp>

#include "route.hpp"

using namespace micro;

void checkRoute(
    const LabyrinthGraph& graph,
    const char prevJunction,
    const Segment::Id& src,
    const Segment::Id& dest,
    const bool allowBackwardNavigation,
    const RouteConnections& expectedConnections) {

    const auto* currentSeg = graph.findSegment(src);
    ASSERT_NE(nullptr, currentSeg);

    const auto prevConn = std::find_if(currentSeg->edges.begin(),currentSeg->edges.end(),
        [prevJunction](const auto& c){ return c->junction->id == prevJunction; });
    ASSERT_NE(currentSeg->edges.end(), prevConn);

    const auto* destSeg = graph.findSegment(dest);
    ASSERT_NE(nullptr, destSeg);


    auto route = LabyrinthRoute::create(**prevConn, *currentSeg, *destSeg, allowBackwardNavigation);
    ASSERT_EQ(expectedConnections.size(), route.connections.size());

    const Segment *seg = currentSeg;
    for (uint32_t i = 0; i < expectedConnections.size(); ++i) {
        EXPECT_EQ(seg, route.startSeg);
        const Connection *nextConn = route.firstConnection();
        EXPECT_EQ(expectedConnections[i].junction, nextConn->junction->id);
        const auto decision = nextConn->getDecision(*(seg = nextConn->getOtherSegment(*seg)));
        EXPECT_EQ_JUNCTION_DECISION(expectedConnections[i].decision, decision);
        route.pop_front();
    }
}