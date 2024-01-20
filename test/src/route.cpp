#include <micro/test/utils.hpp>

#include <LabyrinthRoute.hpp>

#include "route.hpp"

using namespace micro;

void checkRoute(
    const Connection& prevConn,
    const Segment& src,
    const Segment& dest,
    const micro::set<uint8_t, cfg::MAX_NUM_LABYRINTH_SEGMENTS>& forbiddenJunctions,
    const bool allowBackwardNavigation,
    const RouteConnections& expectedConnections) {
    
    LabyrinthRoute route = LabyrinthRoute::create(prevConn, src, dest, forbiddenJunctions, allowBackwardNavigation);

    ASSERT_EQ(expectedConnections.size(), route.connections.size());

    const Segment *seg = &src;
    for (uint32_t i = 0; i < expectedConnections.size(); ++i) {
        EXPECT_EQ(seg, route.startSeg);
        const Connection *nextConn = route.firstConnection();
        const auto decision = nextConn->getDecision(*(seg = nextConn->getOtherSegment(*seg)));
        EXPECT_EQ(expectedConnections[i].junction, nextConn->junction);
        EXPECT_EQ_JUNCTION_DECISION(expectedConnections[i].decision, decision);
        route.pop_front();
    }

    EXPECT_EQ(&dest, route.startSeg);
}