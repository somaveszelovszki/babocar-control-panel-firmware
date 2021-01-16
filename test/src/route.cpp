#include <micro/test/utils.hpp>

#include <LabyrinthRoute.hpp>

#include "route.hpp"

using namespace micro;

void checkRoute(const Connection& prevConn, const Segment& src, const Segment& dest, const RouteConnections& expectedConnections) {
    
    LabyrinthRoute route = LabyrinthRoute::create(prevConn, src, dest);

    ASSERT_EQ(expectedConnections.size(), route.connections.size());

    const Segment *seg = &src;
    for (uint32_t i = 0; i < expectedConnections.size(); ++i) {
        EXPECT_EQ(seg, route.startSeg);
        const Connection *nextConn = route.firstConnection();
        EXPECT_EQ(expectedConnections[i].junction, nextConn->junction);
        EXPECT_EQ(expectedConnections[i].decision, nextConn->getDecision(*(seg = nextConn->getOtherSegment(*seg))));
        route.pop_front();
    }

    EXPECT_EQ(&dest, route.startSeg);
}