#include <micro/test/utils.hpp>

#include <LabyrinthGraph.hpp>
#include <LabyrinthRoute.hpp>
#include <track.hpp>

#include "route.hpp"

using namespace micro;

TEST(race_labyrinth, valid) {
    LabyrinthGraph graph = buildRaceLabyrinthGraph();
    ASSERT_TRUE(graph.valid());
}

TEST(race_labyrinth, U_T) {
    LabyrinthGraph graph = buildRaceLabyrinthGraph();

    const Segment& src         = *graph.findSegment('U');
    const Segment& dest        = *graph.findSegment('T');
    const Junction& prevJunc   = *graph.findJunction(13);
    const Connection& prevConn = **std::find_if(src.edges.begin(), src.edges.end(), [&prevJunc](const Connection *c) {
        return c->junction == &prevJunc;
    });

    checkRoute(prevConn, src, dest, false, {
        { graph.findJunction(12), JunctionDecision(radian_t(PI_2), Direction::LEFT) }
    });
}

TEST(race_labyrinth, U_J) {
    LabyrinthGraph graph = buildRaceLabyrinthGraph();

    const Segment& src         = *graph.findSegment('U');
    const Segment& dest        = *graph.findSegment('J');
    const Junction& prevJunc   = *graph.findJunction(13);
    const Connection& prevConn = **std::find_if(src.edges.begin(), src.edges.end(), [&prevJunc](const Connection *c) {
        return c->junction == &prevJunc;
    });

    checkRoute(prevConn, src, dest, false, {
        { graph.findJunction(12), JunctionDecision(radian_t(PI_2), Direction::RIGHT) }
    });
}