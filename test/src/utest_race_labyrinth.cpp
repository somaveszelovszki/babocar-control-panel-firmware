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

TEST(race_labyrinth, W_J) {
    LabyrinthGraph graph = buildRaceLabyrinthGraph();

    const Segment& src         = *graph.findSegment('W');
    const Segment& dest        = *graph.findSegment('J');
    const Junction& prevJunc   = *graph.findJunction(13);
    const Connection& prevConn = **std::find_if(src.edges.begin(), src.edges.end(), [&prevJunc](const Connection *c) {
        return c->junction == &prevJunc;
    });

    checkRoute(prevConn, src, dest, false, {
        { graph.findJunction(12), JunctionDecision(radian_t(PI), Direction::RIGHT) }
    });
}

TEST(race_labyrinth, W_M) {
    LabyrinthGraph graph = buildRaceLabyrinthGraph();

    const Segment& src         = *graph.findSegment('W');
    const Segment& dest        = *graph.findSegment('M');
    const Junction& prevJunc   = *graph.findJunction(13);
    const Connection& prevConn = **std::find_if(src.edges.begin(), src.edges.end(), [&prevJunc](const Connection *c) {
        return c->junction == &prevJunc;
    });

    checkRoute(prevConn, src, dest, false, {
        { graph.findJunction(12), JunctionDecision(radian_t(PI), Direction::LEFT) },
        { graph.findJunction(10), JunctionDecision(radian_t(3 * PI_2), Direction::RIGHT) }
    });
}