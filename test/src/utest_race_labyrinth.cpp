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
        { graph.findJunction(12), JunctionDecision(PI_2, Direction::LEFT) }
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
        { graph.findJunction(12), JunctionDecision(PI_2, Direction::RIGHT) }
    });
}

TEST(race_labyrinth, U_N) {
    LabyrinthGraph graph = buildRaceLabyrinthGraph();

    const Segment& src         = *graph.findSegment('U');
    const Segment& dest        = *graph.findSegment('N');
    const Junction& prevJunc   = *graph.findJunction(12);
    const Connection& prevConn = **std::find_if(src.edges.begin(), src.edges.end(), [&prevJunc](const Connection *c) {
        return c->junction == &prevJunc;
    });

    checkRoute(prevConn, src, dest, true, {
        { graph.findJunction(13), JunctionDecision(PI, Direction::CENTER) },
    });
}

TEST(race_labyrinth, U_O) {
    LabyrinthGraph graph = buildRaceLabyrinthGraph();

    const Segment& src         = *graph.findSegment('U');
    const Segment& dest        = *graph.findSegment('O');
    const Junction& prevJunc   = *graph.findJunction(13);
    const Connection& prevConn = **std::find_if(src.edges.begin(), src.edges.end(), [&prevJunc](const Connection *c) {
        return c->junction == &prevJunc;
    });

    // TODO: ???
    checkRoute(prevConn, src, dest, true, {
        { graph.findJunction(13), JunctionDecision(PI,          Direction::CENTER) },
        { graph.findJunction(13), JunctionDecision(radian_t(0), Direction::LEFT) }
    });
}

TEST(race_labyrinth, U_M) {
    LabyrinthGraph graph = buildRaceLabyrinthGraph();

    const Segment& src         = *graph.findSegment('U');
    const Segment& dest        = *graph.findSegment('M');
    const Junction& prevJunc   = *graph.findJunction(13);
    const Connection& prevConn = **std::find_if(src.edges.begin(), src.edges.end(), [&prevJunc](const Connection *c) {
        return c->junction == &prevJunc;
    });

    checkRoute(prevConn, src, dest, true, {
        { graph.findJunction(12), JunctionDecision(PI_2, Direction::LEFT) },
        { graph.findJunction(10), JunctionDecision(PI,   Direction::RIGHT) },
    });
}

TEST(race_labyrinth, U_M_backward) {
    LabyrinthGraph graph = buildRaceLabyrinthGraph();

    const Segment& src         = *graph.findSegment('U');
    const Segment& dest        = *graph.findSegment('M');
    const Junction& prevJunc   = *graph.findJunction(12);
    const Connection& prevConn = **std::find_if(src.edges.begin(), src.edges.end(), [&prevJunc](const Connection *c) {
        return c->junction == &prevJunc;
    });

    checkRoute(prevConn, src, dest, true, {
        { graph.findJunction(12), JunctionDecision(PI_2, Direction::LEFT) },
        { graph.findJunction(10), JunctionDecision(PI,   Direction::RIGHT) },
    });
}

TEST(race_labyrinth, U_K) {
    LabyrinthGraph graph = buildRaceLabyrinthGraph();

    const Segment& src         = *graph.findSegment('U');
    const Segment& dest        = *graph.findSegment('K');
    const Junction& prevJunc   = *graph.findJunction(12);
    const Connection& prevConn = **std::find_if(src.edges.begin(), src.edges.end(), [&prevJunc](const Connection *c) {
        return c->junction == &prevJunc;
    });

    checkRoute(prevConn, src, dest, true, {
        { graph.findJunction(13), JunctionDecision(PI,   Direction::CENTER) },
        { graph.findJunction(11), JunctionDecision(PI_2, Direction::LEFT) },
    });
}


TEST(race_labyrinth, L_J) {
    LabyrinthGraph graph = buildRaceLabyrinthGraph();

    const Segment& src         = *graph.findSegment('L');
    const Segment& dest        = *graph.findSegment('J');
    const Junction& prevJunc   = *graph.findJunction(11);
    const Connection& prevConn = **std::find_if(src.edges.begin(), src.edges.end(), [&prevJunc](const Connection *c) {
        return c->junction == &prevJunc;
    });

    checkRoute(prevConn, src, dest, true, {
        { graph.findJunction(9), JunctionDecision(PI_2,     Direction::CENTER) },
        { graph.findJunction(8), JunctionDecision(PI, Direction::CENTER) },
        { graph.findJunction(8), JunctionDecision(radian_t(0), Direction::LEFT) },
    });
}