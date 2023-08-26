#include <micro/test/utils.hpp>

#include <LabyrinthGraph.hpp>
#include <LabyrinthRoute.hpp>
#include <track.hpp>

#include "route.hpp"

using namespace micro;

namespace {

LabyrinthGraph createGraph() {
    LabyrinthGraph graph;

    graph.addSegment(Segment('A', meter_t(6), false));
    graph.addSegment(Segment('B', meter_t(1), false));
    graph.addSegment(Segment('C', meter_t(4), false));
    graph.addSegment(Segment('D', meter_t(4), false));
    graph.addSegment(Segment('E', meter_t(4), false));
    graph.addSegment(Segment('F', meter_t(3), false));
    graph.addSegment(Segment('G', meter_t(4), false));
    graph.addSegment(Segment('H', meter_t(1), false));
    graph.addSegment(Segment('I', meter_t(3), true));
    graph.addSegment(Segment('J', meter_t(4), false));
    graph.addSegment(Segment('K', meter_t(3), false));
    graph.addSegment(Segment('L', meter_t(5), false));

    graph.addJunction(Junction(1, { meter_t(3.5), meter_t(7.0) }));
    graph.addJunction(Junction(2, { meter_t(1.0), meter_t(7.5) }));
    graph.addJunction(Junction(3, { meter_t(4.0), meter_t(5.5) }));
    graph.addJunction(Junction(4, { meter_t(6.0), meter_t(2.0) }));
    graph.addJunction(Junction(5, { meter_t(4.5), meter_t(1.0) }));
    graph.addJunction(Junction(6, { meter_t(2.0), meter_t(2.5) }));
    graph.addJunction(Junction(7, { meter_t(2.0), meter_t(3.5) }));

    graph.connect(graph.findSegment('A'), graph.findJunction(1), JunctionDecision(radian_t(0), Direction::LEFT));
    graph.connect(graph.findSegment('B'), graph.findJunction(1), JunctionDecision(radian_t(0), Direction::RIGHT));
    graph.connect(graph.findSegment('K'), graph.findJunction(1), JunctionDecision(PI,          Direction::LEFT));
    graph.connect(graph.findSegment('C'), graph.findJunction(1), JunctionDecision(PI,          Direction::RIGHT));

    graph.connect(graph.findSegment('C'), graph.findJunction(2), JunctionDecision(PI_2,        Direction::CENTER));
    graph.connect(graph.findSegment('K'), graph.findJunction(2), JunctionDecision(3 * PI_2,    Direction::LEFT));
    graph.connect(graph.findSegment('J'), graph.findJunction(2), JunctionDecision(3 * PI_2,    Direction::RIGHT));

    graph.connect(graph.findSegment('B'), graph.findJunction(3), JunctionDecision(PI_2,        Direction::CENTER));
    graph.connect(graph.findSegment('D'), graph.findJunction(3), JunctionDecision(3 * PI_2,    Direction::LEFT));
    graph.connect(graph.findSegment('L'), graph.findJunction(3), JunctionDecision(3 * PI_2,    Direction::CENTER));
    graph.connect(graph.findSegment('E'), graph.findJunction(3), JunctionDecision(3 * PI_2,    Direction::RIGHT));

    graph.connect(graph.findSegment('D'), graph.findJunction(4), JunctionDecision(PI_2,        Direction::LEFT));
    graph.connect(graph.findSegment('A'), graph.findJunction(4), JunctionDecision(PI_2,        Direction::RIGHT));
    graph.connect(graph.findSegment('F'), graph.findJunction(4), JunctionDecision(3 * PI_2,    Direction::CENTER));

    graph.connect(graph.findSegment('F'), graph.findJunction(5), JunctionDecision(radian_t(0), Direction::CENTER));
    graph.connect(graph.findSegment('G'), graph.findJunction(5), JunctionDecision(PI,          Direction::LEFT));
    graph.connect(graph.findSegment('L'), graph.findJunction(5), JunctionDecision(PI,          Direction::RIGHT));

    graph.connect(graph.findSegment('H'), graph.findJunction(6), JunctionDecision(PI_2,        Direction::LEFT));
    graph.connect(graph.findSegment('E'), graph.findJunction(6), JunctionDecision(PI_2,        Direction::RIGHT));
    graph.connect(graph.findSegment('G'), graph.findJunction(6), JunctionDecision(3 * PI_2,    Direction::CENTER));

    graph.connect(graph.findSegment('J'), graph.findJunction(7), JunctionDecision(PI_2,        Direction::LEFT));
    graph.connect(graph.findSegment('I'), graph.findJunction(7), JunctionDecision(PI_2,        Direction::RIGHT));
    graph.connect(graph.findSegment('H'), graph.findJunction(7), JunctionDecision(3 * PI_2,    Direction::CENTER));

    return graph;
}

} // namespace

TEST(labyrinthRoute, route_0_conn) {
    LabyrinthGraph graph = createGraph();

    const Segment& src         = *graph.findSegment('L');
    const Segment& dest        = *graph.findSegment('L');
    const Junction& prevJunc   = *graph.findJunction(3);
    const Connection& prevConn = **std::find_if(src.edges.begin(), src.edges.end(), [&prevJunc](const Connection *c) {
        return c->junction == &prevJunc;
    });

    checkRoute(prevConn, src, dest, false, {});
}

TEST(labyrinthRoute, route_1_conn) {
    LabyrinthGraph graph = createGraph();

    const Segment& src         = *graph.findSegment('A');
    const Segment& dest        = *graph.findSegment('C');
    const Junction& prevJunc   = *graph.findJunction(4);
    const Connection& prevConn = **std::find_if(src.edges.begin(), src.edges.end(), [&prevJunc](const Connection *c) {
        return c->junction == &prevJunc;
    });

    checkRoute(prevConn, src, dest, false, {
        { graph.findJunction(1), JunctionDecision(PI, Direction::RIGHT) }
    });
}

TEST(labyrinthRoute, route_4_conn_hoop) {
    LabyrinthGraph graph = createGraph();

    const Segment& src         = *graph.findSegment('B');
    const Segment& dest        = *graph.findSegment('D');
    const Junction& prevJunc   = *graph.findJunction(3);
    const Connection& prevConn = **std::find_if(src.edges.begin(), src.edges.end(), [&prevJunc](const Connection *c) {
        return c->junction == &prevJunc;
    });

    checkRoute(prevConn, src, dest, false, {
        { graph.findJunction(1), JunctionDecision(PI,          Direction::LEFT) },
        { graph.findJunction(2), JunctionDecision(PI_2,        Direction::CENTER) },
        { graph.findJunction(1), JunctionDecision(radian_t(0), Direction::RIGHT) },
        { graph.findJunction(3), JunctionDecision(3 * PI_2,    Direction::LEFT) },
    });
}

TEST(labyrinthRoute, route_3_conn_fastest) {
    LabyrinthGraph graph = createGraph();

    const Segment& src         = *graph.findSegment('F');
    const Segment& dest        = *graph.findSegment('C');
    const Junction& prevJunc   = *graph.findJunction(5);
    const Connection& prevConn = **std::find_if(src.edges.begin(), src.edges.end(), [&prevJunc](const Connection *c) {
        return c->junction == &prevJunc;
    });

    checkRoute(prevConn, src, dest, false, {
        { graph.findJunction(4), JunctionDecision(PI_2, Direction::LEFT) },
        { graph.findJunction(3), JunctionDecision(PI_2, Direction::CENTER) },
        { graph.findJunction(1), JunctionDecision(PI,   Direction::RIGHT) }
    });
}
