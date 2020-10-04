#include <cfg_track.hpp>
#include <LabyrinthGraph.hpp>
#include <LabyrinthGraphBuilder.hpp>

using namespace micro;

LabyrinthGraph buildLabyrinthGraph() {
    LabyrinthGraph graph;

#if LABYRINTH == TEST_LABYRINTH

    graph.addSegment(Segment('A', meter_t(2), false));
    graph.addSegment(Segment('B', meter_t(3), false));

    graph.addJunction(Junction(1, { meter_t(2), meter_t(0) }));

    graph.connect(graph.findSegment('A'), graph.findJunction(1), Maneuver(PI, Direction::CENTER));
    graph.connect(graph.findSegment('B'), graph.findJunction(1), Maneuver(radian_t(0), Direction::LEFT));
    graph.connect(graph.findSegment('B'), graph.findJunction(1), Maneuver(radian_t(0), Direction::RIGHT));

#elif LABYRINTH == RACE_LABYRINTH

#endif

    return graph;
}
