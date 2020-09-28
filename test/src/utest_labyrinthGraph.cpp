#include <micro/test/utils.hpp>

#include <LabyrinthGraph.hpp>
#include <LabyrinthGraphBuilder.hpp>

using namespace micro;

namespace {

LabyrinthGraph createGraph() {
    LabyrinthGraph graph;

    graph.addSegment(Segment('A', meter_t(4), false));
    graph.addSegment(Segment('B', meter_t(4), false));
    graph.addSegment(Segment('C', meter_t(4), false));

    graph.addJunction(Junction(1, { meter_t(2),  meter_t(0) }));
    graph.addJunction(Junction(2, { meter_t(-2), meter_t(0) }));

    graph.connect(graph.findSegment('A'), graph.findJunction(1), Maneuver(PI,          Direction::CENTER));
    graph.connect(graph.findSegment('B'), graph.findJunction(1), Maneuver(radian_t(0), Direction::LEFT));
    graph.connect(graph.findSegment('B'), graph.findJunction(1), Maneuver(radian_t(0), Direction::RIGHT));

    graph.connect(graph.findSegment('A'), graph.findJunction(2), Maneuver(radian_t(0), Direction::CENTER));
    graph.connect(graph.findSegment('C'), graph.findJunction(2), Maneuver(PI,          Direction::LEFT));
    graph.connect(graph.findSegment('C'), graph.findJunction(2), Maneuver(PI,          Direction::RIGHT));

    return graph;
}

} // namespace

TEST(labyrinthGraph, segments) {
    LabyrinthGraph graph = createGraph();

    LabyrinthGraph::Segments::iterator segA = graph.findSegment('A');
    ASSERT_NE(graph.segments.end(), segA);
    ASSERT_EQ('A', segA->name);

    LabyrinthGraph::Segments::iterator segB = graph.findSegment('B');
    ASSERT_NE(graph.segments.end(), segB);
    ASSERT_EQ('B', segB->name);

    LabyrinthGraph::Segments::iterator segC = graph.findSegment('C');
    ASSERT_NE(graph.segments.end(), segC);
    ASSERT_EQ('C', segC->name);
}

TEST(labyrinthGraph, edges) {
    LabyrinthGraph graph = createGraph();

    LabyrinthGraph::Segments::iterator segA = graph.findSegment('A');
    ASSERT_NE(graph.segments.end(), segA);
    ASSERT_EQ(4, segA->edges.size());
    EXPECT_EQ(graph.findSegment('B'), segA->edges[0]->getOtherSegment(*segA));
    EXPECT_EQ(graph.findSegment('B'), segA->edges[1]->getOtherSegment(*segA));
    EXPECT_EQ(graph.findSegment('C'), segA->edges[2]->getOtherSegment(*segA));
    EXPECT_EQ(graph.findSegment('C'), segA->edges[3]->getOtherSegment(*segA));

    LabyrinthGraph::Segments::iterator segB = graph.findSegment('B');
    ASSERT_NE(graph.segments.end(), segB);
    ASSERT_EQ(2, segB->edges.size());
    EXPECT_EQ(graph.findSegment('A'), segB->edges[0]->getOtherSegment(*segB));
    EXPECT_EQ(graph.findSegment('A'), segB->edges[1]->getOtherSegment(*segB));

    LabyrinthGraph::Segments::iterator segC = graph.findSegment('C');
    ASSERT_NE(graph.segments.end(), segC);
    ASSERT_EQ(2, segC->edges.size());
    EXPECT_EQ(graph.findSegment('A'), segC->edges[0]->getOtherSegment(*segC));
    EXPECT_EQ(graph.findSegment('A'), segC->edges[1]->getOtherSegment(*segC));
}

TEST(labyrinthGraph, junctions) {
    LabyrinthGraph graph = createGraph();

    LabyrinthGraph::Junctions::iterator j1 = graph.findJunction(1);
    ASSERT_NE(graph.junctions.end(), j1);
    ASSERT_EQ(2, j1->segments.size());

    Junction::segment_map::const_iterator j1_rightSideSegs = j1->getSideSegments(radian_t(0));
    ASSERT_NE(j1->segments.end(), j1_rightSideSegs);
    ASSERT_EQ(2, j1_rightSideSegs->second.size());
    EXPECT_EQ(graph.findSegment('B'), *j1_rightSideSegs->second.at(Direction::LEFT));
    EXPECT_EQ(graph.findSegment('B'), *j1_rightSideSegs->second.at(Direction::RIGHT));

    Junction::segment_map::const_iterator j1_leftSideSegs = j1->getSideSegments(PI);
    ASSERT_NE(j1->segments.end(), j1_leftSideSegs);
    ASSERT_EQ(1, j1_leftSideSegs->second.size());
    EXPECT_EQ(graph.findSegment('A'), *j1_leftSideSegs->second.at(Direction::CENTER));

    LabyrinthGraph::Junctions::iterator j2 = graph.findJunction(2);
    ASSERT_NE(graph.junctions.end(), j2);
    ASSERT_EQ(2, j2->segments.size());

    Junction::segment_map::const_iterator j2_rightSideSegs = j2->getSideSegments(radian_t(0));
    ASSERT_NE(j1->segments.end(), j2_rightSideSegs);
    ASSERT_EQ(1, j2_rightSideSegs->second.size());
    EXPECT_EQ(graph.findSegment('A'), *j2_rightSideSegs->second.at(Direction::CENTER));

    Junction::segment_map::const_iterator j2_leftSideSegs = j2->getSideSegments(PI);
    ASSERT_NE(j1->segments.end(), j2_leftSideSegs);
    ASSERT_EQ(2, j2_leftSideSegs->second.size());
    EXPECT_EQ(graph.findSegment('C'), *j2_leftSideSegs->second.at(Direction::LEFT));
    EXPECT_EQ(graph.findSegment('C'), *j2_leftSideSegs->second.at(Direction::RIGHT));
}

TEST(labyrinthGraph, connections) {
    LabyrinthGraph graph = createGraph();

    ASSERT_EQ(4, graph.connections.size());

    EXPECT_EQ(graph.findJunction(1), graph.connections[0].junction);
    EXPECT_EQ(graph.findSegment('B'), graph.connections[0].node1);
    EXPECT_EQ(Maneuver(radian_t(0), Direction::LEFT), graph.connections[0].maneuver1);
    EXPECT_EQ(graph.findSegment('A'), graph.connections[0].node2);
    EXPECT_EQ(Maneuver(PI, Direction::CENTER), graph.connections[0].maneuver2);

    EXPECT_EQ(graph.findJunction(1), graph.connections[1].junction);
    EXPECT_EQ(graph.findSegment('B'), graph.connections[1].node1);
    EXPECT_EQ(Maneuver(radian_t(0), Direction::RIGHT), graph.connections[1].maneuver1);
    EXPECT_EQ(graph.findSegment('A'), graph.connections[1].node2);
    EXPECT_EQ(Maneuver(PI, Direction::CENTER), graph.connections[1].maneuver2);

    EXPECT_EQ(graph.findJunction(2), graph.connections[2].junction);
    EXPECT_EQ(graph.findSegment('C'), graph.connections[2].node1);
    EXPECT_EQ(Maneuver(PI, Direction::LEFT), graph.connections[2].maneuver1);
    EXPECT_EQ(graph.findSegment('A'), graph.connections[2].node2);
    EXPECT_EQ(Maneuver(radian_t(0), Direction::CENTER), graph.connections[2].maneuver2);

    EXPECT_EQ(graph.findJunction(2), graph.connections[3].junction);
    EXPECT_EQ(graph.findSegment('C'), graph.connections[3].node1);
    EXPECT_EQ(Maneuver(PI, Direction::RIGHT), graph.connections[3].maneuver1);
    EXPECT_EQ(graph.findSegment('A'), graph.connections[3].node2);
    EXPECT_EQ(Maneuver(radian_t(0), Direction::CENTER), graph.connections[3].maneuver2);
}
