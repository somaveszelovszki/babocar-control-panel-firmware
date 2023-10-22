#include <micro/test/utils.hpp>

#define private public
#include <LabyrinthGraph.hpp>
#undef private

using namespace micro;

namespace {

LabyrinthGraph createGraph() {
    LabyrinthGraph graph;

    graph.addSegment(Segment('A', meter_t(4), false));
    graph.addSegment(Segment('B', meter_t(4), false));
    graph.addSegment(Segment('C', meter_t(4), false));

    graph.addJunction(Junction(1, { meter_t(2),  meter_t(0) }));
    graph.addJunction(Junction(2, { meter_t(-2), meter_t(0) }));

    graph.connect(graph.findSegment('A'), graph.findJunction(1), JunctionDecision(PI,          Direction::CENTER));
    graph.connect(graph.findSegment('B'), graph.findJunction(1), JunctionDecision(radian_t(0), Direction::LEFT));
    graph.connect(graph.findSegment('B'), graph.findJunction(1), JunctionDecision(radian_t(0), Direction::RIGHT));

    graph.connect(graph.findSegment('A'), graph.findJunction(2), JunctionDecision(radian_t(0), Direction::CENTER));
    graph.connect(graph.findSegment('C'), graph.findJunction(2), JunctionDecision(PI,          Direction::LEFT));
    graph.connect(graph.findSegment('C'), graph.findJunction(2), JunctionDecision(PI,          Direction::RIGHT));

    return graph;
}

} // namespace

TEST(labyrinthGraph, segments) {
    LabyrinthGraph graph = createGraph();

    const auto* segA = graph.findSegment('A');
    ASSERT_NE(nullptr, segA);
    EXPECT_EQ('A', segA->name);

    const auto* segB = graph.findSegment('B');
    ASSERT_NE(nullptr, segB);
    EXPECT_EQ('B', segB->name);

    const auto* segC = graph.findSegment('C');
    ASSERT_NE(nullptr, segC);
    EXPECT_EQ('C', segC->name);
}

TEST(labyrinthGraph, edges) {
    LabyrinthGraph graph = createGraph();

    const auto* segA = graph.findSegment('A');
    ASSERT_NE(nullptr, segA);
    ASSERT_EQ(4, segA->edges.size());
    EXPECT_EQ(graph.findSegment('B'), segA->edges[0]->getOtherSegment(*segA));
    EXPECT_EQ(graph.findSegment('B'), segA->edges[1]->getOtherSegment(*segA));
    EXPECT_EQ(graph.findSegment('C'), segA->edges[2]->getOtherSegment(*segA));
    EXPECT_EQ(graph.findSegment('C'), segA->edges[3]->getOtherSegment(*segA));

    const auto* segB = graph.findSegment('B');
    ASSERT_NE(nullptr, segB);
    ASSERT_EQ(2, segB->edges.size());
    EXPECT_EQ(graph.findSegment('A'), segB->edges[0]->getOtherSegment(*segB));
    EXPECT_EQ(graph.findSegment('A'), segB->edges[1]->getOtherSegment(*segB));

    const auto* segC = graph.findSegment('C');
    ASSERT_NE(nullptr, segC);
    ASSERT_EQ(2, segC->edges.size());
    EXPECT_EQ(graph.findSegment('A'), segC->edges[0]->getOtherSegment(*segC));
    EXPECT_EQ(graph.findSegment('A'), segC->edges[1]->getOtherSegment(*segC));
}

TEST(labyrinthGraph, junctions) {
    LabyrinthGraph graph = createGraph();

    const auto* j1 = graph.findJunction(1);
    ASSERT_NE(nullptr, j1);
    ASSERT_EQ(2, j1->segments.size());

    const auto j1_rightSideSegs = j1->getSideSegments(radian_t(0));
    ASSERT_NE(nullptr, j1_rightSideSegs);
    ASSERT_EQ(2, j1_rightSideSegs->size());
    EXPECT_EQ(graph.findSegment('B'), j1_rightSideSegs->at(Direction::LEFT));
    EXPECT_EQ(graph.findSegment('B'), j1_rightSideSegs->at(Direction::RIGHT));

    const auto j1_leftSideSegs = j1->getSideSegments(PI);
    ASSERT_NE(nullptr, j1_leftSideSegs);
    ASSERT_EQ(1, j1_leftSideSegs->size());
    EXPECT_EQ(graph.findSegment('A'), j1_leftSideSegs->at(Direction::CENTER));

    const auto* j2 = graph.findJunction(2);
    ASSERT_NE(nullptr, j2);
    ASSERT_EQ(2, j2->segments.size());

    const auto j2_rightSideSegs = j2->getSideSegments(radian_t(0));
    ASSERT_NE(nullptr, j2_rightSideSegs);
    ASSERT_EQ(1, j2_rightSideSegs->size());
    EXPECT_EQ(graph.findSegment('A'), j2_rightSideSegs->at(Direction::CENTER));

    const auto j2_leftSideSegs = j2->getSideSegments(PI);
    ASSERT_NE(nullptr, j2_leftSideSegs);
    ASSERT_EQ(2, j2_leftSideSegs->size());
    EXPECT_EQ(graph.findSegment('C'), j2_leftSideSegs->at(Direction::LEFT));
    EXPECT_EQ(graph.findSegment('C'), j2_leftSideSegs->at(Direction::RIGHT));
}

TEST(labyrinthGraph, connections) {
    LabyrinthGraph graph = createGraph();

    ASSERT_EQ(4, graph.connections_.size());

    const auto* conn0 = graph.findConnection(*graph.findSegment('A'), *graph.findSegment('B'));
    const auto* conn1 = graph.findConnection(*graph.findSegment('A'), *graph.findSegment('B'));
    const auto* conn2 = graph.findConnection(*graph.findSegment('A'), *graph.findSegment('B'));
    const auto* conn3 = graph.findConnection(*graph.findSegment('A'), *graph.findSegment('B'));

    EXPECT_EQ(graph.findJunction(1), graph.connections_[0].junction);
    EXPECT_EQ(graph.findSegment('B'), graph.connections_[0].node1);
    EXPECT_EQ(JunctionDecision(radian_t(0), Direction::LEFT), graph.connections_[0].decision1);
    EXPECT_EQ(graph.findSegment('A'), graph.connections_[0].node2);
    EXPECT_EQ(JunctionDecision(PI, Direction::CENTER), graph.connections_[0].decision2);

    EXPECT_EQ(graph.findJunction(1), graph.connections_[1].junction);
    EXPECT_EQ(graph.findSegment('B'), graph.connections_[1].node1);
    EXPECT_EQ(JunctionDecision(radian_t(0), Direction::RIGHT), graph.connections_[1].decision1);
    EXPECT_EQ(graph.findSegment('A'), graph.connections_[1].node2);
    EXPECT_EQ(JunctionDecision(PI, Direction::CENTER), graph.connections_[1].decision2);

    EXPECT_EQ(graph.findJunction(2), graph.connections_[2].junction);
    EXPECT_EQ(graph.findSegment('C'), graph.connections_[2].node1);
    EXPECT_EQ(JunctionDecision(PI, Direction::LEFT), graph.connections_[2].decision1);
    EXPECT_EQ(graph.findSegment('A'), graph.connections_[2].node2);
    EXPECT_EQ(JunctionDecision(radian_t(0), Direction::CENTER), graph.connections_[2].decision2);

    EXPECT_EQ(graph.findJunction(2), graph.connections_[3].junction);
    EXPECT_EQ(graph.findSegment('C'), graph.connections_[3].node1);
    EXPECT_EQ(JunctionDecision(PI, Direction::RIGHT), graph.connections_[3].decision1);
    EXPECT_EQ(graph.findSegment('A'), graph.connections_[3].node2);
    EXPECT_EQ(JunctionDecision(radian_t(0), Direction::CENTER), graph.connections_[3].decision2);
}
