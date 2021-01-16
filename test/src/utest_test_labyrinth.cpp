#include <micro/test/utils.hpp>

#include <LabyrinthGraph.hpp>
#include <LabyrinthRoute.hpp>
#include <track.hpp>

using namespace micro;

TEST(labyrinthGraph, valid) {
    LabyrinthGraph graph = buildTestLabyrinthGraph();
    ASSERT_TRUE(graph.valid());
}
