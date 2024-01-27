#include <micro/test/utils.hpp>

#include <LabyrinthGraph.hpp>
#include <LabyrinthRoute.hpp>
#include <track.hpp>

#include "route.hpp"

using namespace micro;

class RaceLabyrinthTest : public testing::Test {
public:
    RaceLabyrinthTest() {
        buildRaceLabyrinthGraph(graph_);
    }

protected:
    LabyrinthGraph graph_;
};

TEST_F(RaceLabyrinthTest, valid) {
    ASSERT_TRUE(graph_.valid());
}
