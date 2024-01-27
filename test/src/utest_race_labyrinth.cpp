#include <micro/test/utils.hpp>

#include <LabyrinthGraph.hpp>
#include <LabyrinthRoute.hpp>
#include <track.hpp>

#include "route.hpp"

using namespace micro;

class RaceLabyrinthTest : public testing::Test {
public:
    RaceLabyrinthTest() = default;

protected:
    LabyrinthGraph graph_{buildRaceLabyrinthGraph()};
};

TEST_F(RaceLabyrinthTest, valid) {
    ASSERT_TRUE(graph_.valid());
}
