#include <micro/test/utils.hpp>

#include <LabyrinthGraph.hpp>
#include <LabyrinthRoute.hpp>
#include <track.hpp>

#include "route.hpp"

using namespace micro;

class TestLabyrinthTest : testing::Test {
public:
    TestLabyrinthTest() = default;

protected:
    LabyrinthGraph graph_{buildTestLabyrinthGraph()};
}

TEST_F(TestLabyrinthTest, valid) {
    ASSERT_TRUE(graph_.valid());
}

TEST_F(TestLabyrinthTest, AC_F) {
    checkRoute(graph_, 'A', 'C', 'F', {}, false, {
        {'C', JunctionDecision(radian_t(0), Direction::CENTER)},
        {'E', JunctionDecision(radian_t(0), Direction::LEFT)}
    });
}
