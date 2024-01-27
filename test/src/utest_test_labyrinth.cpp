#include <micro/test/utils.hpp>

#include <LabyrinthGraph.hpp>
#include <LabyrinthRoute.hpp>
#include <track.hpp>

#include "route.hpp"

using namespace micro;

class TestLabyrinthTest : public testing::Test {
public:
    TestLabyrinthTest() {
        buildTestLabyrinthGraph(graph_);
    }

protected:
    LabyrinthGraph graph_;
};

TEST_F(TestLabyrinthTest, valid) {
    ASSERT_TRUE(graph_.valid());
}

TEST_F(TestLabyrinthTest, A_AC_FH) {
    checkRoute(graph_, 'A', "AC", "FH", false, {
        {'C', {radian_t(0), Direction::CENTER}},
        {'E', {radian_t(0), Direction::LEFT}},
        {'F', {radian_t(0), Direction::LEFT}}
    });
}
