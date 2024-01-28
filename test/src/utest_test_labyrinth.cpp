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

TEST_F(TestLabyrinthTest, F_CF_FH_backbardNavigationDisabled) {
    checkRoute(graph_, 'F', "CF", {"FH"}, {}, {}, false, {
        {'C', {radian_t(0), Direction::CENTER}},
        {'E', {radian_t(0), Direction::LEFT}},
        {'F', {radian_t(0), Direction::LEFT}}
    });
}

TEST_F(TestLabyrinthTest, T_RT_VX_backbardNavigationDisabled) {
    checkRoute(graph_, 'T', "RT", {"VX"}, {}, {}, false, {
        {'R', {PI, Direction::RIGHT}},
        {'P', {PI, Direction::RIGHT}},
        {'M', {PI, Direction::RIGHT}},
        {'K', {PI, Direction::RIGHT}},
        {'H', {PI, Direction::RIGHT}},
        {'F', {PI, Direction::RIGHT}},
        {'C', {radian_t(0), Direction::CENTER}},
        {'E', {radian_t(0), Direction::RIGHT}},
        {'G', {radian_t(0), Direction::RIGHT}},
        {'I', {radian_t(0), Direction::CENTER}},
        {'L', {radian_t(0), Direction::RIGHT}},
        {'N', {radian_t(0), Direction::RIGHT}},
        {'Q', {radian_t(0), Direction::RIGHT}},
        {'S', {radian_t(0), Direction::RIGHT}},
        {'V', {radian_t(0), Direction::CENTER}}
    });
}

TEST_F(TestLabyrinthTest, T_RT_VX_backbardNavigationEnabled) {
    checkRoute(graph_, 'T', "RT", {"VX"}, {}, {}, true, {
        {'T', {radian_t(0), Direction::RIGHT}},
        {'V', {radian_t(0), Direction::CENTER}}
    });
}
