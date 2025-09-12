#include <LabyrinthGraph.hpp>
#include <LabyrinthRoute.hpp>
#include <track.hpp>

#include <micro/test/utils.hpp>

#include "route.hpp"

using namespace micro;

class TestLabyrinthTest : public testing::Test {
  public:
    TestLabyrinthTest() { buildTestLabyrinthGraph(graph_); }

  protected:
    LabyrinthGraph graph_;
};

TEST_F(TestLabyrinthTest, valid) {
    ASSERT_TRUE(graph_.valid());
}

TEST_F(TestLabyrinthTest, F_CF_FH_F_backbardNavigationDisabled) {
    checkRoute(graph_, 'F', "CF", "FH", 'F', {}, false,
               {{'C', {PI, Direction::CENTER}},
                {'E', {PI, Direction::LEFT}},
                {'F', {PI, Direction::LEFT}}});
}

TEST_F(TestLabyrinthTest, T_RT_VX_V_backbardNavigationDisabled) {
    checkRoute(graph_, 'T', "RT", "VX", 'V', {}, false,
               {{'R', {radian_t(0), Direction::RIGHT}},
                {'P', {radian_t(0), Direction::RIGHT}},
                {'M', {radian_t(0), Direction::RIGHT}},
                {'K', {radian_t(0), Direction::RIGHT}},
                {'H', {radian_t(0), Direction::RIGHT}},
                {'F', {radian_t(0), Direction::RIGHT}},
                {'C', {PI, Direction::CENTER}},
                {'E', {PI, Direction::RIGHT}},
                {'G', {PI, Direction::RIGHT}},
                {'I', {PI, Direction::CENTER}},
                {'L', {PI, Direction::RIGHT}},
                {'N', {PI, Direction::RIGHT}},
                {'Q', {PI, Direction::RIGHT}},
                {'S', {PI, Direction::RIGHT}},
                {'V', {PI, Direction::CENTER}}});
}

TEST_F(TestLabyrinthTest, T_RT_VX_V_backbardNavigationEnabled) {
    checkRoute(graph_, 'T', "RT", "VX", 'V', {}, true,
               {{'T', {PI, Direction::RIGHT}}, {'V', {PI, Direction::CENTER}}});
}
