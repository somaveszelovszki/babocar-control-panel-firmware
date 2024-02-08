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

TEST_F(RaceLabyrinthTest, X_UX_AF_F_backbardNavigationDisabled) {
    checkRoute(graph_, 'X', "UX", "AF", 'F', {}, false, {
        {'U', {PI_2, Direction::RIGHT}},
        {'O', {radian_t(0), Direction::CENTER}},
        {'L', {PI_2, Direction::RIGHT}},
        {'I', {radian_t(0), Direction::LEFT}},
        {'F', {PI_2, Direction::RIGHT}}
    });
}

TEST_F(RaceLabyrinthTest, L_JL_QV_Q_backbardNavigationDisabled) {
    checkRoute(graph_, 'L', "JL", "QV", 'Q', {}, false, {
        {'J', {radian_t(0), Direction::LEFT}},
        {'G', {PI_2, Direction::LEFT}},
        {'I', {PI, Direction::RIGHT}},
        {'K', {PI_2, Direction::LEFT}},
        {'M', {PI, Direction::CENTER}},
        {'Q', {PI, Direction::LEFT}}
    });
}