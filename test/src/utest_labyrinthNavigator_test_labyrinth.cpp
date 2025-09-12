#include "LabyrinthNavigatorTest.hpp"

#include <LabyrinthGraph.hpp>
#include <LabyrinthNavigator.hpp>
#include <cfg_car.hpp>
#include <micro/math/random_generator.hpp>
#include <micro/test/utils.hpp>
#include <micro/utils/Line.hpp>
#include <micro/utils/LinePattern.hpp>
#include <micro/utils/point2.hpp>
#include <micro/utils/types.hpp>
#include <micro/utils/units.hpp>
#include <track.hpp>

using namespace micro;

namespace {

class TestLabyrinthNavigatorTest : public LabyrinthNavigatorTest {
  public:
    TestLabyrinthNavigatorTest() : LabyrinthNavigatorTest() { buildTestLabyrinthGraph(graph_); }

    void SetUp() override {
        const auto* prevSeg                  = graph_.findSegment("Y_");
        const auto* currentSeg               = graph_.findSegment("WY");
        const auto* laneChangeSeg            = graph_.findSegment("NQ");
        const auto* junctionBeforeLaneChange = graph_.findJunction('N');
        const auto* prevConn                 = graph_.findConnection(*prevSeg, *currentSeg);

        navigator_.initialize(graph_.getVisitableSegments(), {}, currentSeg, prevConn,
                              laneChangeSeg, junctionBeforeLaneChange, LABYRINTH_SPEED,
                              LABYRINTH_DEAD_END_SPEED);
    }
};

TEST_F(TestLabyrinthNavigatorTest, RandomNavigationNoObstacle) {
    moveCar(getJunctionPos('Y'), meter_t(0));
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('W'), getSegmentLength("WY"));
    setLines({LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::LEFT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({LinePattern::JUNCTION_1, Sign::POSITIVE});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('U'), getSegmentLength("UW"));
    setNextDecision(Direction::LEFT);
    setLines({LinePattern::JUNCTION_1, Sign::NEGATIVE});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('T'), getSegmentLength("TU"));
    setNextDecision(Direction::RIGHT);
    setLines({LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::LEFT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('R'), getSegmentLength("RT"));
    setNextDecision(Direction::RIGHT);
    setLines({LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::RIGHT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('P'), getSegmentLength("PR"));
    setNextDecision(Direction::LEFT);
    setLines({LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::LEFT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('O'), getSegmentLength("OP"));
    setNextDecision(Direction::LEFT);
    setLines({LinePattern::JUNCTION_1, Sign::NEGATIVE});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('N'), getSegmentLength("NO"));
    setNextDecision(Direction::LEFT);
    setLines({LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::RIGHT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('L'), getSegmentLength("LN"));
    setNextDecision(Direction::RIGHT);
    setLines({LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::RIGHT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('J'), getSegmentLength("JL"));
    setLines({LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::RIGHT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({LinePattern::JUNCTION_1, Sign::POSITIVE});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('H'), getSegmentLength("HJ"));
    setNextDecision(Direction::LEFT);
    setLines({LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::RIGHT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('G'), getSegmentLength("GH"));
    setLines({LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::LEFT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({LinePattern::JUNCTION_1, Sign::POSITIVE});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('E'), getSegmentLength("EG"));
    setLines({LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::RIGHT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({LinePattern::JUNCTION_1, Sign::POSITIVE});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    // Checks that navigating randomly into a dead-end segment is not allowed.
    moveCar(getJunctionPos('C'), getSegmentLength("CE"));
    setNextDecision(Direction::LEFT);
    setLines({LinePattern::JUNCTION_1, Sign::NEGATIVE});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('F'), getSegmentLength("CF"));
    setNextDecision(Direction::LEFT);
    setLines({LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::RIGHT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    // Checks that navigation prefers unvisited segments.
    moveCar(getJunctionPos('H'), getSegmentLength("FH"));
    setNextDecision(Direction::RIGHT);
    setLines({LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::RIGHT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
}

TEST_F(TestLabyrinthNavigatorTest, KeepRightAvoidObstacle) {
    moveCar(getJunctionPos('Y'), meter_t(0));
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('W'), getSegmentLength("WY"));
    setLines({LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::LEFT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({LinePattern::JUNCTION_1, Sign::POSITIVE});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('U'), getSegmentLength("UW"));
    setNextDecision(Direction::RIGHT);
    setLines({LinePattern::JUNCTION_1, Sign::NEGATIVE});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('R'), getSegmentLength("RU"));
    setNextDecision(Direction::RIGHT);
    setLines({LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::LEFT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('P'), getSegmentLength("PR"));
    setNextDecision(Direction::RIGHT);
    setLines({LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::LEFT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('M'), getSegmentLength("MP"));
    setNextDecision(Direction::RIGHT);
    setLines({LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::LEFT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('K'), getSegmentLength("KM"));
    setNextDecision(Direction::RIGHT);
    setLines({LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::LEFT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('H'), getSegmentLength("HK"));
    setNextDecision(Direction::RIGHT);
    setLines({LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::LEFT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('F'), getSegmentLength("FH"));
    setNextDecision(Direction::RIGHT);
    setLines({LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::LEFT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('C'), getSegmentLength("CF"));
    setLines({LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::RIGHT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({LinePattern::JUNCTION_1, Sign::POSITIVE});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    // Obstacle is in GI and will continue towards EG.
    // Therefore when the car reaches E, and by default it would navigate to the RIGHT,
    // it is forced to navigate towards F (LEFT).
    navigator_.setObstaclePositions({{"IG", "EG", millisecond_t(0)}});

    moveCar(getJunctionPos('E'), getSegmentLength("CE"));
    setNextDecision(Direction::RIGHT);
    setLines({LinePattern::JUNCTION_1, Sign::NEGATIVE});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    // Obstacle is still in GI and will continue towards EG.
    // Therefore when the car reaches F, and by default it would navigate to the RIGHT,
    // it is forced to navigate towards H (LEFT).

    moveCar(getJunctionPos('F'), getSegmentLength("EF"));
    setNextDecision(Direction::RIGHT);
    setLines({LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::LEFT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    // Checks that navigation prefers unvisited segments.
    moveCar(getJunctionPos('H'), getSegmentLength("FH"));
    setNextDecision(Direction::LEFT);
    setLines({LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::RIGHT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
}

TEST_F(TestLabyrinthNavigatorTest, ReverseWhenEnteringRestrictedSegment) {
    moveCar(getJunctionPos('Y'), meter_t(0));
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('W'), getSegmentLength("WY"));
    setLines({LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::LEFT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({LinePattern::JUNCTION_1, Sign::POSITIVE});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('U'), getSegmentLength("UW"));
    setNextDecision(Direction::LEFT);
    setLines({LinePattern::JUNCTION_1, Sign::NEGATIVE});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('T'), getSegmentLength("TU"));
    setNextDecision(Direction::RIGHT);
    setLines({LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::LEFT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('R'), getSegmentLength("RT"));
    setNextDecision(Direction::RIGHT);
    setLines({LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::RIGHT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('P'), getSegmentLength("PR"));
    setNextDecision(Direction::LEFT);
    setLines({LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::LEFT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('O'), getSegmentLength("OP"));
    setNextDecision(Direction::LEFT);
    setLines({LinePattern::JUNCTION_1, Sign::NEGATIVE});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('N'), getSegmentLength("NO"));
    setNextDecision(Direction::LEFT);
    setLines({LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::RIGHT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    navigator_.setObstaclePositions({{"FH", "HJ", millisecond_t(0)}});

    // At this point navigating towards J is acceptable
    moveCar(getJunctionPos('L'), getSegmentLength("LN"));
    setNextDecision(Direction::RIGHT);
    setLines({LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::RIGHT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    // From J the car can only navigate towards H, which is a restricted segment.
    // Therefore the car needs to reverse once the rear line pattern is SINGLE_LINE.
    moveCar(getJunctionPos('J'), getSegmentLength("JL"));
    setLines({LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::RIGHT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({LinePattern::JUNCTION_1, Sign::POSITIVE});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(-LABYRINTH_SPEED, LINE_POS_CENTER);
}

TEST_F(TestLabyrinthNavigatorTest, ReverseWhenObstacleEntersSameSegment) {
    moveCar(getJunctionPos('Y'), meter_t(0));
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('W'), getSegmentLength("WY"));
    setLines({LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::LEFT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({LinePattern::JUNCTION_1, Sign::POSITIVE});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('U'), getSegmentLength("UW"));
    setNextDecision(Direction::RIGHT);
    setLines({LinePattern::JUNCTION_1, Sign::NEGATIVE});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('R'), getSegmentLength("RU"));
    setNextDecision(Direction::RIGHT);
    setLines({LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::LEFT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    // The obstacle comes into the same segment where the car is, from in front of the car.
    // The current segment becomes restricted.
    // The car needs to reverse its speed.
    navigator_.setObstaclePositions({{"PR", "RT", millisecond_t(0)}});

    moveCar(getJunctionPos('R') + point2m{meter_t(1), meter_t(0)}, meter_t(1));
    testUpdate(-LABYRINTH_SPEED, LINE_POS_CENTER);

    // When going back to the previous junction, it navigates towards the unvisited segment.
    moveCar(getJunctionPos('R'), meter_t(1));
    setNextDecision(Direction::LEFT);
    setLines({LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::RIGHT});
    testUpdate(-LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT});
    testUpdate(-LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(-LABYRINTH_SPEED, LINE_POS_CENTER);

    // The obstacle comes into the same segment where the car is, from behind the car.
    // The current segment does not become restricted.
    // The car needs to maintain its speed.
    navigator_.setObstaclePositions({{"RT", "TV", millisecond_t(1000)}});

    moveCar(getJunctionPos('R') + point2m{meter_t(-1), meter_t(0.5)}, meter_t(1));
    testUpdate(-LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('T'), getSegmentLength("RT"));
    setNextDecision(Direction::LEFT);
    setLines({LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::RIGHT});
    testUpdate(-LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT});
    testUpdate(-LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(-LABYRINTH_SPEED, LINE_POS_CENTER);
}

TEST_F(TestLabyrinthNavigatorTest, NavigateToLaneChange) {
    moveCar(getJunctionPos('Y'), meter_t(0));
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    // Tests if creating route causes any issues
    navigator_.navigateToLaneChange();

    moveCar(getJunctionPos('W'), getSegmentLength("WY"));
    setLines({LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::LEFT});
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({LinePattern::JUNCTION_1, Sign::POSITIVE});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({LinePattern::SINGLE_LINE, Sign::NEUTRAL});
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
}

} // namespace
