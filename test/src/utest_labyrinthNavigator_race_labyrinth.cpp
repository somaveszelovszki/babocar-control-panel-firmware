#include <micro/math/random_generator.hpp>
#include <micro/test/utils.hpp>
#include <micro/utils/Line.hpp>
#include <micro/utils/LinePattern.hpp>
#include <micro/utils/point2.hpp>
#include <micro/utils/types.hpp>
#include <micro/utils/units.hpp>

#include <cfg_car.hpp>
#include <LabyrinthGraph.hpp>
#include <LabyrinthNavigator.hpp>
#include <track.hpp>

#include "LabyrinthNavigatorTest.hpp"

using namespace micro;

namespace {

class RaceLabyrinthNavigatorTest : public LabyrinthNavigatorTest {
public:
    RaceLabyrinthNavigatorTest() : LabyrinthNavigatorTest() {
        buildRaceLabyrinthGraph(graph_);
    }

    void SetUp() override {
        const auto* prevSeg                  = graph_.findSegment("X_");
        const auto* currentSeg               = graph_.findSegment("UX");
        const auto* laneChangeSeg            = graph_.findSegment("QV");
        const auto* junctionBeforeLaneChange = graph_.findJunction('Q');
        const auto* prevConn                 = graph_.findConnection(*prevSeg, *currentSeg);

         const SegmentIds forbiddenSegments = {
            "MP",
            "MQ",
            "NS",
            "OU",
            "PR",
            "TU",
            "UX"
        };

        navigator_.initialize(
            graph_.getVisitableSegments(),
            forbiddenSegments,
            currentSeg,
            prevConn,
            laneChangeSeg,
            junctionBeforeLaneChange,
            LABYRINTH_SPEED,
            LABYRINTH_DEAD_END_SPEED);
    }
};

TEST_F(RaceLabyrinthNavigatorTest, RandomNavigationNoObstacle) {
    moveCar(getJunctionPos('X') + point2m{centimeter_t(30), centimeter_t(0)}, meter_t(0));
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('U'), getSegmentLength("UX"));
    setNextDecision(Direction::RIGHT);
    setLines({ LinePattern::JUNCTION_1, Sign::NEGATIVE });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('O'), getSegmentLength("OU"));
    setLines({ LinePattern::JUNCTION_3, Sign::NEGATIVE, Direction::LEFT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({ LinePattern::JUNCTION_1, Sign::POSITIVE });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('L'), getSegmentLength("LO"));
    setNextDecision(Direction::CENTER);
    setLines({ LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({ LinePattern::JUNCTION_3, Sign::POSITIVE, Direction::CENTER });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('K'), getSegmentLength("KL"));
    setNextDecision(Direction::LEFT);
    setLines({ LinePattern::JUNCTION_3, Sign::NEGATIVE, Direction::CENTER });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    
    // Checks that navigating randomly into a forbidden segment is not allowed.
    moveCar(getJunctionPos('M'), getSegmentLength("KM"));
    setNextDecision(Direction::CENTER);
    setLines({ LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({ LinePattern::JUNCTION_3, Sign::POSITIVE, Direction::CENTER });
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
}

TEST_F(RaceLabyrinthNavigatorTest, ObstacleInCrossroads) {
    moveCar(getJunctionPos('X') + point2m{centimeter_t(30), centimeter_t(0)}, meter_t(0));
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('U'), getSegmentLength("UX"));
    setNextDecision(Direction::RIGHT);
    setLines({ LinePattern::JUNCTION_1, Sign::NEGATIVE });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('O'), getSegmentLength("OU"));
    setLines({ LinePattern::JUNCTION_3, Sign::NEGATIVE, Direction::LEFT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({ LinePattern::JUNCTION_1, Sign::POSITIVE });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    // Obstacle is in FG and will continue towards EG.
    // Since FG is in crossroad, DI will be added as a phantom obstacle position.
    // Therefore when the car reaches L, and by default it would navigate to the RIGHT,
    // it is forced to navigate towards either K (RIGHT) or N (LEFT).
    // Since the randomizer's next choice will be RIGHT, the car will continue towards K (RIGHT).
    navigator_.setObstaclePositions({{"FG", "EG", millisecond_t(0)}, {"DI", "IL", millisecond_t(0), true}});

    moveCar(getJunctionPos('L'), getSegmentLength("LO"));
    setNextDecision(Direction::RIGHT);
    setLines({ LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({ LinePattern::JUNCTION_3, Sign::POSITIVE, Direction::CENTER });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
}

TEST_F(RaceLabyrinthNavigatorTest, KeepSpeedSignWhenObstacleEntersCrossroad) {
    moveCar(getJunctionPos('X') + point2m{centimeter_t(30), centimeter_t(0)}, meter_t(0));
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('U'), getSegmentLength("UX"));
    setNextDecision(Direction::RIGHT);
    setLines({ LinePattern::JUNCTION_1, Sign::NEGATIVE });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('O'), getSegmentLength("OU"));
    setLines({ LinePattern::JUNCTION_3, Sign::NEGATIVE, Direction::LEFT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({ LinePattern::JUNCTION_1, Sign::POSITIVE });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('L'), getSegmentLength("LO"));
    setNextDecision(Direction::CENTER);
    setLines({ LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({ LinePattern::JUNCTION_3, Sign::POSITIVE, Direction::CENTER });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    // Obstacle is in IN and will continue towards DI.
    // Since IN is in crossroad, KL will be added as a phantom obstacle position.
    // The phantom obstacle position should be ignored and the car should continue its route with the same speed sign.
    navigator_.setObstaclePositions({{"IN", "DI", millisecond_t(1000), false}, {"KL", "JL", millisecond_t(1000), true}});

    moveCar(getJunctionPos('L') + point2m{meter_t(1), meter_t(0)}, meter_t(1));
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
}

} // namespace
