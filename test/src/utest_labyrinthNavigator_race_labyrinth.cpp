#include "micro/utils/point2.hpp"
#include <micro/math/random_generator.hpp>
#include <micro/test/utils.hpp>
#include <micro/utils/Line.hpp>
#include <micro/utils/LinePattern.hpp>
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

        const auto* prevSeg                  = graph_.findSegment("X_");
        const auto* currentSeg               = graph_.findSegment("UX");
        const auto* laneChangeSeg            = graph_.findSegment("QV");
        const auto* junctionBeforeLaneChange = graph_.findJunction('Q');
        const auto* prevConn                 = graph_.findConnection(*prevSeg, *currentSeg);

        navigator_.initialize(
            graph_.getVisitableSegments(),
            {},
            currentSeg,
            prevConn,
            laneChangeSeg,
            junctionBeforeLaneChange,
            LABYRINTH_SPEED,
            LABYRINTH_DEAD_END_SPEED);
    }
};

TEST_F(RaceLabyrinthNavigatorTest, RandomNavigationNoObstacle) {
    // moveCar(getJunctionPos('Y'), meter_t(0));
    // setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    // testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    // moveCar(getJunctionPos('W'), getSegmentLength("WY"));
    // setLines({ LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::LEFT });
    // testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    // setLines({ LinePattern::JUNCTION_1, Sign::POSITIVE });
    // testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    // setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    // testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
}

} // namespace
