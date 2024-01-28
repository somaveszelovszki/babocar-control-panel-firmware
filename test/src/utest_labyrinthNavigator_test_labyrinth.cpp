#include "micro/math/random_generator.hpp"
#include <micro/test/utils.hpp>

#include <cfg_car.hpp>
#include <LabyrinthNavigator.hpp>
#include <track.hpp>

using namespace micro;

namespace {

m_per_sec_t LABYRINTH_SPEED          = m_per_sec_t(1.0f);
m_per_sec_t LABYRINTH_FAST_SPEED     = m_per_sec_t(1.0f);
m_per_sec_t LABYRINTH_DEAD_END_SPEED = m_per_sec_t(1.0f);

class LabyrinthNavigatorTest : public ::testing::Test {
public:
    LabyrinthNavigatorTest() {
        buildTestLabyrinthGraph(graph_);
        const auto* currentSeg = graph_.findSegment("WY");
        const auto* prevConn = graph_.findConnection("Y_", "WY");
        const auto* laneChangeSeg = graph_.findSegment("NQ");
        navigator_.initialize(graph_.getVisitableSegments(), currentSeg, prevConn, laneChangeSeg, LABYRINTH_SPEED, LABYRINTH_FAST_SPEED, LABYRINTH_DEAD_END_SPEED);
    }

protected:
    LabyrinthGraph graph_;
    micro::fixed_number_generator random_{0.0f};
    LabyrinthNavigator navigator_{graph_, random_};
    CarProps car_;
    LineInfo lineInfo_;
    MainLine mainLine_{cfg::CAR_FRONT_REAR_SENSOR_ROW_DIST};
    ControlData controlData_{LABYRINTH_SPEED, millisecond_t(500), false, {}};
};

TEST_F(LabyrinthNavigatorTest, KeepRight) {
    // Sets up initial lines
    lineInfo_.front.lines   = { { centimeter_t(0), 1 } };
    lineInfo_.front.pattern = { LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER, meter_t(0) };
    lineInfo_.rear.lines    = { { centimeter_t(0), 1 } };
    lineInfo_.rear.pattern  = { LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER, meter_t(0) };

    micro::updateMainLine(lineInfo_.front.lines, lineInfo_.rear.lines, mainLine_);
    navigator_.update(car_, lineInfo_, mainLine_, controlData_);
    car_.speed = controlData_.speed;





    car_.pose.pos   = { centimeter_t(330), meter_t(0) };
    car_.pose.angle = radian_t(0);
    car_.distance   = centimeter_t(330);

    lineInfo_.front.lines   = { { centimeter_t(0), 1 }, { centimeter_t(3.8f), 2 } };
    lineInfo_.front.pattern = { LinePattern::JUNCTION_1, Sign::NEGATIVE, Direction::CENTER, centimeter_t(300) };
    lineInfo_.rear.lines    = { { centimeter_t(0), 1 } };
    lineInfo_.rear.pattern  = { LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER, meter_t(0) };

    micro::updateMainLine(lineInfo_.front.lines, lineInfo_.rear.lines, mainLine_);
    navigator_.update(car_, lineInfo_, mainLine_, controlData_);
    car_.speed = controlData_.speed;
}

} // namespace