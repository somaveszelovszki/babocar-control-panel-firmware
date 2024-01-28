#include <micro/math/random_generator.hpp>
#include <micro/test/utils.hpp>
#include <micro/utils/Line.hpp>
#include <micro/utils/LinePattern.hpp>
#include <micro/utils/types.hpp>
#include <micro/utils/units.hpp>

#include <cfg_car.hpp>
#include <LabyrinthNavigator.hpp>
#include <track.hpp>

using namespace micro;

namespace {

constexpr auto LABYRINTH_SPEED          = m_per_sec_t(1.0f);
constexpr auto LABYRINTH_FAST_SPEED     = m_per_sec_t(1.0f);
constexpr auto LABYRINTH_DEAD_END_SPEED = m_per_sec_t(1.0f);

constexpr auto LINE_POS_LEFT   = centimeter_t(-3.8f);
constexpr auto LINE_POS_CENTER = centimeter_t(0);
constexpr auto LINE_POS_RIGHT  = centimeter_t(3.8f);

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
    void setCar(const point2m& pos, const meter_t distance) {
        car_.pose.pos = pos;
        car_.distance = distance;
    }

    void setFrontLines(const LinePattern& pattern) {
        lineInfo_.front.pattern = pattern;
        lineInfo_.front.pattern.startDist = car_.distance;
        lineInfo_.front.lines = makeLines(pattern);
    }

    void setRearLines(const LinePattern& pattern) {
        lineInfo_.rear.pattern = pattern;
        lineInfo_.rear.pattern.startDist = car_.distance;
        lineInfo_.rear.lines = makeLines(pattern);
    }

    static Lines makeLines(const LinePattern& pattern) {
        const auto numLines = [&pattern]() -> uint8_t {
            if (pattern.type == LinePattern::NONE) {
                return 0;
            }

            if (pattern.type == LinePattern::SINGLE_LINE) {
                return 1;
            }

            if (pattern.dir != Sign::POSITIVE) {
                return 1;
            }

            return LabyrinthNavigator::numJunctionSegments(pattern);
        }();

        return [numLines]() -> Lines {
            switch (numLines) {
            case 1:
                return Lines{ { LINE_POS_CENTER, 1 } };
            case 2:
                return Lines{ { LINE_POS_LEFT, 1 }, { LINE_POS_RIGHT, 2 } };
            case 3:
                return Lines{ { LINE_POS_LEFT, 1 }, { LINE_POS_CENTER, 2 }, { LINE_POS_RIGHT, 3 } };
            case 0:
            default:
                return {};
            }
        }();
    }

    void testUpdate(const m_per_sec_t speed, const Direction targetLineDir) {
        micro::updateMainLine(lineInfo_.front.lines, lineInfo_.rear.lines, mainLine_);
        navigator_.update(car_, lineInfo_, mainLine_, controlData_);
        car_.speed = controlData_.speed;
        const auto targetLinePos = [targetLineDir](){
            switch (targetLineDir) {
            case Direction::LEFT:
                return LINE_POS_LEFT;

            case Direction::RIGHT:
                return LINE_POS_RIGHT;

            case Direction::CENTER:
            default:
                return LINE_POS_CENTER;
            }
        }();

        EXPECT_NEAR_UNIT_DEFAULT(speed, car_.speed);
        EXPECT_NEAR_UNIT_DEFAULT(targetLinePos, controlData_.lineControl.target.pos);
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
    setCar({meter_t(0), meter_t(0)}, meter_t(0));
    setFrontLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER });
    setRearLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER });
    testUpdate(LABYRINTH_SPEED, Direction::CENTER);
}

} // namespace