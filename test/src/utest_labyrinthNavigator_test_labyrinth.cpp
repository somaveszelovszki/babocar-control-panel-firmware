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

using namespace micro;

namespace {

constexpr auto LABYRINTH_SPEED          = m_per_sec_t(1.0f);
constexpr auto LABYRINTH_DEAD_END_SPEED = m_per_sec_t(1.0f);

constexpr auto LINE_POS_LEFT   = centimeter_t(-3.8f);
constexpr auto LINE_POS_CENTER = centimeter_t(0);
constexpr auto LINE_POS_RIGHT  = centimeter_t(3.8f);

class LabyrinthNavigatorTest : public ::testing::Test {
public:
    LabyrinthNavigatorTest() {
        buildTestLabyrinthGraph(graph_);
        const auto* prevSeg                  = graph_.findSegment("QS");
        const auto* currentSeg               = graph_.findSegment("SV");
        const auto* laneChangeSeg            = graph_.findSegment("NQ");
        const auto* junctionBeforeLaneChange = graph_.findJunction('N');
        const auto* prevConn                 = graph_.findConnection(*prevSeg, *currentSeg);

        navigator_.initialize(
            graph_.getVisitableSegments(),
            currentSeg,
            prevConn,
            laneChangeSeg,
            junctionBeforeLaneChange,
            LABYRINTH_SPEED,
            LABYRINTH_DEAD_END_SPEED);
    }

protected:
    void moveCar(const point2m& pos, const meter_t distance) {
        car_.pose.pos = pos;
        car_.distance += distance;
    }

    void setNextDecision(const Direction dir) {
        random_.setValue(dir == Direction::RIGHT ? 0.0f : 0.9999f);
    }

    void setLines(const LinePattern& pattern) {
        auto& front = car_.speed >= micro::m_per_sec_t(0) ? lineInfo_.front : lineInfo_.rear;
        auto& rear = car_.speed >= micro::m_per_sec_t(0) ? lineInfo_.rear : lineInfo_.front;
        
        front.pattern = pattern;
        front.pattern.startDist = car_.distance;
        front.lines = makeLines(pattern);

        rear.pattern = front.pattern;

        rear.lines.clear();
        for (auto line : front.lines) {
            line.pos = -line.pos;
            rear.lines.insert(line);
        }
        
        micro::updateMainLine(lineInfo_.front.lines, lineInfo_.rear.lines, mainLine_);
    }

    Lines makeLines(const LinePattern& pattern) const {
        const auto id = [this](const uint8_t incr = 0) { return static_cast<uint8_t>(mainLine_.frontLine.id + incr); };

        switch (pattern.type) {
        case LinePattern::SINGLE_LINE:
        case LinePattern::JUNCTION_1:
            return { { LINE_POS_CENTER, id() } };

        case LinePattern::JUNCTION_2:
            switch (pattern.side) {
            case Direction::LEFT:
                return { { LINE_POS_LEFT, id(1) }, { LINE_POS_RIGHT, id() } };

            case Direction::RIGHT:
                return { { LINE_POS_LEFT, id() }, { LINE_POS_RIGHT, id(1) } };

            default:
                return {};
            }

        case LinePattern::JUNCTION_3:
            switch (pattern.side) {
            case Direction::LEFT:
                return { { LINE_POS_LEFT, id(1) }, { LINE_POS_CENTER, id(2) }, { LINE_POS_RIGHT, id() } };

            case Direction::CENTER:
                return { { LINE_POS_LEFT, id(1) }, { LINE_POS_CENTER, id() }, { LINE_POS_RIGHT, id(2) } };

            case Direction::RIGHT:
                return { { LINE_POS_LEFT, id() }, { LINE_POS_CENTER, id(1) }, { LINE_POS_RIGHT, id(2) } };

            default:
                return {};
            }

        case LinePattern::NONE:
        default:
            return {};        
        }
    }

    void testUpdate(const m_per_sec_t speed, const centimeter_t targetLinePos) {
        navigator_.update(car_, lineInfo_, mainLine_, controlData_);
        car_.speed = controlData_.speed;
        EXPECT_NEAR_UNIT_DEFAULT(speed, car_.speed);
        EXPECT_NEAR_UNIT_DEFAULT(targetLinePos * micro::sgn(car_.speed), controlData_.lineControl.actual.pos);
    }

    point2m getJunctionPos(const char junctionId) const {
        return graph_.findJunction(junctionId)->pos;
    }

    meter_t getSegmentLength(const Segment::Id& segmentId) {
        return graph_.findSegment(segmentId)->length;
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

TEST_F(LabyrinthNavigatorTest, RandomNavigationNoObstacle) {
    moveCar(getJunctionPos('Y'), meter_t(0));
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('W'), getSegmentLength("WY"));
    setLines({ LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::LEFT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({ LinePattern::JUNCTION_1, Sign::POSITIVE });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('U'), getSegmentLength("UW"));
    setNextDecision(Direction::LEFT);
    setLines({ LinePattern::JUNCTION_1, Sign::NEGATIVE });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('T'), getSegmentLength("TU"));
    setNextDecision(Direction::RIGHT);
    setLines({ LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::LEFT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('R'), getSegmentLength("RT"));
    setNextDecision(Direction::RIGHT);
    setLines({ LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('P'), getSegmentLength("PR"));
    setNextDecision(Direction::LEFT);
    setLines({ LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::LEFT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('O'), getSegmentLength("OP"));
    setNextDecision(Direction::LEFT);
    setLines({ LinePattern::JUNCTION_1, Sign::NEGATIVE });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('N'), getSegmentLength("NO"));
    setNextDecision(Direction::LEFT);
    setLines({ LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('L'), getSegmentLength("LN"));
    setNextDecision(Direction::RIGHT);
    setLines({ LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('J'), getSegmentLength("JL"));
    setLines({ LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({ LinePattern::JUNCTION_1, Sign::POSITIVE });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('H'), getSegmentLength("HJ"));
    setNextDecision(Direction::LEFT);
    setLines({ LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('G'), getSegmentLength("GH"));
    setLines({ LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::LEFT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({ LinePattern::JUNCTION_1, Sign::POSITIVE });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('E'), getSegmentLength("EG"));
    setLines({ LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({ LinePattern::JUNCTION_1, Sign::POSITIVE });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    // Checks that navigating randomly into a dead-end segment is not allowed.
    moveCar(getJunctionPos('C'), getSegmentLength("CE"));
    setNextDecision(Direction::LEFT);
    setLines({ LinePattern::JUNCTION_1, Sign::NEGATIVE });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('F'), getSegmentLength("CF"));
    setNextDecision(Direction::LEFT);
    setLines({ LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    // Checks that navigation prefers unvisited segments.
    moveCar(getJunctionPos('H'), getSegmentLength("FH"));
    setNextDecision(Direction::RIGHT);
    setLines({ LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
}

TEST_F(LabyrinthNavigatorTest, KeepRightAvoidObstacle) {
    moveCar(getJunctionPos('Y'), meter_t(0));
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('W'), getSegmentLength("WY"));
    setLines({ LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::LEFT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({ LinePattern::JUNCTION_1, Sign::POSITIVE });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('U'), getSegmentLength("UW"));
    setNextDecision(Direction::RIGHT);
    setLines({ LinePattern::JUNCTION_1, Sign::NEGATIVE });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('R'), getSegmentLength("RU"));
    setNextDecision(Direction::RIGHT);
    setLines({ LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::LEFT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('P'), getSegmentLength("PR"));
    setNextDecision(Direction::RIGHT);
    setLines({ LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::LEFT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('M'), getSegmentLength("MP"));
    setNextDecision(Direction::RIGHT);
    setLines({ LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::LEFT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('K'), getSegmentLength("KM"));
    setNextDecision(Direction::RIGHT);
    setLines({ LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::LEFT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('H'), getSegmentLength("HK"));
    setNextDecision(Direction::RIGHT);
    setLines({ LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::LEFT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('F'), getSegmentLength("FH"));
    setNextDecision(Direction::RIGHT);
    setLines({ LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::LEFT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('C'), getSegmentLength("CF"));
    setLines({ LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({ LinePattern::JUNCTION_1, Sign::POSITIVE });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    // Marks G and I as restricted.
    // Therefore when the car reaches E, and by default it would navigate to the RIGHT,
    // it is forced to navigate towards F (LEFT).
    navigator_.setObstaclePosition({
        graph_.findSegment(Segment::makeId('G', 'I')),
        graph_.findSegment(Segment::makeId('G', 'E')),
        millisecond_t(0)
    });

    moveCar(getJunctionPos('E'), getSegmentLength("CE"));
    setNextDecision(Direction::RIGHT);
    setLines({ LinePattern::JUNCTION_1, Sign::NEGATIVE });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    // G and I are still restricted.
    // Therefore when the car reaches F, and by default it would navigate to the RIGHT,
    // it is forced to navigate towards H (LEFT).

    moveCar(getJunctionPos('F'), getSegmentLength("EF"));
    setNextDecision(Direction::RIGHT);
    setLines({ LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::LEFT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    // Checks that navigation prefers unvisited segments.
    moveCar(getJunctionPos('H'), getSegmentLength("FH"));
    setNextDecision(Direction::LEFT);
    setLines({ LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
}

TEST_F(LabyrinthNavigatorTest, ReverseWhenEnteringRestrictedSegment) {
    moveCar(getJunctionPos('Y'), meter_t(0));
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('W'), getSegmentLength("WY"));
    setLines({ LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::LEFT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({ LinePattern::JUNCTION_1, Sign::POSITIVE });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('U'), getSegmentLength("UW"));
    setNextDecision(Direction::LEFT);
    setLines({ LinePattern::JUNCTION_1, Sign::NEGATIVE });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('T'), getSegmentLength("TU"));
    setNextDecision(Direction::RIGHT);
    setLines({ LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::LEFT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('R'), getSegmentLength("RT"));
    setNextDecision(Direction::RIGHT);
    setLines({ LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('P'), getSegmentLength("PR"));
    setNextDecision(Direction::LEFT);
    setLines({ LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::LEFT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('O'), getSegmentLength("OP"));
    setNextDecision(Direction::LEFT);
    setLines({ LinePattern::JUNCTION_1, Sign::NEGATIVE });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('N'), getSegmentLength("NO"));
    setNextDecision(Direction::LEFT);
    setLines({ LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    navigator_.setObstaclePosition({
        graph_.findSegment(Segment::makeId('H', 'F')),
        graph_.findSegment(Segment::makeId('H', 'J')),
        millisecond_t(0)
    });

    // At this point navigating towards J is acceptable
    moveCar(getJunctionPos('L'), getSegmentLength("LN"));
    setNextDecision(Direction::RIGHT);
    setLines({ LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    // From J the car can only navigate towards H, which is a restricted segment.
    // Therefore the car needs to reverse once the rear line pattern is SINGLE_LINE.
    moveCar(getJunctionPos('J'), getSegmentLength("JL"));
    setLines({ LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({ LinePattern::JUNCTION_1, Sign::POSITIVE });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(-LABYRINTH_SPEED, LINE_POS_CENTER);
}

TEST_F(LabyrinthNavigatorTest, ReverseWhenSegmentBecomesRestricted) {
    moveCar(getJunctionPos('Y'), meter_t(0));
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('W'), getSegmentLength("WY"));
    setLines({ LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::LEFT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({ LinePattern::JUNCTION_1, Sign::POSITIVE });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('U'), getSegmentLength("UW"));
    setNextDecision(Direction::RIGHT);
    setLines({ LinePattern::JUNCTION_1, Sign::NEGATIVE });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('R'), getSegmentLength("RU"));
    setNextDecision(Direction::RIGHT);
    setLines({ LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::LEFT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    // The obstacle comes into the same segment where the car is, from in front of the car.
    // The current segment becomes restricted.
    // The car needs to reverse its speed.
    navigator_.setObstaclePosition({
        graph_.findSegment(Segment::makeId('P', 'R')),
        graph_.findSegment(Segment::makeId('R', 'T')),
        millisecond_t(0)
    });

    moveCar(getJunctionPos('R') + point2m{meter_t(1), meter_t(0)}, meter_t(1));
    testUpdate(-LABYRINTH_SPEED, LINE_POS_CENTER);

    // When going back to the previous junction, it navigates towards the unvisited segment.
    moveCar(getJunctionPos('R'), meter_t(1));
    setNextDecision(Direction::LEFT);
    setLines({ LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::RIGHT });
    testUpdate(-LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT });
    testUpdate(-LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(-LABYRINTH_SPEED, LINE_POS_CENTER);

    // The obstacle comes into the same segment where the car is, from behind the car.
    // The current segment does not become restricted.
    // The car needs to maintain its speed.
    navigator_.setObstaclePosition({
        graph_.findSegment(Segment::makeId('R', 'T')),
        graph_.findSegment(Segment::makeId('T', 'V')),
        millisecond_t(1000)
    });

    moveCar(getJunctionPos('R') + point2m{meter_t(-1), meter_t(0.5)}, meter_t(1));
    testUpdate(-LABYRINTH_SPEED, LINE_POS_CENTER);

    moveCar(getJunctionPos('T'), getSegmentLength("RT"));
    setNextDecision(Direction::LEFT);
    setLines({ LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::RIGHT });
    testUpdate(-LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT });
    testUpdate(-LABYRINTH_SPEED, LINE_POS_LEFT);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(-LABYRINTH_SPEED, LINE_POS_CENTER);
}

TEST_F(LabyrinthNavigatorTest, NavigateToLaneChange) {
    moveCar(getJunctionPos('Y'), meter_t(0));
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);

    // Tests if creating route causes any issues
    navigator_.navigateToLaneChange();

    moveCar(getJunctionPos('W'), getSegmentLength("WY"));
    setLines({ LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::LEFT });
    testUpdate(LABYRINTH_SPEED, LINE_POS_RIGHT);
    setLines({ LinePattern::JUNCTION_1, Sign::POSITIVE });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
    setLines({ LinePattern::SINGLE_LINE, Sign::NEUTRAL });
    testUpdate(LABYRINTH_SPEED, LINE_POS_CENTER);
}

} // namespace
