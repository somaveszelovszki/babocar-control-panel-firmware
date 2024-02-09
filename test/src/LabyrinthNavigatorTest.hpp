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

namespace {

constexpr auto LABYRINTH_SPEED          = micro::m_per_sec_t(1.0f);
constexpr auto LABYRINTH_DEAD_END_SPEED = micro::m_per_sec_t(1.0f);

constexpr auto LINE_POS_LEFT   = micro::centimeter_t(-3.8f);
constexpr auto LINE_POS_CENTER = micro::centimeter_t(0);
constexpr auto LINE_POS_RIGHT  = micro::centimeter_t(3.8f);

class LabyrinthNavigatorTest : public ::testing::Test {
protected:
    void moveCar(const micro::point2m& pos, const micro::meter_t distance) {
        car_.pose.pos = pos;
        car_.distance += distance;
    }

    void setNextDecision(const micro::Direction dir) {
        const auto value = [dir]() {
            switch (dir) {
            case micro::Direction::LEFT:
                return 0.999999f;

            case micro::Direction::CENTER:
                return 0.5f;

            case micro::Direction::RIGHT:
                return 0.0f;

            default:
                return 0.0f;
            }
        }();
        random_.setValue(value);
    }

    void setLines(const micro::LinePattern& pattern) {
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

    micro::Lines makeLines(const micro::LinePattern& pattern) const {
        const auto id = [this](const uint8_t incr = 0) { return static_cast<uint8_t>(mainLine_.frontLine.id + incr); };

        switch (pattern.type) {
        case micro::LinePattern::SINGLE_LINE:
        case micro::LinePattern::JUNCTION_1:
            return { { LINE_POS_CENTER, id() } };

        case micro::LinePattern::JUNCTION_2:
            switch (pattern.side) {
            case micro::Direction::LEFT:
                return { { LINE_POS_LEFT, id(1) }, { LINE_POS_RIGHT, id() } };

            case micro::Direction::RIGHT:
                return { { LINE_POS_LEFT, id() }, { LINE_POS_RIGHT, id(1) } };

            default:
                return {};
            }

        case micro::LinePattern::JUNCTION_3:
            switch (pattern.side) {
            case micro::Direction::LEFT:
                return { { LINE_POS_LEFT, id(1) }, { LINE_POS_CENTER, id(2) }, { LINE_POS_RIGHT, id() } };

            case micro::Direction::CENTER:
                return { { LINE_POS_LEFT, id(1) }, { LINE_POS_CENTER, id() }, { LINE_POS_RIGHT, id(2) } };

            case micro::Direction::RIGHT:
                return { { LINE_POS_LEFT, id() }, { LINE_POS_CENTER, id(1) }, { LINE_POS_RIGHT, id(2) } };

            default:
                return {};
            }

        case micro::LinePattern::NONE:
        default:
            return {};        
        }
    }

    void testUpdate(const micro::m_per_sec_t speed, const micro::centimeter_t targetLinePos) {
        navigator_.update(car_, lineInfo_, mainLine_, controlData_);
        car_.speed = controlData_.speed;
        EXPECT_NEAR_UNIT_DEFAULT(speed, car_.speed);
        EXPECT_NEAR_UNIT_DEFAULT(targetLinePos * micro::sgn(car_.speed), controlData_.lineControl.actual.pos);
    }

    micro::point2m getJunctionPos(const char junctionId) const {
        return graph_.findJunction(junctionId)->pos;
    }

    micro::meter_t getSegmentLength(const Segment::Id& segmentId) {
        return graph_.findSegment(segmentId)->length;
    }

protected:
    LabyrinthGraph graph_;
    micro::fixed_number_generator random_{0.0f};
    LabyrinthNavigator navigator_{graph_, random_};
    micro::CarProps car_;
    micro::LineInfo lineInfo_;
    micro::MainLine mainLine_{cfg::CAR_FRONT_REAR_SENSOR_ROW_DIST};
    micro::ControlData controlData_{LABYRINTH_SPEED, micro::millisecond_t(500), false, {}};
};

} // namespace
