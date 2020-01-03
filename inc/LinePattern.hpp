#pragma once

#include <micro/utils/Line.hpp>
#include <micro/utils/point2.hpp>
#include <micro/container/infinite_buffer.hpp>

#include <ProgramState.hpp>

#include <functional>

namespace micro {

/* @brief Stores data of a detected line pattern.
 **/
struct LinePattern {
public:
    /* @brief Defines line pattern type.
     **/
    enum Type : uint8_t {
        NONE        = 0, ///< No lines detected
        SINGLE_LINE = 1, ///< 1 solid 1.9mm wide line.
        ACCELERATE  = 2, ///< 1 solid line in the middle, 2 dashed lines on both sides
        BRAKE       = 3, ///< 1 solid line in the middle, 2 solid lines on both sides
        LANE_CHANGE = 4, ///< 1 solid line in the middle, 1 dashed line on one (UNKNOWN!) side (dash and space lengths decreasing)
        JUNCTION_1  = 5, ///< Labyrinth junction of 1 segment
        JUNCTION_2  = 6, ///< Labyrinth junction of 2 segments
        JUNCTION_3  = 7, ///< Labyrinth junction of 3 segments
        DEAD_END    = 8  ///< Labyrinth dead-end
    };

    Type type;        // Line pattern type.
    Sign dir;         // The pattern direction (POSITIVE means it is the same as the car direction).
    Direction side;   // The side of the line the pattern is on. In case of symmetrical pattern: CENTER.
    meter_t startDist;

    LinePattern()
        : type(Type::NONE)
        , dir(Sign::POSITIVE)
        , side(Direction::CENTER)
        , startDist(0) {}

    LinePattern(Type type, Sign dir, Direction side, meter_t startDist = meter_t(0))
        : type(type)
        , dir(dir)
        , side(side)
        , startDist(startDist) {}

    bool operator==(const LinePattern& other) const {
        return this->type == other.type && this->dir == other.dir && this->side == other.side;
    }

    bool operator!=(const LinePattern& other) const {
        return !(*this == other);
    }
};

class LinePatternCalculator {
public:
    struct StampedLines {
        LinePositions front;
        LinePositions rear;
        meter_t distance;
    };

    typedef infinite_buffer<StampedLines, 500> measurement_buffer_t;
    typedef infinite_buffer<LinePattern, 200> pattern_buffer_t;
    typedef vec<LinePattern, 10> linePatterns_t;

    struct LinePatternInfo {
        meter_t validityLength;
        std::function<bool(const measurement_buffer_t&, const LinePattern&, const LinePositions&, const LinePositions&, meter_t)> isValid;
        std::function<linePatterns_t(const LinePattern&, const ProgramState)> validNextPatterns;
    };

    LinePatternCalculator(void)
        : isPatternChangeCheckActive(false)
        , currentPatternInfo(nullptr) {
        this->prevPatterns.push_back({ LinePattern::NONE, Sign::POSITIVE, Direction::CENTER, meter_t(0) });
    }

    void update(const ProgramState programState, const LinePositions& front, const LinePositions& rear, meter_t currentDist);

    const LinePattern& pattern() const {
        return const_cast<LinePatternCalculator*>(this)->currentPattern();
    }

    static meter_t distanceSinceNumLines(const measurement_buffer_t& prevMeas, uint8_t numLines, meter_t currentDist);

private:
    LinePattern& currentPattern() {
        return this->prevPatterns.peek_back(0);
    }

    void changePattern(const LinePattern& newPattern);

    measurement_buffer_t prevMeas;
    pattern_buffer_t prevPatterns;

    bool isPatternChangeCheckActive;
    linePatterns_t possiblePatterns;
    const LinePatternInfo *currentPatternInfo;
};

} // namespace micro
