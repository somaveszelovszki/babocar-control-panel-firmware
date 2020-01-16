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
        , dir(Sign::NEUTRAL)
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
        Lines lines;
        meter_t distance;
    };

    typedef infinite_buffer<StampedLines, 1000> measurement_buffer_t;
    typedef infinite_buffer<LinePattern, 200> pattern_buffer_t;
    typedef vec<LinePattern, 10> linePatterns_t;

    struct LinePatternInfo {
        meter_t validityLength;
        std::function<bool(const measurement_buffer_t&, const LinePattern&, const Lines&, meter_t)> isValid;
        std::function<linePatterns_t(const LinePattern&, const ProgramTask)> validNextPatterns;
    };

    LinePatternCalculator()
        : isPatternChangeCheckActive(false) {
        this->prevPatterns.push_back({ LinePattern::NONE, Sign::NEUTRAL, Direction::CENTER, meter_t(0) });
    }
    void update(const ProgramTask activeTask, const Lines& lines, meter_t currentDist);

    const LinePattern& pattern() const {
        return const_cast<LinePatternCalculator*>(this)->currentPattern();
    }

    static StampedLines peek_back(const measurement_buffer_t& prevMeas, std::function<bool(const StampedLines&)> condition);

    static StampedLines peek_back(const measurement_buffer_t& prevMeas, meter_t peekBackDist);

    static Lines::const_iterator getMainLine(const Lines& lines, const measurement_buffer_t& prevMeas);

    static bool areClose(const Lines& lines);

    static bool areFar(const Lines& lines);

private:
    LinePattern& currentPattern() {
        return this->prevPatterns.peek_back(0);
    }

    void changePattern(const LinePattern& newPattern);

    measurement_buffer_t prevMeas;
    pattern_buffer_t prevPatterns;

    bool isPatternChangeCheckActive;
    linePatterns_t possiblePatterns;
};

} // namespace micro
