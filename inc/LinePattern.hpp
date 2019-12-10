#pragma once

#include <micro/utils/Line.hpp>
#include <micro/utils/point2.hpp>
#include <micro/container/infinite_buffer.hpp>

namespace micro {

/* @brief Stores data of a detected line pattern.
 **/
struct LinePattern {
public:
    /* @brief Defines line pattern type.
     **/
    enum Type {
        NONE        = 0, ///< No lines detected
        SINGLE_LINE = 1, ///< 1 solid 1.9mm wide line.
        ACCELERATE  = 2, ///< 1 solid line in the middle, 2 dashed lines on both sides
        BRAKE       = 3, ///< 1 solid line in the middle, 2 solid lines on both sides
        LANE_CHANGE = 4, ///< 1 solid line in the middle, 1 dashed line on one (UNKNOWN!) side (dash and space lengths decreasing)
        JUNCTION    = 5, ///< Labyrinth junction
        DEAD_END    = 6, ///< Labyrinth dead-end

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

    LinePattern(Type type, Sign dir, Direction side, meter_t startDist)
        : type(type)
        , dir(dir)
        , side(side)
        , startDist(startDist) {}
};

class LinePatternCalculator {
public:
    LinePatternCalculator(void)
        : isPatternChangeCheckActive(false) {
        this->prevPatterns.push_back({ LinePattern::NONE, Sign::POSITIVE, Direction::CENTER, meter_t(0) });
    }

    void update(const LinePositions& front, const LinePositions& rear, meter_t currentDist);

    const LinePattern& pattern() const {
        return const_cast<LinePatternCalculator*>(this)->currentPattern();
    }

private:
    struct StampedLines {
        LinePositions front;
        LinePositions rear;
        meter_t distance;
    };

    LinePattern& currentPattern() {
        return this->prevPatterns.peek_back(0);
    }

    void changePattern(const LinePattern& newPattern);

    meter_t distanceSinceNumLinesIs(uint8_t numLines, meter_t currentDist) const;

    bool isPatternValid(const LinePattern& pattern, const LinePositions& front, const LinePositions& rear, meter_t currentDist);

    void startPatternChangeCheck(const std::initializer_list<LinePattern>& possiblePatterns);

    infinite_buffer<StampedLines, 500> prevMeas;
    infinite_buffer<LinePattern, 200> prevPatterns;

    bool isPatternChangeCheckActive;
    static constexpr uint32_t MAX_NUM_POSSIBLE_PATTERNS = 10;
    vec<LinePattern, MAX_NUM_POSSIBLE_PATTERNS> possiblePatterns;
};

} // namespace micro
