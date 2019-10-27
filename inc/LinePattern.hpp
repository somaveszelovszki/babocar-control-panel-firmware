#pragma once

#include <micro/utils/Line.hpp>
#include <micro/utils/point2.hpp>

namespace micro {

/* @brief Stores data of a detected line pattern.
 **/
struct LinePattern {
public:
    /* @brief Defines line pattern type.
     **/
    enum Type {
        SINGLE_LINE = 0,    // 1 solid 1.9mm wide line.
        ACCELERATE  = 1,    // 1 solid line in the middle, 2 dashed lines on both sides
        BRAKE       = 2,    // 1 solid line in the middle, 2 solid lines on both sides
        LANE_CHANGE = 3,    // 1 solid line in the middle, 1 dashed line on one (UNKNOWN!) side (dash and space lengths decreasing)
        JUNCTION    = 4     // Labyrinth junction
    };

    Type type;        // Line pattern type.
    Sign dir;         // The pattern direction (POSITIVE means it is the same as the car direction).
    Direction side;   // The side of the line the pattern is on. In case of symmetrical pattern: CENTER.
    meter_t startDist;

    LinePattern(void)
        : type(Type::SINGLE_LINE)
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
        : currentMeasIdx(0)
        , isCurrentPatternValidated(true)
        , isPatternChangeCheckNeeded(false) {}

    void update(meter_t currentDist, const Lines& lines);
private:
    struct StampedLines {
        meter_t distance;
        Lines lines;
    };

    void changePattern(LinePattern newPattern);

    meter_t distanceSinceNumLinesIs(uint8_t numLines, meter_t currentDist) const;

    bool isPatternValid(LinePattern::Type patternType, const Lines& lines, meter_t currentDist);

    static constexpr uint8_t PREV_MEAS_SIZE = 100;

    StampedLines prevMeas[PREV_MEAS_SIZE];
    uint8_t currentMeasIdx;

    LinePattern prevPrevPattern;
    LinePattern prevPattern;
    LinePattern currentPattern;
    bool isCurrentPatternValidated;
    bool isPatternChangeCheckNeeded;
};

} // namespace micro
