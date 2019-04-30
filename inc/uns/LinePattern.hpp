#pragma once

#include <uns/Line.hpp>
#include <uns/util/units.hpp>
#include <uns/Point2.hpp>

namespace uns {

/* @brief Stores data of a detected line pattern.
 **/
struct LinePattern : public Line {
public:
    static const Point2<distance_t> UNKNOWN_POS;

    /* @brief Defines line pattern type.
     **/
    enum class Type : uint8_t {
        SINGLE_LINE = 0,    // 1 solid 1.9mm wide line.
        ACCELERATE  = 1,    // 1 solid line in the middle, 2 dashed lines on both sides
        BRAKE       = 2,    // 1 solid line in the middle, 2 solid lines on both sides
        LANE_CHANGE = 3,    // 1 solid line in the middle, 1 dashed line on one (UNKNOWN!) side (dash and space lengths decreasing)
        JUNCTION    = 4     // Labyrinth junction
    };

    Type type;          // Line pattern type.
    Sign dir;           // The pattern direction (POSITIVE means it is the same as the car direction). e.g. at a junction: POSITIVE means the car has two options to choose from (forward or exit)
    RotationDir side;   // The side of the line the pattern is on. In case of symmetrical pattern: CENTER.
    Point2<distance_t> startPos;    // The pattern start position.
    Point2<distance_t> endPos;      // The pattern end position.

    LinePattern()
        : type(Type::SINGLE_LINE)
        , dir(Sign::POSITIVE)
        , side(RotationDir::CENTER)
        , startPos(UNKNOWN_POS)
        , endPos(UNKNOWN_POS) {}
};

} // namespace uns
