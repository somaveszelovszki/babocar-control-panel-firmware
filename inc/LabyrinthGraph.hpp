#pragma once

#include "Graph.hpp"
#include <micro/utils/point2.hpp>
#include <micro/utils/units.hpp>
#include <micro/utils/CarProps.hpp>
#include <micro/utils/algorithm.hpp>

#include "cfg_track.hpp"

namespace micro {

struct Junction;
struct Segment;

/* @brief Labyrinth segment connection.
 */
struct Connection : public Edge {
    Connection()
        : junction(nullptr)
        , dir(Direction::CENTER) {}

    Connection(Segment *centerNode_, Segment *curveNode_, Junction *junction_, Direction dir_);

    void updateNodes();

    Junction *junction;     // The junction.
    Direction dir;    // The connection direction (CENTER -> CURVE node steering direction)
};

/* @brief Labyrinth junction (crossroads).
 */
struct Junction {
    struct SegmentOrientation {
        Segment *seg;
        bool isCenter;
//
//        // Indicates segment position in junction - used for determining floating segments.
//        // Every junction has 3 segments connected to each other in a Y shape.
//        // The bottom segment is the CENTER segment, the upper segments are the LEFT and RIGHT segments.
//        // Given a connection type and a side, the given segment can be obtained.
//        // e.g. if the current connection type is FORWARD and the pattern side is LEFT, then the car is coming from the top left segment of the Y.
//        Direction pos;

        optional<radian_t> value;
    };

    Junction() : idx(-1) {
//        this->segOrientations[0].pos = Direction::CENTER;
//        this->segOrientations[1].pos = Direction::LEFT;
//        this->segOrientations[2].pos = Direction::RIGHT;
        this->segOrientations[0].seg = this->segOrientations[1].seg = this->segOrientations[2].seg = nullptr;
    }

    Status setOrientation(Segment *seg, radian_t orientation);

    Status addSegment(Segment *seg);

    Status setCenterSegment(Segment *seg);

//    Status setSegmentPos(Segment *seg, Direction segmentPos);

    Status getOrientation(Segment *seg, optional<radian_t>& result) const;

//    Status getOrientation(Direction pos, optional<radian_t>& result) const;

//    Segment* getSegment(Connection::Type type, Direction side) const;

//    Segment* getSegment(Direction segmentPos) const;

    Segment* getSegment(radian_t orientation) const;

    Status updateSegment(Segment *oldSeg, Segment *newSeg);

    bool isConnected(Segment *seg) const;

    optional<radian_t> getCenterOrientation() const;

    int32_t idx;
    point2<meter_t> pos; // Junction position - relative to car start position.
    SegmentOrientation segOrientations[3];  // Segments orientations at the junction - used for compensating car orientation sensor drift.

//    static Direction getSegmentPos(Connection::Type type, Direction side);
};

/* @brief Labyrinth segment.
 */
struct Segment : public Node {
    Segment() : name('_') {}


    /* @brief Gets fastest route to the other segment.
     * @param dest The destination segment.
     * @param result The result route.
     */
    void getFastestRoute(const Segment& dest, vec<Segment*, cfg::MAX_NUM_LAB_SEGMENTS>& result);

    bool isFloating() const;

    void reset();

    char name;
    meter_t length;  // The segment length.
};

} // namespace uns
