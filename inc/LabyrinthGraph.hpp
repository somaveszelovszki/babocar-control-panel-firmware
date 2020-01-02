#pragma once

#include <micro/utils/point2.hpp>
#include <micro/utils/units.hpp>
#include <micro/container/map.hpp>

#include <Graph.hpp>
#include <cfg_track.hpp>

#include <algorithm>

namespace micro {

struct Junction;
struct Segment;

/* @brief Labyrinth segment connection.
 */
struct Connection : public Edge<Segment> {
    Connection()
        : Edge()
        , junction(nullptr) {}

    Connection(Segment *seg1, Segment *seg2, Junction *junction)
        : Edge(seg1, seg2)
        , junction(junction) {}

    Status updateSegment(Segment *oldSeg, Segment *newSeg);

    Segment* getOtherSegment(const Segment *seg) const;

    Junction *junction; // The junction.
};

/* @brief Labyrinth junction (cross-roads).
 */
struct Junction {
    typedef micro::unordered_map<micro::Direction, Segment*, cfg::MAX_NUM_CROSSING_SEGMENTS_SIDE> side_segment_map;
    typedef micro::unordered_map<radian_t, side_segment_map, 2> segment_map;
    typedef micro::vec<std::pair<radian_t, Direction>, 2> segment_info;

    Junction() : idx(-1) {}

    Status addSegment(Segment *seg, radian_t orientation, Direction dir);

    Segment* getSegment(radian_t orientation, Direction dir);

    Status updateSegment(Segment *oldSeg, Segment *newSeg);

    bool isConnected(Segment *seg) const;

    segment_info getSegmentInfo(radian_t orientation, const Segment *seg);

    segment_info getSegmentInfo(const Segment *seg);

    int32_t idx;
    point2<meter_t> pos; // Junction position - relative to car start position.
    segment_map segments;

    segment_map::const_iterator getSideSegments(radian_t orientation) const {
        return std::find_if(this->segments.begin(), this->segments.end(), [orientation] (const segment_map::entry_type& entry) {
            return micro::eqWithOverflow360(orientation, entry.first, PI_4);
        });
    }

    segment_map::iterator getSideSegments(radian_t orientation) {
        return const_cast<segment_map::iterator>(const_cast<const Junction*>(this)->getSideSegments(orientation));
    }
};

/* @brief Labyrinth segment.
 */
struct Segment : public Node<Connection, cfg::MAX_NUM_CROSSING_SEGMENTS> {
    Segment()
        : name('_')
        , length(0)
        , isDeadEnd(false)
        , isActive(false) {}

    bool isFloating() const;

    bool isLoop() const;

    void reset();

    char name;
    meter_t length;  // The segment length.
    bool isDeadEnd;
    bool isActive;
};

} // namespace uns
