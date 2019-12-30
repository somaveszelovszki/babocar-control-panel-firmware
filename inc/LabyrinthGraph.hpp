#pragma once

#include "Graph.hpp"

#include <micro/utils/point2.hpp>
#include <micro/utils/units.hpp>
#include <micro/utils/CarProps.hpp>
#include <micro/utils/algorithm.hpp>
#include <micro/container/sorted_map.hpp>

#include "cfg_track.hpp"

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

    Junction *junction; // The junction.
};

/* @brief Labyrinth junction (cross-roads).
 */
struct Junction {
    typedef micro::unsorted_map<micro::Direction, Segment*, cfg::MAX_NUM_CROSSING_SEGMENTS_SIDE> side_segment_map_type;
    typedef micro::unsorted_map<radian_t, side_segment_map_type, 2> segment_map_type;

    Junction() : idx(-1) {}

    Status addSegment(Segment *seg, radian_t orientation, Direction dir);

    Segment* getSegment(radian_t orientation, Direction dir);

    Status updateSegment(Segment *oldSeg, Segment *newSeg);

    bool isConnected(Segment *seg) const;

    std::pair<radian_t, Direction> getSegmentInfo(const Segment *seg);

    int32_t idx;
    point2<meter_t> pos; // Junction position - relative to car start position.
    segment_map_type segments;

    segment_map_type::const_iterator getSideSegments(radian_t orientation) const {
        return std::find_if(this->segments.begin(), this->segments.end(), [orientation] (const segment_map_type::entry_type& entry) {
            return micro::eqWithOverflow360(orientation, entry.first, PI_4);
        });
    }

    segment_map_type::iterator getSideSegments(radian_t orientation) {
        return const_cast<segment_map_type::iterator>(const_cast<const Junction*>(this)->getSideSegments(orientation));
    }
};

/* @brief Labyrinth segment.
 */
struct Segment : public Node<Connection, cfg::MAX_NUM_CROSSING_SEGMENTS> {
    Segment()
        : name('_')
        , length(0)
        , isDeadEnd(false) {}

    bool isFloating() const;

    void reset();

    char name;
    meter_t length;  // The segment length.
    bool isDeadEnd;
};

} // namespace uns
