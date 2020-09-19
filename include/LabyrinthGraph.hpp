#pragma once

#include <micro/utils/point2.hpp>
#include <micro/utils/units.hpp>
#include <micro/container/map.hpp>

#include <Graph.hpp>
#include <cfg_track.hpp>

#include <algorithm>

struct Maneuver {
    micro::radian_t orientation;
    micro::Direction direction;

    Maneuver()
        : orientation(0)
        , direction(micro::Direction::CENTER) {}

    Maneuver(micro::radian_t orientation, micro::Direction direction)
        : orientation(orientation)
        , direction(direction) {}

    bool operator==(const Maneuver& other) const {
        return micro::eqWithOverflow360(this->orientation, other.orientation, micro::PI_4) && this->direction == other.direction;
    }

    bool operator!=(const Maneuver& other) const {
        return !(*this == other);
    }
};

struct Junction;
struct Segment;

/* @brief Labyrinth segment connection.
 */
struct Connection : public Edge<Segment> {
    Connection()
        : Edge()
        , junction(nullptr) {}

    micro::Status updateSegment(Segment *oldSeg, Segment *newSeg);

    Segment* getOtherSegment(const Segment *seg) const;

    Maneuver getManeuver(const Segment *seg) const;

    Junction *junction; // The junction.
    Maneuver maneuver1, maneuver2;
};

/* @brief Labyrinth junction (cross-roads).
 */
struct Junction {
    typedef micro::unsorted_map<micro::Direction, Segment*, cfg::MAX_NUM_CROSSING_SEGMENTS_SIDE> side_segment_map;
    typedef micro::unsorted_map<micro::radian_t, side_segment_map, 2> segment_map;
    typedef micro::vec<std::pair<micro::radian_t, micro::Direction>, 2> segment_info;

    Junction() : idx(-1) {}

    micro::Status addSegment(Segment *seg, micro::radian_t orientation, micro::Direction dir);

    Segment* getSegment(micro::radian_t orientation, micro::Direction dir);

    micro::Status updateSegment(Segment *oldSeg, Segment *newSeg);

    bool isConnected(Segment *seg) const;

    segment_info getSegmentInfo(micro::radian_t orientation, const Segment *seg);

    segment_info getSegmentInfo(const Segment *seg);

    int32_t idx;
    micro::point2<micro::meter_t> pos; // Junction position - relative to car start position.
    segment_map segments;

    segment_map::const_iterator getSideSegments(micro::radian_t orientation) const {
        return std::find_if(this->segments.begin(), this->segments.end(), [orientation] (const segment_map::entry_type& entry) {
            return micro::eqWithOverflow360(orientation, entry.first, micro::PI_4);
        });
    }

    segment_map::iterator getSideSegments(micro::radian_t orientation) {
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
    micro::meter_t length;  // The segment length.
    bool isDeadEnd;
    bool isActive;
};

struct Route {
    static constexpr uint32_t MAX_LENGTH = cfg::NUM_LAB_SEGMENTS;
    Segment *startSeg;
    Segment *lastSeg;
    micro::vec<Connection*, MAX_LENGTH> connections;

    Route()
        : startSeg(nullptr)
        , lastSeg(nullptr) {}

    void append(Connection *c);

    Connection* nextConnection();

    Connection* lastConnection() const;

    void reset(Segment *currentSeg);

    bool isConnectionValid(const Connection *lastRouteConn, const Maneuver lastManeuver, const Connection *c) const;
};

Junction* findExistingJunction(Junction *begin, Junction *end, const micro::point2m& pos, micro::radian_t inOri, micro::radian_t outOri, uint8_t numInSegments, uint8_t numOutSegments);
