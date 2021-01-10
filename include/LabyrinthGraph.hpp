#pragma once

#include <micro/utils/point2.hpp>
#include <micro/utils/units.hpp>
#include <micro/container/map.hpp>
#include <micro/container/vec.hpp>

#include <Graph.hpp>
#include <cfg_track.hpp>

#include <algorithm>

struct JunctionDecision {
    micro::radian_t orientation;
    micro::Direction direction;

    JunctionDecision()
        : orientation(0)
        , direction(micro::Direction::CENTER) {}

    JunctionDecision(micro::radian_t orientation, micro::Direction direction)
        : orientation(orientation)
        , direction(direction) {}

    bool operator==(const JunctionDecision& other) const {
        return micro::eqWithOverflow360(this->orientation, other.orientation, micro::PI_4) && this->direction == other.direction;
    }

    bool operator!=(const JunctionDecision& other) const {
        return !(*this == other);
    }
};

struct Junction;
struct Segment;

/* @brief Labyrinth segment connection.
 */
struct Connection : public Edge<Segment> {
    Connection(Segment& seg1, Segment& seg2, Junction& junction, const JunctionDecision& decision1, const JunctionDecision& decision2)
        : Edge(seg1, seg2)
        , junction(&junction)
        , decision1(decision1)
        , decision2(decision2) {}

    Connection()
        : Edge()
        , junction(nullptr)
        , decision1()
        , decision2() {}

    Segment* getOtherSegment(const Segment& seg) const;

    JunctionDecision getDecision(const Segment& seg) const;

    Junction *junction;
    JunctionDecision decision1, decision2;
};

/* @brief Labyrinth junction (cross-roads).
 */
struct Junction {
    typedef micro::unsorted_map<micro::Direction, Segment*, cfg::MAX_NUM_CROSSING_SEGMENTS_SIDE> side_segment_map;
    typedef micro::unsorted_map<micro::radian_t, side_segment_map, 2> segment_map;
    typedef micro::vec<std::pair<micro::radian_t, micro::Direction>, 2> segment_info;

    Junction(uint8_t id, const micro::point2<micro::meter_t>& pos)
        : id(id)
        , pos(pos) {}

    Junction() : Junction(0, {}) {}

    micro::Status addSegment(Segment& seg, const JunctionDecision& decision);

    Segment* getSegment(micro::radian_t orientation, micro::Direction dir) const;

    bool isConnected(const Segment& seg) const;

    segment_info getSegmentInfo(micro::radian_t orientation, const Segment& seg);

    segment_info getSegmentInfo(const Segment& seg);

    uint8_t id;
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

    Segment(char name, micro::meter_t length, bool isDeadEnd)
        : name(name)
        , length(length)
        , isDeadEnd(isDeadEnd) {}

    Segment() : Segment('_', micro::meter_t(0), false) {}

    bool isFloating() const;

    bool isLoop() const;

    char name;
    micro::meter_t length;  // The segment length.
    bool isDeadEnd;
};

class LabyrinthGraph {
public:
    LabyrinthGraph() {}

    void addSegment(const Segment& seg);
    void addJunction(const Junction& junc);
    void connect(Segment *seg, Junction *junc, const JunctionDecision& decision);

    Segment* findSegment(char name);
    const Segment* findSegment(char name) const;

    Junction* findJunction(uint8_t id);
    const Junction* findJunction(uint8_t id) const;

    const Junction* findJunction(const micro::point2m& pos, const micro::vec<std::pair<micro::radian_t, uint8_t>, 2>& numSegments) const;

    const Connection* findConnection(const Segment& seg1, const Segment& seg2) const;

private:
    typedef micro::vec<Segment, cfg::NUM_LABYRINTH_SEGMENTS> Segments;
    typedef micro::vec<Junction, cfg::NUM_LABYRINTH_SEGMENTS> Junctions;
    typedef micro::vec<Connection, cfg::NUM_LABYRINTH_SEGMENTS * 2> Connections;

    Segments segments_;
    Junctions junctions_;
    Connections connections_;
};
