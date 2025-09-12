#pragma once

#include <Graph.hpp>
#include <algorithm>
#include <cfg_track.hpp>

#include <etl/string.h>

#include <micro/container/map.hpp>
#include <micro/container/set.hpp>
#include <micro/container/vector.hpp>
#include <micro/math/unit_utils.hpp>
#include <micro/utils/point2.hpp>
#include <micro/utils/units.hpp>

struct JunctionDecision {
    micro::radian_t orientation;
    micro::Direction direction{micro::Direction::CENTER};

    JunctionDecision() = default;

    JunctionDecision(micro::radian_t orientation, micro::Direction direction)
        : orientation(orientation), direction(direction) {}

    bool operator==(const JunctionDecision& other) const {
        return micro::eqWithOverflow360(this->orientation, other.orientation, micro::PI_4) &&
               this->direction == other.direction;
    }

    bool operator!=(const JunctionDecision& other) const { return !(*this == other); }
};

inline bool operator<(const JunctionDecision& a, const JunctionDecision& b) {
    return a.orientation < b.orientation ||
           (a.orientation == b.orientation &&
            micro::underlying_value(a.direction) < micro::underlying_value(b.direction));
}

inline bool operator>(const JunctionDecision& a, const JunctionDecision& b) {
    return a < b;
}
inline bool operator<=(const JunctionDecision& a, const JunctionDecision& b) {
    return !(b < a);
}
inline bool operator>=(const JunctionDecision& a, const JunctionDecision& b) {
    return !(a < b);
}

struct Junction;
struct Segment;

/* @brief Labyrinth segment connection.
 */
struct Connection : public Edge<Segment> {
    Junction* junction{nullptr};
    JunctionDecision decision1, decision2;

    Connection() = default;

    Connection(Segment& seg1, Segment& seg2, Junction& junction, const JunctionDecision& decision1,
               const JunctionDecision& decision2)
        : Edge(seg1, seg2), junction(&junction), decision1(decision1), decision2(decision2) {}

    Segment* getOtherSegment(const Segment& seg) const;

    JunctionDecision getDecision(const Segment& seg) const;
};

/* @brief Labyrinth junction (cross-roads).
 */
struct Junction {
    using SideSegments =
        micro::map<micro::Direction, Segment*, cfg::MAX_NUM_CROSSING_SEGMENTS_SIDE>;
    using segment_map = micro::map<micro::radian_t, SideSegments, 2>;

    char id{'\0'};
    micro::point2<micro::meter_t> pos; // Junction position - relative to car start position.
    segment_map segments;

    Junction() = default;

    Junction(const char id, const micro::point2<micro::meter_t>& pos) : id(id), pos(pos) {}

    void addSegment(Segment& seg, const JunctionDecision& decision);

    Segment* getSegment(micro::radian_t orientation, micro::Direction dir) const;

    size_t getConnectionCount(const Segment& seg) const;

    const SideSegments* getSideSegments(micro::radian_t orientation) const {
        return const_cast<Junction*>(this)->getSideSegments(orientation);
    }

    SideSegments* getSideSegments(micro::radian_t orientation);
};

/* @brief Labyrinth segment.
 */
struct Segment : public Node<Connection, cfg::MAX_NUM_CROSSING_SEGMENTS> {
    using Id = etl::string<2>;
    Id id;
    micro::meter_t length;
    bool isDeadEnd{false};

    Segment() = default;

    Segment(const char junction1, const char junction2, micro::meter_t length,
            const bool isDeadEnd = false)
        : id{makeId(junction1, junction2)}, length{length}, isDeadEnd{isDeadEnd} {}

    bool isFloating() const;
    bool isLoop() const;

    static Id makeId(const char junction1, const char junction2);
};

using Segments    = micro::vector<Segment, cfg::MAX_NUM_LABYRINTH_SEGMENTS>;
using Junctions   = micro::vector<Junction, cfg::MAX_NUM_LABYRINTH_JUNCTIONS>;
using Connections = micro::vector<Connection, cfg::MAX_NUM_LABYRINTH_SEGMENTS * 2>;

using JunctionIds = micro::set<char, cfg::MAX_NUM_LABYRINTH_JUNCTIONS>;
using SegmentIds  = micro::set<Segment::Id, cfg::MAX_NUM_LABYRINTH_SEGMENTS>;

class LabyrinthGraph {
  public:
    LabyrinthGraph()                      = default;
    LabyrinthGraph(const LabyrinthGraph&) = delete;
    LabyrinthGraph(LabyrinthGraph&&)      = delete;

    void addSegment(const Segment& seg);
    void addJunction(const Junction& junc);
    void connect(const char junc1, const JunctionDecision& decision1, const char junc2,
                 const JunctionDecision& decision2, const micro::meter_t sectionLength);

    void connectDeadEnd(const char junc, const JunctionDecision& decision,
                        const micro::meter_t sectionLength);

    Segment* findSegment(const Segment::Id& id);
    const Segment* findSegment(const Segment::Id& id) const;

    Junction* findJunction(const char id);
    const Junction* findJunction(const char id) const;

    const Junction* findClosestJunction(const micro::point2m& pos) const;

    const Connection* findConnection(const Segment& seg1, const Segment& seg2) const;

    micro::set<Segment::Id, cfg::MAX_NUM_LABYRINTH_SEGMENTS> getVisitableSegments();

    bool valid() const;

  private:
    void connect(Segment* seg, Junction* junc, const JunctionDecision& decision);

  private:
    Segments segments_;
    Junctions junctions_;
    Connections connections_;
};

#define EXPECT_EQ_JUNCTION_DECISION(expected, result)                                              \
    EXPECT_TRUE(micro::eqWithOverflow360(expected.orientation, result.orientation, micro::PI_4));  \
    EXPECT_EQ(expected.direction, result.direction)
