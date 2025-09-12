#include <LabyrinthGraph.hpp>
#include <algorithm>
#include <micro/container/set.hpp>
#include <micro/container/vector.hpp>
#include <micro/log/log.hpp>
#include <micro/math/unit_utils.hpp>

using namespace micro;

Segment* Connection::getOtherSegment(const Segment& seg) const {
    return node1 == &seg ? node2 : node1;
}

JunctionDecision Connection::getDecision(const Segment& seg) const {
    return node1 == &seg ? decision1 : decision2;
}

void Junction::addSegment(Segment& seg, const JunctionDecision& decision) {
    auto* side = getSideSegments(decision.orientation);

    if (!side) {
        segments.insert(std::make_pair(decision.orientation, SideSegments()));
        side = getSideSegments(decision.orientation);
    }

    side->insert(std::make_pair(decision.direction, &seg));
}

Segment* Junction::getSegment(radian_t orientation, Direction dir) const {
    const auto* side = getSideSegments(orientation);
    if (!side) {
        LOG_ERROR("Junction {} has no side segments in orientation: {}deg", id,
                  static_cast<degree_t>(orientation).get());
        return nullptr;
    }

    const auto itSeg = side->find(dir);
    if (itSeg == side->end()) {
        LOG_ERROR("Junction {} has no side segment in orientation: {}deg in direction: {}", id,
                  static_cast<degree_t>(orientation).get(), to_string(dir));
        return nullptr;
    }

    return itSeg->second;
}

size_t Junction::getConnectionCount(const Segment& seg) const {
    return std::accumulate(
        segments.begin(), segments.end(), 0, [&seg](auto& count, const auto& key_side) {
            const auto& side = key_side.second;
            return count + std::count_if(side.begin(), side.end(), [&seg](const auto& key_seg) {
                       return key_seg.second == &seg;
                   });
        });
}

auto Junction::getSideSegments(micro::radian_t orientation) -> SideSegments* {
    const auto it =
        std::find_if(segments.begin(), segments.end(), [orientation](const auto& entry) {
            return micro::eqWithOverflow360(orientation, entry.first, micro::PI_4);
        });
    return it != segments.end() ? &it->second : nullptr;
}

bool Segment::isFloating() const {
    return !isDeadEnd && edges.size() < 2;
}

bool Segment::isLoop() const {
    return edges.size() > 0 && edges[0]->junction->getConnectionCount(*this) == 2;
}

auto Segment::makeId(const char junction1, const char junction2) -> Id {
    Id id;
    id.push_back(std::min(junction1, junction2));
    id.push_back(std::max(junction1, junction2));
    return id;
}

void LabyrinthGraph::addSegment(const Segment& seg) {
    segments_.push_back(seg);
}

void LabyrinthGraph::addJunction(const Junction& junc) {
    junctions_.push_back(junc);
}

void LabyrinthGraph::connect(const char junc1, const JunctionDecision& decision1, const char junc2,
                             const JunctionDecision& decision2,
                             const micro::meter_t sectionLength) {
    auto* junction1 = findJunction(junc1);
    auto* junction2 = findJunction(junc2);
    if (!junction1 || !junction2) {
        return;
    }

    addSegment(Segment(junc1, junc2, sectionLength));
    auto* segment = to_raw_pointer(segments_.rbegin());

    connect(segment, junction1, decision1);
    connect(segment, junction2, decision2);
}

void LabyrinthGraph::connectDeadEnd(const char junc, const JunctionDecision& decision,
                                    const micro::meter_t sectionLength) {
    auto* junction = findJunction(junc);
    if (!junction) {
        return;
    }

    addSegment(Segment('_', junc, sectionLength, true));
    auto* segment = to_raw_pointer(segments_.rbegin());

    connect(segment, junction, decision);
}

void LabyrinthGraph::connect(Segment* seg, Junction* junc, const JunctionDecision& decision) {
    junc->addSegment(*seg, decision);

    const auto oppositeOrientation = micro::round90(decision.orientation + PI);
    auto* oppositeSide             = junc->getSideSegments(oppositeOrientation);
    if (!oppositeSide) {
        return;
    }

    for (auto out = oppositeSide->begin(); out != oppositeSide->end(); ++out) {
        const auto& [outDir, outSeg] = *out;
        connections_.push_back(
            Connection(*seg, *outSeg, *junc, decision, {oppositeOrientation, outDir}));
        auto& conn = connections_.back();
        seg->edges.push_back(&conn);
        outSeg->edges.push_back(&conn);
    }
}

Segment* LabyrinthGraph::findSegment(const Segment::Id& id) {
    return const_cast<Segment*>(const_cast<const LabyrinthGraph*>(this)->findSegment(id));
}

const Segment* LabyrinthGraph::findSegment(const Segment::Id& id) const {
    const auto it = std::find_if(segments_.begin(), segments_.end(),
                                 [&id](const auto& s) { return s.id == id; });
    return it != segments_.end() ? to_raw_pointer(it) : nullptr;
}

Junction* LabyrinthGraph::findJunction(const char id) {
    return const_cast<Junction*>(const_cast<const LabyrinthGraph*>(this)->findJunction(id));
}

const Junction* LabyrinthGraph::findJunction(const char id) const {
    const Junctions::const_iterator it = std::find_if(
        junctions_.begin(), junctions_.end(), [id](const Junction& junc) { return junc.id == id; });

    return it != junctions_.end() ? to_raw_pointer(it) : nullptr;
}

const Junction* LabyrinthGraph::findClosestJunction(const point2m& pos) const {
    const auto it = std::min_element(
        junctions_.begin(), junctions_.end(),
        [&pos](const auto& a, const auto& b) { return pos.distance(a.pos) < pos.distance(b.pos); });

    return it != junctions_.end() ? to_raw_pointer(it) : nullptr;
}

const Connection* LabyrinthGraph::findConnection(const Segment& seg1, const Segment& seg2) const {
    const auto it =
        std::find_if(seg1.edges.begin(), seg1.edges.end(), [&seg1, &seg2](const auto& c) {
            return (c->node1 == &seg1 && c->node2 == &seg2) ||
                   (c->node1 == &seg2 && c->node2 == &seg1);
        });

    return it != seg1.edges.end() ? *it : nullptr;
}

micro::set<Segment::Id, cfg::MAX_NUM_LABYRINTH_SEGMENTS> LabyrinthGraph::getVisitableSegments() {
    micro::set<Segment::Id, cfg::MAX_NUM_LABYRINTH_SEGMENTS> segments;
    for (const auto& seg : segments_) {
        if (!seg.isDeadEnd) {
            segments.insert(seg.id);
        }
    }
    return segments;
}

bool LabyrinthGraph::valid() const {
    for (const auto& seg : segments_) {
        if (seg.id.size() != 2) {
            return false;
        }

        if (seg.length <= meter_t(0)) {
            return false;
        }

        if (seg.isFloating()) {
            return false;
        }

        micro::set<Junction*, cfg::MAX_NUM_LABYRINTH_SEGMENTS * 2> junctions;
        for (const auto& conn : connections_) {
            if (conn.node1 == &seg || conn.node2 == &seg) {
                junctions.insert(conn.junction);
            }
        }

        if (junctions.size() != (seg.isDeadEnd || seg.isLoop() ? 1 : 2)) {
            return false;
        }

        uint32_t occurences = 0;
        for (const auto& junc : junctions_) {
            occurences += junc.getConnectionCount(seg);
        }

        if (occurences != (seg.isDeadEnd ? 1 : 2)) {
            return false;
        }
    }

    return true;
}
