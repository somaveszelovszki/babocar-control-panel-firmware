#include <micro/utils/log.hpp>
#include <micro/math/unit_utils.hpp>
#include <micro/container/vec.hpp>

#include <LabyrinthGraph.hpp>

using namespace micro;

Segment* Connection::getOtherSegment(const Segment& seg) const {
    return this->node1 == &seg ? this->node2 : this->node2 == &seg ? this->node1 : nullptr;
}

JunctionDecision Connection::getDecision(const Segment& seg) const {
    return this->node1 == &seg ? this->decision1 : this->decision2;
}

Status Junction::addSegment(Segment& seg, const JunctionDecision& decision) {
    Status result = Status::ERROR;

    segment_map::iterator sideSegments = this->getSideSegments(decision.orientation);

    if (sideSegments == this->segments.end()) {
        this->segments.emplace(decision.orientation, side_segment_map());
        sideSegments = this->getSideSegments(decision.orientation);
    }

    if (!sideSegments->second.at(decision.direction)) {
        sideSegments->second.emplace(decision.direction, &seg);
        result = Status::OK;
    } else {
        result = Status::INVALID_DATA;
        LOG_ERROR("Junction %u already has one side segment in orientation: %fdeg and direction: %s",
            static_cast<uint32_t>(this->id), static_cast<degree_t>(decision.orientation).get(), to_string(decision.direction));
    }

    return result;
}

Segment* Junction::getSegment(radian_t orientation, Direction dir) const {
    Segment *result = nullptr;
    segment_map::const_iterator sideSegments = this->getSideSegments(orientation);

    if (sideSegments != this->segments.end()) {
        Segment * const * seg = sideSegments->second.at(dir);
        if (seg) {
            if (!(*seg)) {
                LOG_ERROR("Junction %u: side segment in orientation: %fdeg and direction: %s is nullptr",
                    static_cast<uint32_t>(this->id), static_cast<degree_t>(orientation).get(), to_string(dir));
            }
            result = *seg;
        } else {
            LOG_ERROR("Junction %u has no side segment in orientation: %fdeg in direction: %s",
                static_cast<uint32_t>(this->id), static_cast<degree_t>(orientation).get(), to_string(dir));
        }
    } else {
        LOG_ERROR("Junction %u has no side segments in orientation: %fdeg", static_cast<uint32_t>(this->id), static_cast<degree_t>(orientation).get());
    }

    return result;
}

bool Junction::isConnected(const Segment& seg) const {
    return std::find_if(this->segments.begin(), this->segments.end(), [seg] (const segment_map::entry_type& s1) {
        return std::find_if(s1.second.begin(), s1.second.end(), [&seg] (const side_segment_map::entry_type& s2) {
            return s2.second == &seg;
        }) != s1.second.end();
    }) != this->segments.end();
}

Junction::segment_info Junction::getSegmentInfo(radian_t orientation, const Segment& seg) {
    segment_info info;

    const segment_map::iterator itSide = this->getSideSegments(orientation);
    if (itSide != this->segments.end()) {
        for (side_segment_map::iterator itSeg = itSide->second.begin(); itSeg != itSide->second.end(); ++itSeg) {
            if (itSeg->second == &seg) {
                info.push_back({ itSide->first, itSeg->first });
            }
        }
    }

    return info;
}

Junction::segment_info Junction::getSegmentInfo(const Segment& seg) {
    segment_info info;

    for (segment_map::iterator itSide = this->segments.begin(); itSide != this->segments.end(); ++itSide) {
        for (side_segment_map::iterator itSeg = itSide->second.begin(); itSeg != itSide->second.end(); ++itSeg) {
            if (itSeg->second == &seg) {
                info.push_back({ itSide->first, itSeg->first });
            }
        }
    }

    return info;
}

bool Segment::isFloating() const {
    Junction *j1 = nullptr;
    bool floating = true;

    if (this->isDeadEnd) {
        floating = false; // a dead-end section cannot be floating (it only has one connected end by definition)
    } else {
        // iterates through all connections to check if the segment is connected to 2 different junctions,
        // meaning that the segment is not floating
        for (const Connection *c : this->edges) {
            if (j1 != c->junction) {
                if (j1) {
                    floating = false;
                    break;
                }
                j1 = c->junction;
            }
        }

        // if the segment can be approached via the same junction, but in 2 different directions
        // (e.g. LEFT and RIGHT or from different angles), then the segment is not floating, but it is closing into itself (loop)
        if (floating) {
            floating = !this->isLoop();
        }
    }

    return floating;
}

bool Segment::isLoop() const {
    return this->edges.size() > 0 && this->edges[0]->junction->getSegmentInfo(*this).size() == 2;
}

void LabyrinthGraph::addSegment(const Segment& seg) {
    this->segments.push_back(seg);
}

void LabyrinthGraph::addJunction(const Junction& junc) {
    this->junctions.push_back(junc);
}

void LabyrinthGraph::connect(Segments::iterator seg, Junctions::iterator junc, const JunctionDecision& decision) {

    junc->addSegment(*seg, decision);

    Junction::segment_map::iterator otherSideSegments = junc->getSideSegments(round90(decision.orientation + PI));

    if (otherSideSegments != junc->segments.end()) {
        for (Junction::side_segment_map::iterator out = otherSideSegments->second.begin(); out != otherSideSegments->second.end(); ++out) {
            Connections::iterator conn = this->connections.push_back(Connection(*seg, *out->second, *junc, decision, { otherSideSegments->first, out->first }));
            seg->edges.push_back(conn);
            out->second->edges.push_back(conn);
        }
    }
}

LabyrinthGraph::Segments::iterator LabyrinthGraph::findSegment(char name) {
    return const_cast<LabyrinthGraph::Segments::iterator>(const_cast<const LabyrinthGraph*>(this)->findSegment(name));
}

LabyrinthGraph::Segments::const_iterator LabyrinthGraph::findSegment(char name) const {
    return std::find_if(this->segments.begin(), this->segments.end(), [name](const Segment& seg) { return seg.name == name; });
}

LabyrinthGraph::Junctions::iterator LabyrinthGraph::findJunction(uint8_t id) {
    return const_cast<LabyrinthGraph::Junctions::iterator>(const_cast<const LabyrinthGraph*>(this)->findJunction(id));
}

LabyrinthGraph::Junctions::const_iterator LabyrinthGraph::findJunction(uint8_t id) const {
    return std::find_if(this->junctions.begin(), this->junctions.end(), [id](const Junction& junc) { return junc.id == id; });
}

LabyrinthGraph::Junctions::const_iterator LabyrinthGraph::findJunction(const point2m& pos, const micro::vec<std::pair<micro::radian_t, uint8_t>, 2>& numSegments) const {

    Junctions::const_iterator result = this->junctions.end();

    struct JunctionDist {
        Junctions::const_iterator junc;
        meter_t dist;
    };

    LOG_DEBUG("pos: (%f, %f)", pos.X.get(), pos.Y.get());

    // FIRST:  closest junction to current position
    // SECOND: closest junction to current position with the correct topology
    std::pair<JunctionDist, JunctionDist> closest = {
        { this->junctions.end(), micro::numeric_limits<meter_t>::infinity() },
        { this->junctions.end(), micro::numeric_limits<meter_t>::infinity() }
    };

    for(Junctions::const_iterator it = this->junctions.begin(); it != this->junctions.end(); ++it) {
        const meter_t dist = pos.distance(it->pos);

        if (dist < closest.first.dist) {
            closest.first.junc = it;
            closest.first.dist = dist;
        }

        if (dist < closest.second.dist) {
            bool topologyOk = true;
            for (const std::pair<micro::radian_t, uint8_t>& numSegs : numSegments) {
                const Junction::segment_map::const_iterator segments = it->getSideSegments(numSegs.first);
                if (segments == it->segments.end() || segments->second.size() != numSegs.second) {
                    topologyOk = false;
                    break;
                }
            }

            if (topologyOk) {
                closest.second.junc = it;
                closest.second.dist = dist;
            }
        }
    }

    if (closest.first.junc) {
        LOG_DEBUG("closest: (%f, %f)", closest.first.junc->pos.X.get(), closest.first.junc->pos.Y.get());
    }

    if (closest.second.junc) {
        LOG_DEBUG("closest with ori: (%f, %f)", closest.second.junc->pos.X.get(), closest.second.junc->pos.Y.get());
    }

    if (closest.second.dist < centimeter_t(120)) {
        // a junction at the right position and the correct topology has been found
        result = closest.second.junc;
    } else if (closest.first.dist < centimeter_t(80)) {
        // a junction at the right position but with incorrect topology has been found
        result = closest.first.junc;
    } else {
        // the junction has not been found
        result = this->junctions.end();
    }

    return result;
}

LabyrinthGraph::Connections::const_iterator LabyrinthGraph::findFirstConnection(const Segment& seg1, const Segment& seg2) const {
    return std::find_if(this->connections.begin(), this->connections.end(), [&seg1, &seg2](const Connection& c) {
        return (c.node1->name == seg1.name && c.node2->name == seg2.name) || (c.node1->name == seg2.name && c.node2->name == seg1.name);
    });
}
