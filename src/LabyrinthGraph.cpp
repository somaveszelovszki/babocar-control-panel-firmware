#include <LabyrinthGraph.hpp>
#include <micro/utils/log.hpp>

using namespace micro;

Status Connection::updateSegment(Segment *oldSeg, Segment *newSeg) {

    Status result = Status::INVALID_ID;
    if (oldSeg && newSeg) {
        if (this->node1 == oldSeg) {
            this->node1 = newSeg;
            result = Status::OK;
        } else if (this->node2 == oldSeg) {
            this->node2 = newSeg;
            result = Status::OK;
        }
    }
    return result;
}

Segment* Connection::getOtherSegment(const Segment *seg) const {
    return this->node1 == seg ? this->node2 : this->node2 == seg ? this->node1 : nullptr;
}

Maneuver Connection::getManeuver(const Segment *seg) const {
    return this->node1 == seg ? this->maneuver1 : this->maneuver2;
}

Status Junction::addSegment(Segment *seg, radian_t orientation, Direction dir) {
    Status result = Status::ERROR;

    if (seg) {
        segment_map::iterator sideSegments = this->getSideSegments(orientation);

        if (sideSegments == this->segments.end()) {
            this->segments.emplace(orientation, side_segment_map());
            sideSegments = this->getSideSegments(orientation);
        }

        if (!sideSegments->second.at(dir)) {
            sideSegments->second.emplace(dir, seg);
            result = Status::OK;
        } else {
            result = Status::INVALID_DATA;
            LOG_ERROR("Junction %d already has one side segment in orientation: %fdeg and direction: %s",
                this->idx, static_cast<degree_t>(orientation).get(), to_string(dir));
        }
    } else {
        result = Status::INVALID_DATA;
        LOG_ERROR("Trying to add nullptr as segment to junction %d", this->idx);
    }

    return result;
}

Segment* Junction::getSegment(radian_t orientation, Direction dir) {
    Segment *result = nullptr;
    segment_map::const_iterator sideSegments = this->getSideSegments(orientation);

    if (sideSegments != this->segments.end()) {
        Segment * const * seg = sideSegments->second.at(dir);
        if (seg) {
            if (!(*seg)) {
                LOG_ERROR("Junction %d: side segment in orientation: %fdeg and direction: %s is nullptr",
                    this->idx, static_cast<degree_t>(orientation).get(), to_string(dir));
            }
            result = *seg;
        } else {
            LOG_ERROR("Junction %d has no side segment in orientation: %fdeg in direction: %s",
                this->idx, static_cast<degree_t>(orientation).get(), to_string(dir));
        }
    } else {
        LOG_ERROR("Junction %d has no side segments in orientation: %fdeg", this->idx, static_cast<degree_t>(orientation).get());
    }

    return result;
}

Status Junction::updateSegment(Segment *oldSeg, Segment *newSeg) {

    Status result = Status::INVALID_ID;
    if (oldSeg && newSeg) {
        for (segment_map::iterator itSide = this->segments.begin(); itSide != this-> segments.end(); ++itSide) {
            for (side_segment_map::iterator itSeg = itSide->second.begin(); itSeg != itSide->second.end(); ++itSeg) {
                if (itSeg->second == oldSeg) {
                    itSeg->second = newSeg;
                    result = Status::OK;
                    break;
                }
            }

            if (result == Status::OK) {
                break;
            }
        }
    }
    return result;
}

bool Junction::isConnected(Segment *seg) const {
    return !!seg ? std::find_if(this->segments.begin(), this->segments.end(), [seg] (const segment_map::entry_type& s1) {
        return std::find_if(s1.second.begin(), s1.second.end(), [seg] (const side_segment_map::entry_type& s2) {
            return s2.second == seg;
        }) != s1.second.end();
    }) != this->segments.end() : false;
}

Junction::segment_info Junction::getSegmentInfo(radian_t orientation, const Segment *seg) {
    segment_info info;

    const segment_map::iterator itSide = this->getSideSegments(orientation);
    if (itSide != this->segments.end()) {
        for (side_segment_map::iterator itSeg = itSide->second.begin(); itSeg != itSide->second.end(); ++itSeg) {
            if (itSeg->second == seg) {
                info.push_back({ itSide->first, itSeg->first });
            }
        }
    }

    return info;
}

Junction::segment_info Junction::getSegmentInfo(const Segment *seg) {
    segment_info info;

    for (segment_map::iterator itSide = this->segments.begin(); itSide != this->segments.end(); ++itSide) {
        for (side_segment_map::iterator itSeg = itSide->second.begin(); itSeg != itSide->second.end(); ++itSeg) {
            if (itSeg->second == seg) {
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
    return this->edges.size() > 0 && this->edges[0]->junction->getSegmentInfo(this).size() == 2;
}

void Segment::reset() {
    this->edges.clear();
}

void Route::append(Connection *c) {
    this->connections.push_back(c);
    this->lastSeg = c->getOtherSegment(this->lastSeg);
}

Connection* Route::nextConnection() {
    Connection *conn = nullptr;
    if (this->connections.size()) {
        conn = this->connections[0];
        this->startSeg = conn->getOtherSegment(this->startSeg);
        this->connections.erase(this->connections.begin());
    }
    return conn;
}

Connection* Route::lastConnection() const {
    return this->connections.size() > 0 ? this->connections[this->connections.size() - 1] : nullptr;
}

void Route::reset(Segment *currentSeg) {
    this->startSeg = this->lastSeg = currentSeg;
    this->connections.clear();
}

bool Route::isConnectionValid(const Connection *lastRouteConn, const Maneuver lastManeuver, const Connection *c) const {
    bool valid = false;

    // does not permit going backwards
    if (!lastRouteConn ||
        c->junction != lastRouteConn->junction ||
        c->getManeuver(this->lastSeg) != lastManeuver) {

        // does not permit navigating through dead-end segments
        if (!c->getOtherSegment(this->lastSeg)->isDeadEnd) {

            const Segment *newSeg = c->getOtherSegment(this->lastSeg);
            const Maneuver newManeuver = c->getManeuver(newSeg);

            valid = true;

            // does not permit cycles (taking the same junction exit twice)
            const Segment *prevSeg = this->startSeg;
            for (const Connection *routeConn : this->connections) {
                const Segment *nextSeg = routeConn->getOtherSegment(prevSeg);
                if (nextSeg == newSeg && routeConn->getManeuver(nextSeg) == newManeuver) {
                    valid = false;
                    break;
                }
                prevSeg = nextSeg;
            }

        }
    }
    return valid;
}

Junction* findExistingJunction(Junction *begin, Junction *end, const point2m& pos, radian_t inOri, radian_t outOri, uint8_t numInSegments, uint8_t numOutSegments) {

    Junction *result = nullptr;

    struct JunctionDist {
        Junction *junc = nullptr;
        meter_t dist = micro::numeric_limits<meter_t>::infinity();
    };

    LOG_DEBUG("pos: (%f, %f)", pos.X.get(), pos.Y.get());

    // FIRST:  closest junction to current position
    // SECOND: closest junction to current position with the correct topology
    std::pair<JunctionDist, JunctionDist> closest = {};

    for(Junction *j = begin; j != end; ++j) {
        const meter_t dist = pos.distance(j->pos);

        if (dist < closest.first.dist) {
            closest.first.junc = j;
            closest.first.dist = dist;
        }

        if (dist < closest.second.dist) {
            const Junction::segment_map::const_iterator inSegments = j->getSideSegments(inOri);
            if (inSegments != j->segments.end() && inSegments->second.size() == numInSegments) {

                const Junction::segment_map::const_iterator outSegments = j->getSideSegments(outOri);
                if (outSegments != j->segments.end() && outSegments->second.size() == numOutSegments) {
                    closest.second.junc = j;
                    closest.second.dist = dist;
                }
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
        result = nullptr;
    }

    return result;
}
