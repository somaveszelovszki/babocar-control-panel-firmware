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

Status Junction::addSegment(Segment *seg, radian_t orientation, Direction dir) {
    Status result = Status::ERROR;

    if (seg) {
        segment_map_type::iterator sideSegments = this->getSideSegments(orientation);

        if (sideSegments == this->segments.end()) {
            this->segments.put(orientation, side_segment_map_type());
            sideSegments = this->getSideSegments(orientation);
        }

        if (!sideSegments->second.get(dir)) {
            sideSegments->second.put(dir, seg);
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
    segment_map_type::const_iterator sideSegments = this->getSideSegments(orientation);

    if (sideSegments != this->segments.end()) {
        Segment * const * seg = sideSegments->second.get(dir);
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
        for (segment_map_type::iterator itSide = this->segments.begin(); itSide != this-> segments.end(); ++itSide) {
            for (side_segment_map_type::iterator itSeg = itSide->second.begin(); itSeg != itSide->second.end(); ++itSeg) {
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
    return !!seg ? std::find_if(this->segments.begin(), this->segments.end(), [seg] (const segment_map_type::entry_type& s1) {
        return std::find_if(s1.second.begin(), s1.second.end(), [seg] (const side_segment_map_type::entry_type& s2) {
            return s2.second == seg;
        }) != s1.second.end();
    }) != this->segments.end() : false;
}

std::pair<radian_t, Direction> Junction::getSegmentInfo(const Segment *seg) {
    bool found = false;
    std::pair<radian_t, Direction> info = { radian_t(0), Direction::CENTER };

    for (segment_map_type::iterator itSide = this->segments.begin(); itSide != this-> segments.end(); ++itSide) {
        for (side_segment_map_type::iterator itSeg = itSide->second.begin(); itSeg != itSide->second.end(); ++itSeg) {
            if (itSeg->second == seg) {
                info.first = itSide->first;
                info.second = itSeg->first;
                found = true;
                break;
            }
        }

        if (found) {
            break;
        }
    }

    return info;
}

bool Segment::isFloating() const {
    Junction *j1 = nullptr;
    bool floating = true;

    // iterates through all connections to check if the segment is connected to 2 different junctions (2 different junctions mean the segment is not floating)
    for (const Connection *c : this->edges) {
        if (j1 != c->junction) {
            if (j1) {
                floating = false;
                break;
            }
            j1 = c->junction;
        }
    }
    return floating;
}

void Segment::reset() {
    this->edges.clear();
}
