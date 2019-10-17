#include <LabyrinthGraph.hpp>
#include <micro/utils/log.hpp>

using namespace micro;

Status Junction::setOrientation(Segment *seg, radian_t orientation) {
    SegmentOrientation& ori0 = this->segOrientations[0];
    SegmentOrientation& ori1 = this->segOrientations[1];
    SegmentOrientation& ori2 = this->segOrientations[2];
    Status status = Status::OK;

    if (ori0.seg == seg) {
        ori0.value = orientation;
    } else if (ori1.seg == seg) {
        ori1.value = orientation;
    } else if (ori2.seg == seg) {
        ori2.value = orientation;
    } else {
        status = Status::INVALID_ID;
    }
    return status;
}

Status Junction::addSegment(Segment *seg) {
    SegmentOrientation& ori0 = this->segOrientations[0];
    SegmentOrientation& ori1 = this->segOrientations[1];
    SegmentOrientation& ori2 = this->segOrientations[2];
    Status status = Status::OK;

    if(!ori0.seg) {
        ori0.seg = seg;
    } else if (!ori1.seg) {
        ori1.seg = seg;
    } else if (!ori2.seg) {
        ori2.seg = seg;
    } else {
        status = Status::INVALID_ID;
    }
    return status;
}

Status Junction::setCenterSegment(Segment *seg) {
    SegmentOrientation& ori0 = this->segOrientations[0];
    SegmentOrientation& ori1 = this->segOrientations[1];
    SegmentOrientation& ori2 = this->segOrientations[2];
    Status status = Status::OK;

    if(ori0.seg == seg) {
        ori0.isCenter = true;
    } else if (ori1.seg == seg) {
        ori1.isCenter = true;
    } else if (ori2.seg == seg) {
        ori2.isCenter = true;
    } else {
        status = Status::INVALID_ID;
    }
    return status;
}

//Status Junction::setSegmentPos(Segment *seg, Direction segmentPos) {
//    SegmentOrientation& ori0 = this->segOrientations[0];
//    SegmentOrientation& ori1 = this->segOrientations[1];
//    SegmentOrientation& ori2 = this->segOrientations[2];
//    Status status = Status::OK;
//
//    if (ori0.seg == seg) {
//        ori0.pos = segmentPos;
//    } else if (ori1.seg == seg) {
//        ori1.pos = segmentPos;
//    } else if (ori2.seg == seg) {
//        ori2.pos = segmentPos;
//    } else if(!ori0.seg) {
//        ori0.seg = seg;
//        ori0.pos = segmentPos;
//    } else if (!ori1.seg) {
//        ori1.seg = seg;
//        ori1.pos = segmentPos;
//    } else if (!ori2.seg) {
//        ori2.seg = seg;
//        ori2.pos = segmentPos;
//    } else {
//        status = Status::INVALID_ID;
//    }
//    return status;
//}

Status Junction::getOrientation(Segment *seg, optional<radian_t>& result) const {
    const SegmentOrientation& ori0 = this->segOrientations[0];
    const SegmentOrientation& ori1 = this->segOrientations[1];
    const SegmentOrientation& ori2 = this->segOrientations[2];
    Status status = Status::OK;

    if (ori0.seg == seg) {
        result = ori0.value;
    } else if (ori1.seg == seg) {
        result = ori1.value;
    } else if (ori2.seg == seg) {
        result = ori2.value;
    } else {
        status = Status::INVALID_ID;
    }
    return status;
}

//Status Junction::getOrientation(Direction pos, optional<radian_t>& result) const {
//    const SegmentOrientation& ori0 = this->segOrientations[0];
//    const SegmentOrientation& ori1 = this->segOrientations[1];
//    const SegmentOrientation& ori2 = this->segOrientations[2];
//    Status status = Status::OK;
//
//    if (ori0.pos == pos) {
//        result = ori0.value;
//    } else if (ori1.pos == pos) {
//        result = ori1.value;
//    } else if (ori2.pos == pos) {
//        result = ori2.value;
//    } else {
//        status = Status::INVALID_ID;
//    }
//    return status;
//}

//Segment* Junction::getSegment(Connection::Type type, Direction side) const {
//    return this->getSegment(Junction::getSegmentPos(type, side));
//}

Segment* Junction::getSegment(radian_t orientation) const {
    static constexpr radian_t DEAD_BAND = degree_t(10.0f);
    const SegmentOrientation& ori0 = this->segOrientations[0];
    const SegmentOrientation& ori1 = this->segOrientations[1];
    const SegmentOrientation& ori2 = this->segOrientations[2];
    Segment *result = nullptr;

    radian_t min_ = radian_t::ZERO();

    if (ori0.value.hasValue() && (min_ == radian_t::ZERO() || abs(*ori0.value - orientation) < min_)) {
        min_ = abs(*ori0.value - orientation);
        result = ori0.seg;
        LOG_DEBUG("getSegment: min: %f degrees -> %c", static_cast<degree_t>(min_).get(), result ? result->name : 'X');
    }

    if (ori1.value.hasValue() && (min_ == radian_t::ZERO() || abs(*ori1.value - orientation) < min_)) {
        min_ = abs(*ori1.value - orientation);
        result = ori1.seg;
        LOG_DEBUG("getSegment: min: %f degrees -> %c", static_cast<degree_t>(min_).get(), result ? result->name : 'X');
   }

    if (ori2.value.hasValue() && (min_ == radian_t::ZERO() || abs(*ori2.value - orientation) < min_)) {
        abs(*ori2.value - orientation);
        result = ori2.seg;
        LOG_DEBUG("getSegment: min: %f degrees -> %c", static_cast<degree_t>(min_).get(), result ? result->name : 'X');
    }

    if (!result || min_ > DEAD_BAND) {
        if (!ori0.value.hasValue()) {
            LOG_DEBUG("segment updated: %c", result ? result->name : 'X');
            result = ori0.seg;
        } else if (!ori1.value.hasValue()) {
            result = ori1.seg;
            LOG_DEBUG("segment updated: %c", result ? result->name : 'X');
        } else if (!ori2.value.hasValue()) {
            result = ori2.seg;
            LOG_DEBUG("segment updated: %c", result ? result->name : 'X');
        }
    }

    return result;
}

Status Junction::updateSegment(Segment *oldSeg, Segment *newSeg) {
    SegmentOrientation& ori0 = this->segOrientations[0];
    SegmentOrientation& ori1 = this->segOrientations[1];
    SegmentOrientation& ori2 = this->segOrientations[2];
    Status status = Status::OK;

    if (ori0.seg == oldSeg) {
        ori0.seg = newSeg;
    } else if (ori1.seg == oldSeg) {
        ori1.seg = newSeg;
    } else if(ori2.seg == oldSeg) {
        ori2.seg = newSeg;
    } else {
        status = Status::INVALID_ID;
    }
    return status;
}

//Segment* Junction::getSegment(Direction segmentPos) const {
//    const SegmentOrientation& ori0 = this->segOrientations[0];
//    const SegmentOrientation& ori1 = this->segOrientations[1];
//    const SegmentOrientation& ori2 = this->segOrientations[2];
//    Segment *result = nullptr;
//
//    if (ori0.pos == segmentPos) {
//        result = ori0.seg;
//    } else if (ori1.pos == segmentPos) {
//        result = ori1.seg;
//    } else if (ori2.pos == segmentPos) {
//        result = ori2.seg;
//    }
//    return result;
//}

bool Junction::isConnected(Segment *seg) const {
    return this->segOrientations[0].seg == seg || this->segOrientations[1].seg == seg || this->segOrientations[2].seg == seg;
}

optional<radian_t> Junction::getCenterOrientation() const {
    const SegmentOrientation& ori0 = this->segOrientations[0];
    const SegmentOrientation& ori1 = this->segOrientations[1];
    const SegmentOrientation& ori2 = this->segOrientations[2];
    optional<radian_t> ori;

    if (ori0.isCenter) {
        ori = ori0.value;
    } else if (ori1.isCenter) {
        ori = ori1.value;
    } else if (ori2.isCenter) {
        ori = ori2.value;
    }

    return ori;
}

//Direction Junction::getSegmentPos(Connection::Type type, Direction side) {
//    Direction result;
//
//    if (type == Connection::Type::STRAIGHT) {   // car can go only straight, so it must be in an upper segment (LEFT or RIGHT)
//        if (side == Direction::LEFT) {    // incoming segment is on the left, so the car is in the upper left segment
//            result = Direction::LEFT;
//        } else {                            // incoming segment is on the right, so the car is in the upper right segment
//            result = Direction::RIGHT;
//        }
//    } else {    // car can to straight and in a curve, so it must be in the bottom segment (CENTER)
//        result = Direction::CENTER;
//    }
//
//    return result;
//}

Connection::Connection(Segment *centerNode_, Segment *curveNode_, Junction *junction_, Direction dir_)
    : Edge{ centerNode_, curveNode_ }
    , junction(junction_)
    , dir(dir_) {}

void Connection::updateNodes() {
    this->node1->edges.append(this);
    this->node2->edges.append(this);
}

void Segment::getFastestRoute(const Segment& dest, vec<Segment*, cfg::MAX_NUM_LAB_SEGMENTS>& result) {
    // TODO
}

bool Segment::isFloating() const {
    Junction *j1 = nullptr;
    bool floating = true;

    // iterates through all connections to check if the segment is connected to 2 different junctions (2 different junctions mean the segment is not floating)
    for (const Edge *e : this->edges) {
        const Connection *c = reinterpret_cast<const Connection*>(e);
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
