#include <uns/LabyrinthGraph.hpp>

using namespace uns;

Status Junction::setOrientation(Segment *seg, RotationDir segmentPos, angle_t orientation) {
    SegmentOrientation& ori0 = this->segOrientations[0];
    SegmentOrientation& ori1 = this->segOrientations[1];
    SegmentOrientation& ori2 = this->segOrientations[2];
    Status status = Status::OK;

    if (ori0.seg == seg) {
        ori0.pos = segmentPos;
        ori0.value = orientation;
    } else if (ori1.seg == seg) {
        ori1.pos = segmentPos;
        ori1.value = orientation;
    } else if (ori2.seg == seg) {
        ori2.pos = segmentPos;
        ori2.value = orientation;
    } else if(!ori0.seg) {
        ori0.seg = seg;
        ori0.pos = segmentPos;
        ori0.value = orientation;
    } else if (!ori1.seg) {
        ori1.seg = seg;
        ori1.pos = segmentPos;
        ori1.value = orientation;
    } else if (!ori2.seg) {
        ori2.seg = seg;
        ori2.pos = segmentPos;
        ori2.value = orientation;
    } else {
        status = Status::INVALID_ID;
    }
    return status;
}

Status Junction::getOrientation(Segment *seg, angle_t *result) const {
    const SegmentOrientation& ori0 = this->segOrientations[0];
    const SegmentOrientation& ori1 = this->segOrientations[1];
    const SegmentOrientation& ori2 = this->segOrientations[2];
    Status status = Status::OK;

    if (ori0.seg == seg) {
        *result = ori0.value;
    } else if (ori1.seg == seg) {
        *result = ori1.value;
    } else if (ori2.seg == seg) {
        *result = ori2.value;
    } else {
        status = Status::INVALID_ID;
    }
    return status;
}

Status Junction::getOrientation(RotationDir pos, angle_t *result) const {
    const SegmentOrientation& ori0 = this->segOrientations[0];
    const SegmentOrientation& ori1 = this->segOrientations[1];
    const SegmentOrientation& ori2 = this->segOrientations[2];
    Status status = Status::OK;

    if (ori0.pos == pos) {
        *result = ori0.value;
    } else if (ori1.pos == pos) {
        *result = ori1.value;
    } else if (ori2.pos == pos) {
        *result = ori2.value;
    } else {
        status = Status::INVALID_ID;
    }
    return status;
}

Segment* Junction::getSegment(Connection::Type type, RotationDir side) const {
    return this->getSegment(Junction::getSegmentPos(type, side));
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

Segment* Junction::getSegment(RotationDir segmentPos) const {
    const SegmentOrientation& ori0 = this->segOrientations[0];
    const SegmentOrientation& ori1 = this->segOrientations[1];
    const SegmentOrientation& ori2 = this->segOrientations[2];
    Segment *result = nullptr;

    if (ori0.pos == segmentPos) {
        result = ori0.seg;
    } else if (ori1.pos == segmentPos) {
        result = ori1.seg;
    } else if (ori2.pos == segmentPos) {
        result = ori2.seg;
    }
    return result;
}

RotationDir Junction::getSegmentPos(Connection::Type type, RotationDir side) {
    RotationDir result;

    if (type == Connection::Type::STRAIGHT) {   // car can to only straight, so it must be in an upper segment (LEFT or RIGHT)
        if (side == RotationDir::LEFT) {    // incoming segment is on the left, so the car is in the upper left segment
            result = RotationDir::LEFT;
        } else {                            // incoming segment is on the right, so the car is in the upper right segment
            result = RotationDir::RIGHT;
        }
    } else {    // car can to straight and in a curve, so it must be in the bottom segment (CENTER)
        result = RotationDir::CENTER;
    }

    return result;
}

Connection::Connection(Segment *_node1, Segment *_node2, Junction *_junction, Type _type)
    : Edge{ _node1, _node2 }
    , junction(_junction)
    , type(_type) {}

void Connection::updateNodes() {
    this->node1->edges.append(this);
    this->node2->edges.append(this);
}

void Segment::getFastestRoute(const Segment& dest, Vec<Segment*, cfg::MAX_NUM_LAB_SEGMENTS>& result) {
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

Status Segment::setEndPosOri(Junction *junc, const PosOri& orientation) {
    EndPosOri& ori0 = this->orientations[0];
    EndPosOri& ori1 = this->orientations[1];
    Status status = Status::OK;

    if (ori0.junc == junc) {
        ori0.value = orientation;
    } else if (ori1.junc == junc) {
        ori1.value = orientation;
    } else if (!ori0.junc) {
        ori0.junc = junc;
        ori0.value = orientation;
    } else if (!ori1.junc) {
        ori1.junc = junc;
        ori1.value = orientation;
    } else {
        status = Status::INVALID_ID;
    }
    return status;
}

Status Segment::getEndPosOri(Junction *junc, PosOri *result) const {
    const EndPosOri& ori0 = this->orientations[0];
    const EndPosOri& ori1 = this->orientations[1];
    Status status = Status::OK;

    if (ori0.junc == junc) {
        *result = ori0.value;
    } else if (ori1.junc == junc) {
        *result = ori1.value;
    } else {
        status = Status::INVALID_ID;
    }
    return status;
}

