#include <micro/utils/log.hpp>

#include <LabyrinthNavigator.hpp>

using namespace micro;

LabyrinthNavigator::LabyrinthNavigator(const LabyrinthGraph& graph, const Segment *startSeg, const Connection *prevConn,
    const micro::m_per_sec_t fwdSpeed, const micro::m_per_sec_t fwdSlowSpeed, const micro::m_per_sec_t bwdSpeed)
    : Maneuver()
    , fwdSpeed_(fwdSpeed)
    , fwdSlowSpeed_(fwdSlowSpeed)
    , bwdSpeed_(bwdSpeed)
    , graph_(graph)
    , startSeg_(startSeg)
    , prevConn_(prevConn)
    , currentSeg_(this->startSeg_)
    , plannedRoute_(startSeg)
    , isLastTarget_(false)
    , lastJuncDist_(0)
    , targetDir_(Direction::CENTER) {}

void LabyrinthNavigator::initialize() {
    this->currentSeg_ = this->startSeg_;
}

const Segment* LabyrinthNavigator::currentSegment() const {
    return this->currentSeg_;
}

const Segment* LabyrinthNavigator::targetSegment() const {
    return this->plannedRoute_.destSeg;
}

const Connection* LabyrinthNavigator::nextConnection() const {
    return this->plannedRoute_.firstConnection();
}

const micro::Pose& LabyrinthNavigator::correctedCarPose() const {
    return this->correctedCarPose_;
}

void LabyrinthNavigator::setTargetSegment(const Segment *targetSeg, bool isLast) {
    LOG_DEBUG("Next target segment: %c", targetSeg->name);
    this->plannedRoute_ = LabyrinthRoute::create(*this->prevConn_, *this->currentSeg_, *targetSeg);
    this->isLastTarget_ = isLast;

    LOG_DEBUG("Planned route:");

    const Segment *prev = this->plannedRoute_.startSeg;
    for (const Connection *c : this->plannedRoute_.connections) {
        const Segment *next = c->getOtherSegment(*prev);
        LOG_DEBUG("-> %c (%s)", next->name, to_string(c->getDecision(*next).direction));
        prev = next;
    }

    this->updateTargetDirection();
}

void LabyrinthNavigator::update(const micro::CarProps& car, const micro::LineInfo& lineInfo, micro::MainLine& mainLine, micro::ControlData& controlData) {

    this->correctedCarPose_ = car.pose;

    updateCarOrientation(car, lineInfo);

    if (lineInfo.front.pattern != this->prevLineInfo_.front.pattern) {
        // checks if the car is coming out of a junction
        if (isJunction(lineInfo.front.pattern) && Sign::POSITIVE == lineInfo.front.pattern.dir) {
            onJunctionDetected(car, numJunctionSegments(prevLineInfo_.front.pattern), numJunctionSegments(lineInfo.front.pattern));
        }

        if (LinePattern::DEAD_END == lineInfo.front.pattern.type || LinePattern::DEAD_END == lineInfo.rear.pattern.type) {
            LOG_ERROR("DEAD_END pattern detected! Something's wrong...");
        }
    }

    if (car.distance - this->lastJuncDist_ >= this->plannedRoute_.startSeg->length / 2) {
        this->updateTargetDirection();
    }

    this->setControl(car, lineInfo, mainLine, controlData);

    this->prevLineInfo_ = lineInfo;

    if (this->isLastTarget_ && LinePattern::LANE_CHANGE == lineInfo.front.pattern.type) {
        this->finish();
    }
}

void LabyrinthNavigator::updateCarOrientation(const CarProps& car, const LineInfo& lineInfo) {
    if (car.orientedDistance > centimeter_t(60)                             &&
        car.distance - this->lastOrientationUpdateDist_ > centimeter_t(100) &&
        LinePattern::SINGLE_LINE == lineInfo.front.pattern.type             &&
        car.distance - lineInfo.front.pattern.startDist > centimeter_t(100) &&
        eqWithOverflow360(car.pose.angle, round90(car.pose.angle), degree_t(10))) {

        this->correctedCarPose_.angle = round90(car.pose.angle);
        this->lastOrientationUpdateDist_ = car.orientedDistance;
    }
}

void LabyrinthNavigator::updateTargetDirection() {
    const Connection *nextConn = this->plannedRoute_.firstConnection();
    this->targetDir_ = nextConn ? nextConn->getDecision(*nextConn->getOtherSegment(*this->plannedRoute_.startSeg)).direction : Direction::CENTER;
    LOG_DEBUG("Target direction: %s", to_string(this->targetDir_));
}

void LabyrinthNavigator::onJunctionDetected(const CarProps& car, uint8_t numInSegments, uint8_t numOutSegments) {

    const radian_t negOri = round90(car.pose.angle + PI);
    const radian_t posOri = round90(car.pose.angle);

    const micro::vec<std::pair<micro::radian_t, uint8_t>, 2> numSegments = {
        { negOri, numInSegments  },
        { posOri, numOutSegments }
    };

    LOG_DEBUG("Junction detected (car pos: (%f, %f), angle: %f deg, segments: (in: %u, out: %u))",
        car.pose.pos.X.get(),
        car.pose.pos.Y.get(),
        static_cast<degree_t>(car.pose.angle).get(),
        static_cast<uint32_t>(numInSegments),
        static_cast<uint32_t>(numOutSegments));

    const Junction *junc = this->graph_.findJunction(car.pose.pos, numSegments);

    if (junc) {
        LOG_DEBUG("Junction found: %u (%f, %f), current segment: %c",
            static_cast<uint32_t>(junc->id),
            junc->pos.X.get(),
            junc->pos.Y.get(),
            this->currentSeg_->name);

        this->correctedCarPose_.pos = junc->pos;

        const Connection *nextConn = this->nextConnection();
        if (!nextConn || junc != nextConn->junction) {
            LOG_ERROR("An unknown error has occurred, resets navigator");
            this->reset(*junc, negOri);
        }

        this->lastJuncDist_ = car.distance;
        this->prevConn_     = this->plannedRoute_.firstConnection();

        if (this->prevConn_) {
            this->plannedRoute_.pop_front();
            this->currentSeg_ = this->plannedRoute_.startSeg;
            LOG_INFO("Current segment: %c", this->currentSeg_->name);
        }

    } else {
        LOG_ERROR("Junction not found");
    }
}

void LabyrinthNavigator::setControl(const micro::CarProps& car, const micro::LineInfo& lineInfo, micro::MainLine& mainLine, micro::ControlData& controlData) {
    const meter_t distSinceLastJunc = car.distance - this->lastJuncDist_;
    const meter_t distUntilNextJunc = this->currentSeg_->length - distSinceLastJunc;

    //  starts going forward when in a junction:
    //     if the speed was slow, starts acceleration
    //     if the car was going backwards from a dead-end segment, starts going forward
    if (isJunction(lineInfo.front.pattern)) {
        controlData.speed = this->fwdSpeed_;

    // start going backward when a dead-end sign is detected by the front sensor
    } else if (LinePattern::DEAD_END == lineInfo.front.pattern.type) {
        controlData.speed = this->bwdSpeed_;

    // start going forward when a dead-end sign is detected by the rear sensor
    } else if (LinePattern::DEAD_END == lineInfo.rear.pattern.type) {
        controlData.speed = this->fwdSpeed_;

    // starts going backward when in a dead-end segment and the gate has been passed (so a new target segment has been received)
    } else if (this->currentSeg_->isDeadEnd && this->targetSegment() != this->currentSeg_) {
        controlData.speed = this->bwdSpeed_;

    // slows down when waiting for the next target segment and getting close to the next junction
    } else if (!this->nextConnection() && distUntilNextJunc < centimeter_t(100)) {
        controlData.speed = this->fwdSlowSpeed_;

    } // else: does not change current speed value

    controlData.rampTime = millisecond_t(700);

    if (distSinceLastJunc < centimeter_t(50) && lineInfo.front.lines.size() && lineInfo.rear.lines.size()) {
        switch (this->targetDir_) {
        case Direction::LEFT:
            mainLine.frontLine = lineInfo.front.lines[0];
            mainLine.rearLine  = *lineInfo.rear.lines.back();
            break;
        case Direction::CENTER:
            mainLine.frontLine = lineInfo.front.lines.size() == 1 ? lineInfo.front.lines[0] : lineInfo.front.lines[1];
            mainLine.rearLine  = lineInfo.rear.lines.size()  == 1 ? lineInfo.rear.lines[0]  : lineInfo.rear.lines[1];
            break;
        case Direction::RIGHT:
            mainLine.frontLine = *lineInfo.front.lines.back();
            mainLine.rearLine  = lineInfo.rear.lines[0];
            break;
        }
    }

    mainLine.updateCenterLine(micro::sgn(car.speed));

    controlData.controlType        = ControlData::controlType_t::Line;
    controlData.lineControl.actual = mainLine.centerLine;
    controlData.lineControl.target = { millimeter_t(0), radian_t(0) };
}

void LabyrinthNavigator::reset(const Junction& junc, radian_t negOri) {
    // Finds a valid current segment - may be any of the segments behind the car, connecting to the current junction.
    // This will be the start segment for the next route.
    const Segment *currentSeg = junc.getSideSegments(negOri)->second.begin()->second;

    // Finds a valid junction connection - may be any of the current segment's connections that are linked to the current junction.
    // This is only necessary temporarily, in order to find a valid previous connection.
    const Connection *junctionConn = *std::find_if(currentSeg->edges.begin(), currentSeg->edges.end(), [&junc](const Connection *c) {
        return c->junction->id == junc.id;
    });

    // Finds a valid previous connection.
    const Connection *prevConn = *std::find_if(currentSeg->edges.begin(), currentSeg->edges.end(), [junctionConn, currentSeg](const Connection *c) {
        return LabyrinthRoute::isNewConnectionValid(*c, *currentSeg, *junctionConn);
    });

    // Resets the navigator's previous connection and current segment, and recreates the route.
    this->prevConn_   = prevConn;
    this->currentSeg_ = currentSeg;
    this->setTargetSegment(this->plannedRoute_.destSeg, false);
}

bool LabyrinthNavigator::isJunction(const LinePattern& pattern) {
    return LinePattern::JUNCTION_1 == pattern.type ||
           LinePattern::JUNCTION_2 == pattern.type ||
           LinePattern::JUNCTION_3 == pattern.type;
}

uint8_t LabyrinthNavigator::numJunctionSegments(const LinePattern& pattern) {
    return isJunction(pattern) ? enum_cast(pattern.type) - enum_cast(LinePattern::JUNCTION_1) + 1 : 0;
}
