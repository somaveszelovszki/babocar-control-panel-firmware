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
    , route_(startSeg)
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
    return this->route_.destSeg;
}

const micro::Pose& LabyrinthNavigator::correctedCarPose() const {
    return this->correctedCarPose_;
}

void LabyrinthNavigator::setTargetSegment(const Segment *targetSeg, bool isLast) {
    LOG_DEBUG("Next target segment: %c", targetSeg->name);
    this->route_ = LabyrinthRoute::create(*this->prevConn_, *this->currentSeg_, *targetSeg);
    this->isLastTarget_ = isLast;

    LOG_DEBUG("Planned route:");

    const Segment *prev = this->route_.startSeg;
    for (const Connection *c : this->route_.connections) {
        const Segment *next = c->getOtherSegment(*prev);
        LOG_DEBUG("-> %c (%s)", next->name, to_string(c->getDecision(*next).direction));
        prev = next;
    }
}

void LabyrinthNavigator::update(const micro::CarProps& car, const micro::LineInfo& lineInfo, micro::MainLine& mainLine, micro::ControlData& controlData) {

    this->correctedCarPose_ = car.pose;

    updateCarOrientation(car, lineInfo);

    if (lineInfo.front.pattern != this->prevLineInfo_.front.pattern) {
        if (isJunction(lineInfo.front.pattern) && Sign::POSITIVE == lineInfo.front.pattern.dir) {
            // car is coming out of a junction
            this->handleJunction(car, numJunctionSegments(prevLineInfo_.front.pattern), numJunctionSegments(lineInfo.front.pattern));

        } else if (this->isDeadEnd(car, lineInfo.front.pattern) || this->isDeadEnd(car, lineInfo.rear.pattern)) {
            LOG_ERROR("Dead-end pattern detected! Something's wrong...");
        }
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
        this->lastOrientationUpdateDist_ = car.distance;
    }
}

void LabyrinthNavigator::handleJunction(const CarProps& car, uint8_t numInSegments, uint8_t numOutSegments) {

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

        const Connection *nextConn = this->route_.firstConnection();
        if (junc != nextConn->junction) {
            LOG_ERROR("Unexpected junction, resets navigator");
            this->reset(*junc, negOri);
        }

        if (!nextConn) {
            LOG_ERROR("No target segment set, chooses target direction randomly");

            // Finds a valid next junction connection - may be any of the current segment's connections that are linked to the current junction.
            nextConn = *std::find_if(this->currentSeg_->edges.begin(), this->currentSeg_->edges.end(), [junc](const Connection *c) {
                return c->junction->id == junc->id;
            });
        }

        if (nextConn) {
            this->targetDir_ = nextConn->getDecision(*nextConn->getOtherSegment(*this->route_.startSeg)).direction;
            LOG_DEBUG("Target direction: %s", to_string(this->targetDir_));

            this->route_.pop_front();
            this->currentSeg_ = this->route_.startSeg;
            LOG_INFO("Current segment: %c", this->currentSeg_->name);

        } else {
            LOG_ERROR("No next connection found, resets navigator");
            this->reset(*junc, negOri);
        }

        this->lastJuncDist_ = car.distance;
        this->prevConn_     = nextConn;

    } else {
        LOG_ERROR("Junction not found");
    }
}

void LabyrinthNavigator::setTargetLine(const micro::CarProps& car, const micro::LineInfo& lineInfo, micro::MainLine& mainLine) const {

    // target line is only overwritten when the car is going in or coming out of a junction
    if (this->isTargetLineOverrideEnabled(car, lineInfo)) {
        switch (this->targetDir_) {
        case Direction::LEFT:
            if (lineInfo.front.lines.size()) {
                mainLine.frontLine = lineInfo.front.lines[0];
            }
            if (lineInfo.rear.lines.size()) {
                mainLine.rearLine = *lineInfo.rear.lines.back();
            }
            break;

        case Direction::CENTER:
            if (3 == lineInfo.front.lines.size()) {
                mainLine.frontLine = lineInfo.front.lines[1];
            }
            if (3 == lineInfo.rear.lines.size()) {
                mainLine.rearLine = lineInfo.rear.lines[1];
            }
            break;

        case Direction::RIGHT:
            if (lineInfo.front.lines.size()) {
                mainLine.frontLine = *lineInfo.front.lines.back();
            }
            if (lineInfo.rear.lines.size()) {
                mainLine.rearLine = lineInfo.rear.lines[0];
            }
            break;
        }

        mainLine.updateCenterLine(micro::sgn(car.speed));
    }
}

void LabyrinthNavigator::setControl(const micro::CarProps& car, const micro::LineInfo& lineInfo, micro::MainLine& mainLine, micro::ControlData& controlData) const {

    const meter_t distUntilNextJunc = this->lastJuncDist_ + this->currentSeg_->length - car.distance;

    //  starts going forward when in a junction:
    //     if the speed was slow, starts acceleration
    //     if the car was going backwards from a dead-end segment, starts going forward
    if (isJunction(lineInfo.front.pattern)) {
        controlData.speed = this->fwdSpeed_;

    // start going backward when a dead-end sign is detected by the front sensor
    } else if (car.speed > m_per_sec_t(0) && this->isDeadEnd(car, lineInfo.front.pattern)) {
        controlData.speed = this->bwdSpeed_;

    // start going forward when a dead-end sign is detected by the rear sensor
    } else if (car.speed < m_per_sec_t(0) && this->isDeadEnd(car, lineInfo.rear.pattern)) {
        controlData.speed = this->fwdSpeed_;

    // starts going backward when in a dead-end segment and the gate has been passed (so a new target segment has been received)
    } else if (this->currentSeg_->isDeadEnd && this->targetSegment() != this->currentSeg_) {
        controlData.speed = this->bwdSpeed_;

    // slows down when waiting for the next target segment and getting close to the next junction
    } else if (!this->route_.firstConnection() && distUntilNextJunc < centimeter_t(100)) {
        controlData.speed = this->fwdSlowSpeed_;

    } // else: does not change current speed value

    controlData.rampTime = millisecond_t(700);

    this->setTargetLine(car, lineInfo, mainLine);

    // enables rear wheel steering when coming out of a junction
    controlData.rearSteerEnabled   = this->isTargetLineOverrideEnabled(car, lineInfo);
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
    this->setTargetSegment(this->route_.destSeg, false);
}

bool LabyrinthNavigator::isTargetLineOverrideEnabled(const CarProps& car, const LineInfo& lineInfo) const {
    return isJunction(lineInfo.front.pattern) && Sign::POSITIVE == lineInfo.front.pattern.dir;
}

bool LabyrinthNavigator::isDeadEnd(const micro::CarProps& car, const micro::LinePattern& pattern) const {
    return LinePattern::NONE == pattern.type && (this->currentSeg_->isDeadEnd || car.distance - pattern.startDist > centimeter_t(30));
}

bool LabyrinthNavigator::isJunction(const LinePattern& pattern) {
    return LinePattern::JUNCTION_1 == pattern.type ||
           LinePattern::JUNCTION_2 == pattern.type ||
           LinePattern::JUNCTION_3 == pattern.type;
}

uint8_t LabyrinthNavigator::numJunctionSegments(const LinePattern& pattern) {
    return isJunction(pattern) ? enum_cast(pattern.type) - enum_cast(LinePattern::JUNCTION_1) + 1 : 0;
}
