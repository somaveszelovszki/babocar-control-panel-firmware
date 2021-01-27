#include <micro/utils/log.hpp>

#include <LabyrinthNavigator.hpp>

using namespace micro;

LabyrinthNavigator::LabyrinthNavigator(const LabyrinthGraph& graph, const Segment *startSeg, const Connection *prevConn,
    const micro::m_per_sec_t targetSpeed, const micro::m_per_sec_t targetFastSpeed)
    : Maneuver()
    , targetSpeed_(targetSpeed)
    , targetFastSpeed_(targetFastSpeed)
    , graph_(graph)
    , startSeg_(startSeg)
    , prevConn_(prevConn)
    , currentSeg_(this->startSeg_)
    , route_(startSeg)
    , isLastTarget_(false)
    , lastJuncDist_(0)
    , targetDir_(Direction::CENTER)
    , targetSpeedSign_(Sign::POSITIVE) {}

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
    this->route_ = LabyrinthRoute::create(*this->prevConn_, *this->currentSeg_, *targetSeg, true);
    this->isLastTarget_ = isLast;

    LOG_DEBUG("Planned route:");

    const Segment *prev = this->route_.startSeg;
    for (const Connection *c : this->route_.connections) {
        const Segment *next = c->getOtherSegment(*prev);
        LOG_DEBUG("-> %c (%s)", next->name, to_string(c->getDecision(*next).direction));
        prev = next;
    }

    // checks if car needs to change speed sign in order to follow route
    const Connection *nextConn = this->route_.firstConnection();
    if (nextConn && this->prevConn_ && nextConn->junction == this->prevConn_->junction && !this->currentSeg_->isLoop()) {
        this->targetSpeedSign_ = -this->targetSpeedSign_;
    }
}

void LabyrinthNavigator::update(const micro::CarProps& car, const micro::LineInfo& lineInfo, micro::MainLine& mainLine, micro::ControlData& controlData) {

    this->correctedCarPose_ = car.pose;

    updateCarOrientation(car, lineInfo);

    const LinePattern& prevFrontPattern = this->frontLinePattern(this->prevLineInfo_);
    const LinePattern& prevRearPattern  = this->rearLinePattern(this->prevLineInfo_);
    const LinePattern& frontPattern     = this->frontLinePattern(lineInfo);
    const LinePattern& rearPattern      = this->rearLinePattern(lineInfo);

    if (frontPattern != prevFrontPattern) {
        if (isJunction(frontPattern) && Sign::POSITIVE == frontPattern.dir) {
            // car is coming out of a junction
            this->handleJunction(car, numJunctionSegments(prevFrontPattern), numJunctionSegments(frontPattern));
        }
    }

    if (rearPattern != prevRearPattern) {
        if (LinePattern::SINGLE_LINE == rearPattern.type && isJunction(prevRearPattern) && Sign::POSITIVE == prevRearPattern.dir) {
            // car has come out of the junction, checks if it needs to change speed sign in order to follow route
            const Connection *nextConn = this->route_.firstConnection();
            if (nextConn && this->prevConn_ && nextConn->junction == this->prevConn_->junction) {
                this->targetSpeedSign_ = -this->targetSpeedSign_;
            }
        }
    }

    // start going backward when a dead-end sign is detected
    if (this->isDeadEnd(car, frontPattern) && this->targetSpeedSign_ == sgn(car.speed)) {
        this->targetSpeedSign_ = -sgn(car.speed);
        LOG_ERROR("Dead-end detected! Something's wrong...");
    }

    this->setControl(car, lineInfo, mainLine, controlData);

    this->prevLineInfo_ = lineInfo;

    if (this->isLastTarget_ && (LinePattern::LANE_CHANGE == frontPattern.type || LinePattern::LANE_CHANGE == rearPattern.type)) {
        this->finish();
    }
}

const micro::LinePattern& LabyrinthNavigator::frontLinePattern(const micro::LineInfo& lineInfo) const {
    return Sign::POSITIVE == this->targetSpeedSign_ ? lineInfo.front.pattern : lineInfo.rear.pattern;
}

const micro::LinePattern& LabyrinthNavigator::rearLinePattern(const micro::LineInfo& lineInfo) const {
    return Sign::POSITIVE == this->targetSpeedSign_ ? lineInfo.rear.pattern : lineInfo.front.pattern;
}

void LabyrinthNavigator::updateCarOrientation(const CarProps& car, const LineInfo& lineInfo) {
    const LinePattern& frontPattern = this->frontLinePattern(lineInfo);
    if (car.orientedDistance > centimeter_t(60)                             &&
        car.distance - this->lastOrientationUpdateDist_ > centimeter_t(100) &&
        LinePattern::SINGLE_LINE == frontPattern.type                       &&
        car.distance - frontPattern.startDist > centimeter_t(100)           &&
        eqWithOverflow360(car.pose.angle, round90(car.pose.angle), degree_t(10))) {

        this->correctedCarPose_.angle = round90(car.pose.angle);
        this->lastOrientationUpdateDist_ = car.distance;
    }
}

void LabyrinthNavigator::handleJunction(const CarProps& car, uint8_t numInSegments, uint8_t numOutSegments) {

    const radian_t posOri = round90(car.speed >= m_per_sec_t(0) ? car.pose.angle : car.pose.angle + PI);
    const radian_t negOri = round90(posOri + PI);

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
        switch (this->targetSpeedSign_ * this->targetDir_) {
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

        mainLine.updateCenterLine();
    }
}

void LabyrinthNavigator::setControl(const micro::CarProps& car, const micro::LineInfo& lineInfo, micro::MainLine& mainLine, micro::ControlData& controlData) const {

    const m_per_sec_t prevSpeed = controlData.speed;

    if (!this->currentSeg_->isDeadEnd                                                                                                   &&
        isBtw(car.distance, this->lastJuncDist_ + centimeter_t(50), this->lastJuncDist_ + this->currentSeg_->length - centimeter_t(50)) &&
        1 == lineInfo.front.lines.size() && LinePattern::SINGLE_LINE == lineInfo.front.pattern.type                                     &&
        1 == lineInfo.rear.lines.size()  && LinePattern::SINGLE_LINE == lineInfo.rear.pattern.type) {
        controlData.speed = this->targetSpeedSign_ * this->targetFastSpeed_;
    } else {
        controlData.speed = this->targetSpeedSign_ * this->targetSpeed_;
    }

    controlData.rampTime = millisecond_t(300);

    if (controlData.speed != prevSpeed) {
        LOG_DEBUG("Target speed changed to %fm/s", controlData.speed.get());
    }

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
        return LabyrinthRoute::isForwardConnection(*c, *currentSeg, *junctionConn);
    });

    // Resets the navigator's previous connection and current segment, and recreates the route.
    this->prevConn_   = prevConn;
    this->currentSeg_ = currentSeg;
    this->setTargetSegment(this->route_.destSeg, false);
}

bool LabyrinthNavigator::isTargetLineOverrideEnabled(const CarProps& car, const LineInfo& lineInfo) const {
    const LinePattern& frontPattern = this->frontLinePattern(lineInfo);
    return isJunction(frontPattern) && Sign::POSITIVE == frontPattern.dir;
}

bool LabyrinthNavigator::isDeadEnd(const micro::CarProps& car, const micro::LinePattern& pattern) const {
    return LinePattern::NONE == pattern.type &&
           car.distance > centimeter_t(15)   && // prevents the car from starting backwards
           (this->currentSeg_->isDeadEnd || car.distance - pattern.startDist > centimeter_t(10));
}

bool LabyrinthNavigator::isJunction(const LinePattern& pattern) {
    return LinePattern::JUNCTION_1 == pattern.type ||
           LinePattern::JUNCTION_2 == pattern.type ||
           LinePattern::JUNCTION_3 == pattern.type;
}

uint8_t LabyrinthNavigator::numJunctionSegments(const LinePattern& pattern) {
    return isJunction(pattern) ? enum_cast(pattern.type) - enum_cast(LinePattern::JUNCTION_1) + 1 : 0;
}
