#include <micro/utils/log.hpp>

#include <LabyrinthNavigator.hpp>

using namespace micro;

LabyrinthNavigator::LabyrinthNavigator(const LabyrinthGraph& graph, const Segment *startSeg, const Connection *prevConn, const Segment *laneChangeSeg,
    const micro::m_per_sec_t targetSpeed, const micro::m_per_sec_t targetFastSpeed, const micro::m_per_sec_t targetDeadEndSpeed)
    : Maneuver()
    , targetSpeed_(targetSpeed)
    , targetFastSpeed_(targetFastSpeed)
    , targetDeadEndSpeed_(targetDeadEndSpeed)
    , graph_(graph)
    , startSeg_(startSeg)
    , prevConn_(prevConn)
    , currentSeg_(this->startSeg_)
    , targetSeg_(startSeg)
    , laneChangeSeg_(laneChangeSeg)
    , route_(startSeg)
    , isLastTarget_(false)
    , lastJuncDist_(0)
    , targetDir_(Direction::CENTER)
    , targetSpeedSign_(Sign::POSITIVE)
    , isSpeedSignChangeInProgress_(false)
    , hasSpeedSignChanged_(false)
    , isInJunction_(false)
    , random_(0) {}

void LabyrinthNavigator::initialize() {
    this->currentSeg_ = this->startSeg_;
}

const Segment* LabyrinthNavigator::currentSegment() const {
    return this->currentSeg_;
}

const Segment* LabyrinthNavigator::targetSegment() const {
    return this->targetSeg_;
}

const micro::Pose& LabyrinthNavigator::correctedCarPose() const {
    return this->correctedCarPose_;
}

bool LabyrinthNavigator::isLastTarget() const {
    return this->isLastTarget_;
}

void LabyrinthNavigator::setTargetSegment(const Segment *targetSeg, bool isLast) {
    LOG_DEBUG("Next target segment: %c", targetSeg->name);
    this->targetSeg_    = targetSeg;
    this->isLastTarget_ = isLast;
}

void LabyrinthNavigator::update(const micro::CarProps& car, const micro::LineInfo& lineInfo, micro::MainLine& mainLine, micro::ControlData& controlData) {

    this->correctedCarPose_ = car.pose;

    updateCarOrientation(car, lineInfo);

    const LinePattern& prevFrontPattern = this->frontLinePattern(this->prevLineInfo_);
    const LinePattern& prevRearPattern  = this->rearLinePattern(this->prevLineInfo_);
    const LinePattern& frontPattern     = this->frontLinePattern(lineInfo);
    const LinePattern& rearPattern      = this->rearLinePattern(lineInfo);

    // does not handle pattern changes while car is changing speed sign
    if (this->isSpeedSignChangeInProgress_) {
        if (sgn(car.speed) == this->targetSpeedSign_ && LinePattern::SINGLE_LINE == frontPattern.type) {
            this->isSpeedSignChangeInProgress_ = false;
            LOG_DEBUG("Speed sign change finished");
        }
    } else {
        if (frontPattern != prevFrontPattern) {
            if (isJunction(frontPattern) && Sign::POSITIVE == frontPattern.dir) {
                // car is coming out of a junction
                this->handleJunction(car, numJunctionSegments(prevFrontPattern), numJunctionSegments(frontPattern));
                this->isInJunction_ = true;
            }
        }

        if (rearPattern != prevRearPattern) {
            if (LinePattern::SINGLE_LINE == rearPattern.type) {
                this->isInJunction_ = false;
            }
        }

        // start going backward when a dead-end sign is detected
        if (this->isDeadEnd(car, frontPattern)) {
            LOG_ERROR("Dead-end detected! Labyrinth target speed sign changed to %s", to_string(this->targetSpeedSign_));
            this->tryToggleTargetSpeedSign(car.distance);
        }
    }

    if (this->targetSeg_ != this->route_.destSeg || this->currentSeg_ != this->route_.startSeg) {
        this->updateRoute();
    }

    // Checks if car needs to change speed sign in order to follow route.
    // @note This is only enabled when the car is not in a junction.
    if (!this->isInJunction_) {
        const Connection *nextConn = this->route_.firstConnection();
        if (nextConn && this->prevConn_ && nextConn->junction == this->prevConn_->junction && !this->currentSeg_->isLoop()) {
            this->tryToggleTargetSpeedSign(car.distance);
        }
    }

    this->setControl(car, lineInfo, mainLine, controlData);

    this->prevLineInfo_ = lineInfo;

    if (this->isLastTarget_ && (LinePattern::LANE_CHANGE == frontPattern.type || LinePattern::LANE_CHANGE == rearPattern.type)) {
        this->finish();
    }
}

bool LabyrinthNavigator::isReducedLineScanEnabled() const {
    const micro::LinePattern& frontPattern = this->frontLinePattern(this->prevLineInfo_);
    return isJunction(frontPattern) && Sign::POSITIVE == frontPattern.dir;
}

const micro::LinePattern& LabyrinthNavigator::frontLinePattern(const micro::LineInfo& lineInfo) const {
    return Sign::POSITIVE == this->targetSpeedSign_ ? lineInfo.front.pattern : lineInfo.rear.pattern;
}

const micro::LinePattern& LabyrinthNavigator::rearLinePattern(const micro::LineInfo& lineInfo) const {
    return Sign::POSITIVE == this->targetSpeedSign_ ? lineInfo.rear.pattern : lineInfo.front.pattern;
}

const micro::Lines& LabyrinthNavigator::frontLines(const micro::LineInfo& lineInfo) const {
    return Sign::POSITIVE == this->targetSpeedSign_ ? lineInfo.front.lines : lineInfo.rear.lines;
}

const micro::Lines& LabyrinthNavigator::rearLines(const micro::LineInfo& lineInfo) const {
    return Sign::POSITIVE == this->targetSpeedSign_ ? lineInfo.rear.lines : lineInfo.front.lines;
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

    // checks if any junction has been found at the current position
    if (junc) {
        LOG_DEBUG("Junction found: %u (%f, %f), current segment: %c",
            static_cast<uint32_t>(junc->id),
            junc->pos.X.get(),
            junc->pos.Y.get(),
            this->currentSeg_->name);

        this->correctedCarPose_.pos = junc->pos;

        // checks if current segment connects to found junction
        if (junc->getSegmentInfo(*this->currentSeg_).size() > 0) {
            const Connection *nextConn = this->route_.firstConnection();

            // checks if next connection is available, meaning the route is not yet finished
            if (nextConn) {
                if (junc == nextConn->junction) {
                    this->targetDir_ = nextConn->getDecision(*nextConn->getOtherSegment(*this->route_.startSeg)).direction;
                    LOG_DEBUG("Next connection ok, target direction: %s", to_string(this->targetDir_));

                    this->route_.pop_front();
                    this->currentSeg_ = this->route_.startSeg;
                    this->prevConn_   = nextConn;

                } else {
                    LOG_ERROR("Unexpected junction, resets navigator");
                    this->reset(*junc, negOri);
                }

            } else {
                LOG_WARN("No next connection available, chooses target direction randomly");

                // finds a valid next junction connection - may be any of the current segment's connections that are linked to the current junction
                micro::vec<Connection*, cfg::MAX_NUM_CROSSING_SEGMENTS_SIDE> validConnections;
                for (Connection *c : this->currentSeg_->edges) {
                    if (c->junction->id == junc->id) {
                        validConnections.push_back(c);
                    }
                }

                this->prevConn_ = validConnections[this->random_.get(0, validConnections.size())];
                this->currentSeg_ = this->prevConn_->getOtherSegment(*this->currentSeg_);
            }

        } else {
            LOG_ERROR("Current segment does not connect to found junction, resets navigator");
            this->reset(*junc, negOri);
        }
    } else {
        LOG_ERROR("Junction not found, chooses target direction randomly");
        const uint8_t targetLineIdx = this->random_.get(0, numOutSegments);

        switch (numOutSegments) {
        case 3:
            this->targetDir_ = 0 == targetLineIdx ? Direction::LEFT : 1 == targetLineIdx ? Direction::CENTER : Direction::RIGHT;
            break;
        case 2:
            this->targetDir_ = 0 == targetLineIdx ? Direction::LEFT : Direction::RIGHT;
            break;
        default:
            this->targetDir_ = Direction::CENTER;
            break;
        }
    }

    LOG_INFO("Current segment: %c", this->currentSeg_->name);

    this->lastJuncDist_ = car.distance;
    this->hasSpeedSignChanged_ = false;
}

void LabyrinthNavigator::tryToggleTargetSpeedSign(const micro::meter_t currentDist) {
    if (!this->hasSpeedSignChanged_) {
        this->targetSpeedSign_             = -this->targetSpeedSign_;
        this->isSpeedSignChangeInProgress_ = true;
        this->hasSpeedSignChanged_         = true;
        this->lastSpeedSignChangeDistance_ = currentDist;
        LOG_DEBUG("Labyrinth target speed sign changed to %s", to_string(this->targetSpeedSign_));
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

    if (this->currentSeg_->isDeadEnd) {
        controlData.speed = this->targetSpeedSign_ * this->targetDeadEndSpeed_;

    } else if (isBtw(car.distance, this->lastJuncDist_ + centimeter_t(20), this->lastJuncDist_ + this->currentSeg_->length - centimeter_t(20)) &&
               1 == lineInfo.front.lines.size() && LinePattern::SINGLE_LINE == lineInfo.front.pattern.type                                     &&
               1 == lineInfo.rear.lines.size()  && LinePattern::SINGLE_LINE == lineInfo.rear.pattern.type                                      &&
               car.distance - this->lastSpeedSignChangeDistance_ >= centimeter_t(100)                                                          &&
               !(this->isLastTarget_ && this->currentSeg_ == this->laneChangeSeg_)) {
        controlData.speed = this->targetSpeedSign_ * this->targetFastSpeed_;

    } else {
        controlData.speed = this->targetSpeedSign_ * this->targetSpeed_;
    }

    if (car.distance < meter_t(1)) {
        controlData.speed = abs(controlData.speed);
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
    // Finds a valid previous segment - may be any of the segments behind the car, connecting to the current junction.
    Junction::segment_map::const_iterator sideSegments = junc.getSideSegments(negOri);
    if (sideSegments == junc.segments.end()) {
        sideSegments = junc.getSideSegments(normalize360(negOri + PI)); // if side segments are not found, tries the other orientation
    }
    const Segment *prevSeg = sideSegments->second.begin()->second;

    // Finds a valid junction connection - may be any of the segment's connections that are linked to the current junction.
    this->prevConn_ = *std::find_if(prevSeg->edges.begin(), prevSeg->edges.end(), [&junc](const Connection *c) {
        return c->junction->id == junc.id;
    });

    this->currentSeg_ = this->prevConn_->getOtherSegment(*prevSeg);
    this->targetDir_ = this->prevConn_->getDecision(*this->currentSeg_).direction;
}

void LabyrinthNavigator::updateRoute() {
    LOG_DEBUG("Updating route to: %c", this->targetSeg_->name);
    this->route_ = LabyrinthRoute::create(*this->prevConn_, *this->currentSeg_, *this->targetSeg_, true);

    LOG_DEBUG("Planned route:");

    const Segment *prev = this->route_.startSeg;
    for (const Connection *c : this->route_.connections) {
        const Segment *next = c->getOtherSegment(*prev);
        LOG_DEBUG("-> %c (%s)", next->name, to_string(c->getDecision(*next).direction));
        prev = next;
    }
}

bool LabyrinthNavigator::isTargetLineOverrideEnabled(const CarProps& car, const LineInfo& lineInfo) const {
    const LinePattern& frontPattern = this->frontLinePattern(lineInfo);
    return isJunction(frontPattern) && Sign::POSITIVE == frontPattern.dir;
}

bool LabyrinthNavigator::isDeadEnd(const micro::CarProps& car, const micro::LinePattern& pattern) const {
    return LinePattern::NONE == pattern.type && (this->currentSeg_->isDeadEnd || car.distance - pattern.startDist > centimeter_t(10));
}

bool LabyrinthNavigator::isJunction(const LinePattern& pattern) {
    return LinePattern::JUNCTION_1 == pattern.type ||
           LinePattern::JUNCTION_2 == pattern.type ||
           LinePattern::JUNCTION_3 == pattern.type;
}

uint8_t LabyrinthNavigator::numJunctionSegments(const LinePattern& pattern) {
    return isJunction(pattern) ? enum_cast(pattern.type) - enum_cast(LinePattern::JUNCTION_1) + 1 : 0;
}
