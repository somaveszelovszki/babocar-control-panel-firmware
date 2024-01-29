#include <micro/container/vector.hpp>
#include <micro/log/log.hpp>
#include <micro/math/numeric.hpp>

#include <LabyrinthNavigator.hpp>

using namespace micro;

LabyrinthNavigator::LabyrinthNavigator(const LabyrinthGraph& graph, micro::irandom_generator& random)
    : Maneuver()
    , graph_(graph)
    , lastJuncDist_(0)
    , targetDir_(Direction::CENTER)
    , targetSpeedSign_(Sign::POSITIVE)
    , isSpeedSignChangeInProgress_(false)
    , hasSpeedSignChanged_(false)
    , isInJunction_(false)
    , random_(random) {}

void LabyrinthNavigator::initialize(
    const micro::set<Segment::Id, cfg::MAX_NUM_LABYRINTH_SEGMENTS>& unvisitedSegments,
    const Segment *currentSeg,
    const Connection *prevConn,
    const Segment *laneChangeSeg,
    const Segment *floodSeg,
    const micro::m_per_sec_t targetSpeed,
    const micro::m_per_sec_t targetFastSpeed,
    const micro::m_per_sec_t targetDeadEndSpeed) {
    unvisitedSegments_ = unvisitedSegments;
    currentSeg_ = currentSeg;
    prevConn_ = prevConn;
    laneChangeSeg_ = laneChangeSeg;
    floodSeg_ = floodSeg;
    targetSpeed_ = targetSpeed;
    targetFastSpeed_ = targetFastSpeed;
    targetDeadEndSpeed_ = targetDeadEndSpeed;

    unvisitedSegments_.erase(currentSeg_->id);
}

const micro::Pose& LabyrinthNavigator::correctedCarPose() const {
    return correctedCarPose_;
}

void LabyrinthNavigator::setForbiddenSegment(const Segment* segment) {
    forbiddenSegment_ = segment;
    LOG_INFO("Forbidden segment set to: ", forbiddenSegment_ ? forbiddenSegment_->id : "NULL");
}

void LabyrinthNavigator::setFlood(const bool flood) {
    const auto currentFlood = targetSeg_ == floodSeg_;
    if (currentFlood == flood) {
        return;
    }

    LOG_INFO("Flood {}", flood ? "activated" : "deactivated");

    if (flood) {
        LOG_INFO("Flood activated");
        targetSeg_ = floodSeg_;
    } else {
        LOG_INFO("Flood deactivated");
        targetSeg_ = nullptr;
        route_.reset();
    }
}

void LabyrinthNavigator::update(const micro::CarProps& car, const micro::LineInfo& lineInfo, micro::MainLine& mainLine, micro::ControlData& controlData) {
    correctedCarPose_ = car.pose;

    updateCarOrientation(car, lineInfo);

    const LinePattern& prevFrontPattern = frontLinePattern(prevLineInfo_);
    const LinePattern& prevRearPattern  = rearLinePattern(prevLineInfo_);
    const LinePattern& frontPattern     = frontLinePattern(lineInfo);
    const LinePattern& rearPattern      = rearLinePattern(lineInfo);

    // does not handle pattern changes while car is changing speed sign
    if (isSpeedSignChangeInProgress_) {
        if (sgn(car.speed) == targetSpeedSign_ && LinePattern::SINGLE_LINE == frontPattern.type) {
            isSpeedSignChangeInProgress_ = false;
            LOG_DEBUG("Speed sign change finished");
        }
    } else {
        if (frontPattern != prevFrontPattern) {
            if (isJunction(frontPattern) && Sign::POSITIVE == frontPattern.dir) {
                // car is coming out of a junction
                handleJunction(car, numJunctionSegments(prevFrontPattern), numJunctionSegments(frontPattern));
                isInJunction_ = true;
            }
        }

        if (rearPattern != prevRearPattern) {
            if (LinePattern::SINGLE_LINE == rearPattern.type) {
                isInJunction_ = false;
            }
        }

        // start going backward when a dead-end sign is detected
        if (isDeadEnd(car, frontPattern) && !isDeadEnd(car, prevFrontPattern)) {
            LOG_ERROR("Dead-end detected! Labyrinth target speed sign changed to {}", to_string(targetSpeedSign_));
            tryToggleTargetSpeedSign(car.distance);
        }
    }

    // Checks if car needs to change speed sign in order to follow route.
    // @note This is only enabled when the car is not in a junction.
    if (!isInJunction_) {
        const Connection *nextConn = route_.firstConnection();
        if (nextConn && prevConn_ && nextConn->junction == prevConn_->junction && !currentSeg_->isLoop()) {
            tryToggleTargetSpeedSign(car.distance);
        }
    }

    // If the car is in the flood segment but the flood is deactivated,
    // the car needs to change speed sign to reversse back into the labyrinth.
    if (currentSeg_ == floodSeg_ && targetSeg_ != floodSeg_) {
        tryToggleTargetSpeedSign(car.distance);
    }

    setControl(car, lineInfo, mainLine, controlData);

    prevLineInfo_ = lineInfo;

    if (targetSeg_ == laneChangeSeg_ && (LinePattern::LANE_CHANGE == frontPattern.type || LinePattern::LANE_CHANGE == rearPattern.type)) {
        finish();
    }
}

void LabyrinthNavigator::navigateToLaneChange() {
    LOG_INFO("Navigating to the lane change segment");
    targetSeg_ = laneChangeSeg_;
}

const micro::LinePattern& LabyrinthNavigator::frontLinePattern(const micro::LineInfo& lineInfo) const {
    return Sign::POSITIVE == targetSpeedSign_ ? lineInfo.front.pattern : lineInfo.rear.pattern;
}

const micro::LinePattern& LabyrinthNavigator::rearLinePattern(const micro::LineInfo& lineInfo) const {
    return Sign::POSITIVE == targetSpeedSign_ ? lineInfo.rear.pattern : lineInfo.front.pattern;
}

void LabyrinthNavigator::updateCarOrientation(const CarProps& car, const LineInfo& lineInfo) {
    const LinePattern& frontPattern = frontLinePattern(lineInfo);
    if (car.orientedDistance > centimeter_t(60)                       &&
        car.distance - lastOrientationUpdateDist_ > centimeter_t(100) &&
        LinePattern::SINGLE_LINE == frontPattern.type                 &&
        car.distance - frontPattern.startDist > centimeter_t(100)     &&
        eqWithOverflow360(car.pose.angle, round90(car.pose.angle), degree_t(10))) {

        correctedCarPose_.angle = round90(car.pose.angle);
        lastOrientationUpdateDist_ = car.distance;
    }
}

void LabyrinthNavigator::handleJunction(const CarProps& car, uint8_t numInSegments, uint8_t numOutSegments) {
    const radian_t posOri = round90(car.speed >= m_per_sec_t(0) ? car.pose.angle : car.pose.angle + PI);
    const radian_t negOri = round90(posOri + PI);

    const micro::vector<std::pair<micro::radian_t, uint8_t>, 2> numSegments = {
        { negOri, numInSegments  },
        { posOri, numOutSegments }
    };

    LOG_DEBUG("Junction detected (car pos: ({}, {}), angle: {} deg, segments: (in: {}, out: {}))",
        car.pose.pos.X.get(),
        car.pose.pos.Y.get(),
        static_cast<degree_t>(car.pose.angle).get(),
        static_cast<uint32_t>(numInSegments),
        static_cast<uint32_t>(numOutSegments));

    lastJuncDist_ = car.distance;
    hasSpeedSignChanged_ = false;

    const Junction *junc = graph_.findJunction(car.pose.pos, numSegments);

    // checks if any junction has been found at the current position
    if (!junc) {
        LOG_ERROR("Junction not found, chooses target direction randomly.");
        targetDir_ = randomDirection(numOutSegments);
        return;
    }

    LOG_DEBUG("Junction found: {}, current segment: {}", junc->id, currentSeg_->id);

    correctedCarPose_.pos = junc->pos;

    // checks if the current segment connects to the found junction
    if (junc->getConnectionCount(*currentSeg_) == 0) {
        LOG_ERROR("Current segment does not connect to found junction.");
        reset(*junc, negOri);
    }

    if (targetSeg_ != route_.destSeg) {
        createRoute();
    }

    stepToNextSegment(*junc);
}

void LabyrinthNavigator::stepToNextSegment(const Junction& junction) {
    const auto* nextConn = route_.firstConnection();
    if (nextConn) {
        LOG_DEBUG("Route connection available");
        route_.pop_front();
    } else {
        LOG_DEBUG("No route defined, choosing randomly from unvisited sections");
        nextConn = randomConnection(junction, *currentSeg_);
    }

    if (!nextConn) {
        LOG_WARN("Could not find a valid next connection");
        return;
    }

    currentSeg_ = nextConn->getOtherSegment(*currentSeg_);
    targetDir_  = nextConn->getDecision(*currentSeg_).direction;
    prevConn_   = nextConn;
    unvisitedSegments_.erase(currentSeg_->id);
    LOG_INFO("Stepping to next segment: {}. Target direction: {}", currentSeg_->id, to_string(targetDir_));
}

void LabyrinthNavigator::tryToggleTargetSpeedSign(const micro::meter_t currentDist) {
    if (!hasSpeedSignChanged_) {
        targetSpeedSign_             = -targetSpeedSign_;
        isSpeedSignChangeInProgress_ = true;
        hasSpeedSignChanged_         = true;
        lastSpeedSignChangeDistance_ = currentDist;
        LOG_INFO("Labyrinth target speed sign changed to {}", to_string(targetSpeedSign_));
    }
}

void LabyrinthNavigator::setTargetLine(const micro::CarProps& car, const micro::LineInfo& lineInfo, micro::MainLine& mainLine) const {
    // target line is only overwritten when the car is going in or coming out of a junction
    if (isTargetLineOverrideEnabled(car, lineInfo)) {
        switch (targetSpeedSign_ * targetDir_) {
        case Direction::LEFT:
            if (!lineInfo.front.lines.empty()) {
                mainLine.frontLine = *lineInfo.front.lines.begin();
            }
            if (!lineInfo.rear.lines.empty()) {
                mainLine.rearLine = *lineInfo.rear.lines.rbegin();
            }
            break;

        case Direction::CENTER:
            if (3 == lineInfo.front.lines.size()) {
                mainLine.frontLine = *std::next(lineInfo.front.lines.begin());
            }
            if (3 == lineInfo.rear.lines.size()) {
                mainLine.rearLine = *std::next(lineInfo.rear.lines.begin());
            }
            break;

        case Direction::RIGHT:
            if (lineInfo.front.lines.size()) {
                mainLine.frontLine = *lineInfo.front.lines.rbegin();
            }
            if (lineInfo.rear.lines.size()) {
                mainLine.rearLine = *lineInfo.rear.lines.begin();
            }
            break;
        }

        mainLine.updateCenterLine();
    }
}

void LabyrinthNavigator::setControl(const micro::CarProps& car, const micro::LineInfo& lineInfo, micro::MainLine& mainLine, micro::ControlData& controlData) const {
    const m_per_sec_t prevSpeed = controlData.speed;

    const auto speed = [this, &car, &lineInfo]() {
        if (currentSeg_ == floodSeg_ && !hasSpeedSignChanged_) {
           return targetDeadEndSpeed_;
        }

        if (currentSeg_->isDeadEnd && !hasSpeedSignChanged_) {
            const auto slowSectionLength = currentSeg_ == floodSeg_ ? centimeter_t(250) : centimeter_t(100);
            if (car.distance - lastJuncDist_ > floodSeg_->length - slowSectionLength) {
                return targetDeadEndSpeed_;
            }
        }

        if (isBtw(car.distance, lastJuncDist_ + centimeter_t(20), lastJuncDist_ + currentSeg_->length - centimeter_t(20)) &&
            1 == lineInfo.front.lines.size() && LinePattern::SINGLE_LINE == lineInfo.front.pattern.type                   &&
            1 == lineInfo.rear.lines.size()  && LinePattern::SINGLE_LINE == lineInfo.rear.pattern.type                    &&
            car.distance - lastSpeedSignChangeDistance_ >= centimeter_t(100)                                              &&
            currentSeg_ != targetSeg_) {
            return targetFastSpeed_;
        }

        return targetSpeed_;
    }();

    controlData.speed = targetSpeedSign_ * speed;

    if (car.distance < meter_t(1)) {
        controlData.speed = abs(controlData.speed);
    }

    controlData.rampTime = millisecond_t(300);

    if (controlData.speed != prevSpeed) {
        LOG_DEBUG("Target speed changed to {}m/s", controlData.speed.get());
    }

    setTargetLine(car, lineInfo, mainLine);

    // enables rear wheel steering when coming out of a junction
    controlData.rearSteerEnabled   = true;
    controlData.lineControl.actual = mainLine.centerLine;
    controlData.lineControl.target = { millimeter_t(0), radian_t(0) };
}

void LabyrinthNavigator::reset(const Junction& junc, radian_t negOri) {
    // Finds a valid previous segment - may be any of the segments behind the car, connecting to the current junction.
    auto* sideSegments = junc.getSideSegments(negOri);
    if (!sideSegments) {
        // if side segments are not found, tries the other orientation
        sideSegments = junc.getSideSegments(micro::normalize360(negOri + PI));
    }

    currentSeg_ = sideSegments->begin()->second;
    route_.reset();

    LOG_INFO("Navigator reset to the junction: {}. Current segment: {}", junc.id, currentSeg_->id);
}

void LabyrinthNavigator::createRoute() {
    LOG_INFO("Updating route to: {}", targetSeg_->id);

    const auto forbiddenJunctions = forbiddenSegment_
        ? JunctionIds{forbiddenSegment_->id[0], forbiddenSegment_->id[1]}
        : JunctionIds{};

    route_ = LabyrinthRoute::create(*prevConn_, *currentSeg_, *targetSeg_, forbiddenJunctions, false);

    LOG_DEBUG("Planned route:");

    const Segment *prev = route_.startSeg;
    for (const Connection *c : route_.connections) {
        const Segment *next = c->getOtherSegment(*prev);
        LOG_DEBUG("-> {} ({})", next->id, to_string(c->getDecision(*next).direction));
        prev = next;
    }
}

bool LabyrinthNavigator::isTargetLineOverrideEnabled(const CarProps& car, const LineInfo& lineInfo) const {
    const LinePattern& frontPattern = frontLinePattern(lineInfo);
    return isJunction(frontPattern) && Sign::POSITIVE == frontPattern.dir;
}

bool LabyrinthNavigator::isDeadEnd(const micro::CarProps& car, const micro::LinePattern& pattern) const {
    return LinePattern::NONE == pattern.type && (currentSeg_->isDeadEnd || car.distance - pattern.startDist > centimeter_t(10));
}

const Connection* LabyrinthNavigator::randomConnection(const Junction& junc, const Segment& seg) {
    micro::vector<const Connection*, cfg::MAX_NUM_CROSSING_SEGMENTS_SIDE> allowedConnections, unvisitedConnections;

    for (Connection *c : seg.edges) {
        const auto* otherSeg = c->getOtherSegment(seg);
        const auto isAllowed = c->junction->id == junc.id &&
                               !otherSeg->isDeadEnd &&
                               (!forbiddenSegment_ || !graph_.findConnection(*otherSeg, *forbiddenSegment_));
        if (isAllowed) {
            allowedConnections.push_back(c);
            if (unvisitedSegments_.contains(otherSeg->id)) {
                unvisitedConnections.push_back(c);
            }
        }
    }

    if (allowedConnections.empty()) {
        return nullptr;
    }

    auto& connections = !unvisitedConnections.empty() ? unvisitedConnections : allowedConnections;
    std::sort(connections.begin(), connections.end(), [&seg](const auto& a, const auto& b){
        return a->getDecision(*a->getOtherSegment(seg)) < b->getDecision(*b->getOtherSegment(seg));
    });
    
    return connections[static_cast<size_t>(random_() * connections.size())];
}

Direction LabyrinthNavigator::randomDirection(const uint8_t numOutSegments) {
    const auto targetLineIdx = static_cast<uint8_t>(random_() * numOutSegments);

    switch (numOutSegments) {
    case 3:
        return 0 == targetLineIdx ? Direction::RIGHT : 1 == targetLineIdx ? Direction::CENTER : Direction::LEFT;
    case 2:
        return 0 == targetLineIdx ? Direction::RIGHT : Direction::LEFT;
    default:
        return Direction::CENTER;
    }
}

bool LabyrinthNavigator::isJunction(const LinePattern& pattern) {
    return LinePattern::JUNCTION_1 == pattern.type ||
           LinePattern::JUNCTION_2 == pattern.type ||
           LinePattern::JUNCTION_3 == pattern.type;
}

uint8_t LabyrinthNavigator::numJunctionSegments(const LinePattern& pattern) {
    return isJunction(pattern) ? underlying_value(pattern.type) - underlying_value(LinePattern::JUNCTION_1) + 1 : 0;
}
