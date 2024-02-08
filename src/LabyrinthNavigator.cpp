#include <micro/container/vector.hpp>
#include <micro/log/log.hpp>
#include <micro/math/numeric.hpp>
#include <micro/port/timer.hpp>
#include <micro/utils/LinePattern.hpp>

#include <cfg_car.hpp>
#include <LabyrinthNavigator.hpp>

using namespace micro;

// Offset between checkpoint center and car center
const auto CHECKPOINT_OFFSET = micro::point2<micro::meter_t>(
    centimeter_t(10) + cfg::CAR_FRONT_REAR_SENSOR_ROW_DIST / 2,
    meter_t(0)
);

char LabyrinthNavigator::ObstaclePosition::prevJunction() const {
    if (!current || !next) {
        return '\0';
    }

    const auto c0 = current->id[0], c1 = current->id[1];
    const auto n0 = next->id[0], n1 = next->id[1];

    return (c0 == n0 || c0 == n1) ? c1 : c0;
}

char LabyrinthNavigator::ObstaclePosition::nextJunction() const {
    if (!current || !next) {
        return '\0';
    }

    const auto c0 = current->id[0], c1 = current->id[1];
    const auto n0 = next->id[0], n1 = next->id[1];

    return (c0 == n0 || c0 == n1) ? c0 : c1;
}

LabyrinthNavigator::LabyrinthNavigator(const LabyrinthGraph& graph, micro::irandom_generator& random)
    : Maneuver()
    , graph_(graph)
    , lastJuncDist_(0)
    , targetDir_(Direction::CENTER)
    , targetSpeedSign_(Sign::POSITIVE)
    , hasSpeedSignChanged_(false)
    , isInJunction_(false)
    , random_(random) {}

void LabyrinthNavigator::initialize(
    const SegmentIds& unvisitedSegments,
    const SegmentIds& forbiddenSegments,
    const Segment *currentSeg,
    const Connection *prevConn,
    const Segment *laneChangeSeg,
    const Junction *lastJunctionBeforeLaneChange,
    const micro::m_per_sec_t targetSpeed,
    const micro::m_per_sec_t targetDeadEndSpeed) {
    unvisitedSegments_ = unvisitedSegments;
    forbiddenSegments_ = forbiddenSegments;
    currentSeg_ = currentSeg;
    prevConn_ = prevConn;
    laneChangeSeg_ = laneChangeSeg;
    lastJunctionBeforeLaneChange_ = lastJunctionBeforeLaneChange;
    targetSpeed_ = targetSpeed;
    targetDeadEndSpeed_ = targetDeadEndSpeed;

    unvisitedSegments_.erase(currentSeg_->id);
}

const micro::Pose& LabyrinthNavigator::correctedCarPose() const {
    return correctedCarPose_;
}

void LabyrinthNavigator::setObstaclePosition(const ObstaclePosition& obstaclePosition) {
    if (obstaclePosition_.current != obstaclePosition.current || obstaclePosition_.next != obstaclePosition.next) {
        obstaclePosition_ = obstaclePosition;
        LOG_INFO("Obstacle position: current: {}, next: {}]",
            obstaclePosition_.current ? obstaclePosition_.current->id : "NULL",
            obstaclePosition_.next ? obstaclePosition_.next->id : "NULL");
    }
}

void LabyrinthNavigator::update(const micro::CarProps& car, const micro::LineInfo& lineInfo, micro::MainLine& mainLine, micro::ControlData& controlData) {
    correctedCarPose_ = car.pose;

    updateCarOrientation(car, lineInfo);

    const auto& prevFrontPattern = frontLinePattern(prevLineInfo_);
    const auto& frontPattern     = frontLinePattern(lineInfo);
    const auto& lines            = frontLines(lineInfo);

    const auto& prevRearPattern = rearLinePattern(prevLineInfo_);
    const auto& rearPattern     = rearLinePattern(lineInfo);

    if (4 == lines.size() && car.distance - lastPosUpdateDist_ > centimeter_t(50)) {
        if (const auto* junction = findExpectedJunction(); junction && car.pose.pos.distance(junction->pos) < centimeter_t(100)) {
            correctedCarPose_.pos = junction->pos - CHECKPOINT_OFFSET.rotate(car.pose.angle);
            lastPosUpdateDist_ = car.distance;
        }
    }

    if (frontPattern != prevFrontPattern) {
        if (isJunction(frontPattern) && Sign::POSITIVE == frontPattern.dir) {
            // car is coming out of a junction
            handleJunction(car, frontPattern.type);
            isInJunction_ = true;
        }
    }

    if (rearPattern != prevRearPattern) {
        if (LinePattern::SINGLE_LINE == rearPattern.type) {
            isInJunction_ = false;
        }
    }

    if (const auto result = isSpeedSignChangeNeeded(car, frontPattern); result.first) {
        hasSpeedSignChanged_ = true;
        targetSpeedSign_ = -targetSpeedSign_;
        lastSpeedSignChangeDistance_ = car.distance;
        LOG_INFO("Labyrinth target speed sign changed to {}. Reason: {}", to_string(targetSpeedSign_), result.second);
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
    lastJunctionBeforeTargetSeg_ = lastJunctionBeforeLaneChange_;
}

const micro::LinePattern& LabyrinthNavigator::frontLinePattern(const micro::LineInfo& lineInfo) const {
    return Sign::POSITIVE == targetSpeedSign_ ? lineInfo.front.pattern : lineInfo.rear.pattern;
}

const micro::LinePattern& LabyrinthNavigator::rearLinePattern(const micro::LineInfo& lineInfo) const {
    return Sign::POSITIVE == targetSpeedSign_ ? lineInfo.rear.pattern : lineInfo.front.pattern;
}

const micro::Lines& LabyrinthNavigator::frontLines(const micro::LineInfo& lineInfo) const {
    return Sign::POSITIVE == targetSpeedSign_ ? lineInfo.front.lines : lineInfo.rear.lines;
}

const micro::Lines& LabyrinthNavigator::rearLines(const micro::LineInfo& lineInfo) const {
    return Sign::POSITIVE == targetSpeedSign_ ? lineInfo.rear.lines : lineInfo.front.lines;
}

std::pair<bool, const char*> LabyrinthNavigator::isSpeedSignChangeNeeded(const micro::CarProps& car, const micro::LinePattern& frontPattern) const {
    if (hasSpeedSignChanged_ || isInJunction_ || !prevConn_) {
        return {false, nullptr};
    }

    if (isDeadEnd(car, frontPattern)) {
        return {true, "DEAD_END"};
    }

    if (currentSeg_ == obstaclePosition_.current) {
        const auto sameDirection = prevConn_->junction->id == obstaclePosition_.prevJunction();
        const auto obstacleEnteredLater = currentSegStartTime_ < obstaclePosition_.lastUpdateTime;

        if (!(sameDirection && obstacleEnteredLater)) {
            return {true, "OBSTACLE_IN_CURRENT_SEGMENT"};
        }
    }
    
    if (currentSeg_ == obstaclePosition_.next && prevConn_->junction->id != obstaclePosition_.nextJunction()) {
        return {true, "OBSTACLE_IN_NEXT_SEGMENT"};
    }

    const Connection *nextConn = route_.firstConnection();
    if (nextConn && nextConn->junction == prevConn_->junction && !currentSeg_->isLoop()) {
        return {true, "BACKWARD_ROUTE"};
    }

    return {false, nullptr};
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

void LabyrinthNavigator::handleJunction(const CarProps& car, const micro::LinePattern::type_t patternType) {
    const radian_t posOri = round90(car.speed >= m_per_sec_t(0) ? car.pose.angle : car.pose.angle + PI);
    const radian_t negOri = round90(posOri + PI);

    LOG_DEBUG("Junction detected (car pos: ({}, {}), angle: {} deg)",
        car.pose.pos.X.get(),
        car.pose.pos.Y.get(),
        static_cast<degree_t>(car.pose.angle).get());

    lastJuncDist_ = car.distance;
    hasSpeedSignChanged_ = false;

    const auto* junc = [this, &car]() -> const Junction* {
        const auto* expectedJunction = findExpectedJunction();
        if (expectedJunction && car.pose.pos.distance(expectedJunction->pos) < centimeter_t(120)) {
            return expectedJunction;
        }

        const auto* closestJunction = graph_.findClosestJunction(car.pose.pos);
        if (closestJunction && car.pose.pos.distance(closestJunction->pos) < centimeter_t(80)) {
            return closestJunction;
        }

        return nullptr;
    }();

    // checks if any junction has been found at the current position
    if (!junc) {
        LOG_ERROR("Junction not found, chooses target direction randomly.");
        targetDir_ = randomDirection(patternType);
        return;
    }

    LOG_DEBUG("Junction found: {}, current segment: {}", junc->id, currentSeg_->id);

    // checks if the current segment connects to the found junction
    if (junc->getConnectionCount(*currentSeg_) == 0) {
        LOG_ERROR("Current segment does not connect to found junction.");
        reset(*junc, negOri);
    }

    if (targetSeg_ && (targetSeg_ != route_.destSeg || currentSeg_ != route_.startSeg)) {
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
        LOG_DEBUG("No route defined, choosing randomly");
        nextConn = randomConnection(junction, *currentSeg_);
    }

    if (!nextConn) {
        LOG_WARN("Could not find a valid next connection");
        return;
    }

    currentSeg_ = nextConn->getOtherSegment(*currentSeg_);
    targetDir_  = nextConn->getDecision(*currentSeg_).direction;
    prevConn_   = nextConn;
    currentSegStartTime_ = micro::getTime();
    unvisitedSegments_.erase(currentSeg_->id);
    LOG_INFO("Stepping to next segment: {}. Target direction: {}", currentSeg_->id, to_string(targetDir_));
}

const Junction* LabyrinthNavigator::findExpectedJunction() const {
    if (!prevConn_ || !currentSeg_) {
        return nullptr;
    }

    const auto it = std::find_if(currentSeg_->edges.begin(), currentSeg_->edges.end(),
        [this](const auto& c) { return c->junction != prevConn_->junction; });

    return it != currentSeg_->edges.end() ? (*it)->junction : nullptr;
}

void LabyrinthNavigator::setTargetLine(const micro::CarProps& car, const micro::LineInfo& lineInfo, micro::MainLine& mainLine) const {
    const LinePattern& frontPattern = frontLinePattern(lineInfo);

    // target line is only overwritten when the car is going in or coming out of a junction
    if (isJunction(frontPattern) && Sign::POSITIVE == frontPattern.dir) {
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
    const auto speed = targetSpeedSign_ * (targetSeg_ == laneChangeSeg_ && currentSeg_ == targetSeg_ ? targetSpeed_ : targetDeadEndSpeed_);


    if (std::exchange(controlData.speed, speed) != speed) {
        LOG_DEBUG("Target speed changed to {}m/s", controlData.speed.get());
    }

    controlData.rampTime = millisecond_t(350);

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

    prevConn_ = *std::find_if(currentSeg_->edges.begin(), currentSeg_->edges.end(),
        [&junc](const auto& conn) { return conn->junction != &junc; });
    
    currentSegStartTime_ = micro::getTime();
    route_.reset();

    LOG_INFO("Navigator reset to the junction: {}. Current segment: {}", junc.id, currentSeg_->id);
}

void LabyrinthNavigator::createRoute() {
    constexpr auto ALLOW_BACKWARD_NAVIGATION = false;

    LOG_INFO("Updating route to: {}", targetSeg_->id);

    route_ = LabyrinthRoute::create(
        *prevConn_,
        *currentSeg_,
        *targetSeg_,
        *lastJunctionBeforeTargetSeg_,
        {obstaclePosition_.prevJunction(), obstaclePosition_.nextJunction()},
        ALLOW_BACKWARD_NAVIGATION);

    LOG_DEBUG("Planned route:");

    const Segment *prev = route_.startSeg;
    for (const Connection *c : route_.connections) {
        const Segment *next = c->getOtherSegment(*prev);
        LOG_DEBUG("-> {} ({})", next->id, to_string(c->getDecision(*next).direction));
        prev = next;
    }
}

bool LabyrinthNavigator::isDeadEnd(const micro::CarProps& car, const micro::LinePattern& pattern) const {
    return LinePattern::NONE == pattern.type && (currentSeg_->isDeadEnd || car.distance - pattern.startDist > centimeter_t(5));
}

const Connection* LabyrinthNavigator::randomConnection(const Junction& junc, const Segment& seg) {
    using SideConnections = micro::vector<const Connection*, cfg::MAX_NUM_CROSSING_SEGMENTS_SIDE>;
    SideConnections allConnections, allowedConnections, unvisitedConnections, obstacleFreeConnections, unvisitedObstacleFreeConnections;

    for (Connection *c : seg.edges) {
        const auto* otherSeg = c->getOtherSegment(seg);
        
        if (c->junction->id != junc.id || otherSeg->isDeadEnd) {
            continue;
        }

        allConnections.push_back(c);

        if (forbiddenSegments_.contains(otherSeg->id)) {
            continue;
        }

        allowedConnections.push_back(c);

        const auto obstacleFree = std::all_of(otherSeg->edges.begin(), otherSeg->edges.end(),
                [this, &c](const auto& otherConn) {
                    // Segment is allowed only if the other end of the segment is not a restricted junction.
                    return (c->junction == otherConn->junction) ||
                        (otherConn->junction->id != obstaclePosition_.prevJunction() &&
                        otherConn->junction->id != obstaclePosition_.nextJunction());
                });

        const auto unvisited = unvisitedSegments_.contains(otherSeg->id);

        if (obstacleFree) {
            obstacleFreeConnections.push_back(c);
        }

        if (unvisited) {
            unvisitedConnections.push_back(c);
            if (obstacleFree) {
                unvisitedObstacleFreeConnections.push_back(c);
            }
        }
    }

    if (allConnections.empty()) {
        LOG_DEBUG("No connections available");
        return nullptr;
    }

    auto& connections = [&]() -> SideConnections& {
        if (!unvisitedObstacleFreeConnections.empty()) {
            LOG_DEBUG("Choosing from unvisited obstacle-free connections");
            return unvisitedObstacleFreeConnections;
        }

        LOG_DEBUG("No unvisited obstacle-free connections");

        if (!obstacleFreeConnections.empty()) {
            LOG_DEBUG("Choosing from obstacle-free connections");
            return obstacleFreeConnections;
        }

        LOG_DEBUG("No obstacle-free connections");

        if (!unvisitedConnections.empty()) {
            LOG_DEBUG("Choosing from unvisited connections");
            return unvisitedConnections;
        }

        LOG_DEBUG("No unvisited connections");

        if (!allowedConnections.empty()) {
            LOG_DEBUG("Choosing from allowed connections");
            return allowedConnections;
        }

        LOG_DEBUG("No allowed connections. Choosing from all connections");
        return allConnections;
    }();

    std::sort(connections.begin(), connections.end(), [&seg](const auto& a, const auto& b){
        return a->getDecision(*a->getOtherSegment(seg)) < b->getDecision(*b->getOtherSegment(seg));
    });
    
    return connections[static_cast<size_t>(random_() * connections.size())];
}

Direction LabyrinthNavigator::randomDirection(const micro::LinePattern::type_t patternType) {
    const auto numSegments = micro::underlying_value(patternType) - micro::underlying_value(LinePattern::JUNCTION_1) + 1;
    const auto targetLineIdx = static_cast<uint8_t>(random_() * numSegments);

    switch (numSegments) {
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
