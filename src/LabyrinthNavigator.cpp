#include <micro/utils/log.hpp>

#include <LabyrinthNavigator.hpp>

using namespace micro;

LabyrinthNavigator::LabyrinthNavigator(const LabyrinthGraph& graph, const Connection& prevConn, const Segment& currentSeg)
    : graph_(graph)
    , prevConn_(&prevConn)
    , plannedRoute_(currentSeg)
    , lastJuncDist_(0)
    , targetDir_(Direction::CENTER) {}

const Segment* LabyrinthNavigator::currentSegment() const {
    return this->plannedRoute_.startSeg;
}

const Segment* LabyrinthNavigator::targetSegment() const {
    return this->plannedRoute_.destSeg;
}

const Connection* LabyrinthNavigator::nextConnection() const {
    return this->plannedRoute_.firstConnection();
}

void LabyrinthNavigator::setTargetSegment(const Segment& targetSeg) {
    LOG_DEBUG("Next target segment: %c", targetSeg.name);
    this->plannedRoute_ = LabyrinthRoute::create(*this->prevConn_, *this->plannedRoute_.startSeg, targetSeg);

    LOG_DEBUG("Planned route:");

    const Segment *prev = this->plannedRoute_.startSeg;
    for (const Connection *c : this->plannedRoute_.connections) {
        const Segment *next = c->getOtherSegment(*prev);
        LOG_DEBUG("-> %c (%s)", next->name, to_string(c->getManeuver(*next).direction));
        prev = next;
    }

    this->updateTargetDirection();
}

void LabyrinthNavigator::onJunctionDetected(const micro::meter_t distance) {
    this->lastJuncDist_ = distance;
    this->prevConn_     = this->plannedRoute_.firstConnection();
    this->plannedRoute_.pop_front();
}

Direction LabyrinthNavigator::update(const micro::meter_t distance) {
    if (distance - this->lastJuncDist_ >= this->plannedRoute_.startSeg->length / 2) {
        this->updateTargetDirection();
    }

    return this->targetDir_;
}

void LabyrinthNavigator::reset(const Connection& prevConn, const Segment& currentSeg) {
    this->prevConn_ = &prevConn;
    this->plannedRoute_.reset(currentSeg);
    this->setTargetSegment(*this->plannedRoute_.destSeg);   // rebuilds route
}

void LabyrinthNavigator::updateTargetDirection() {
    const Connection *nextConn = this->plannedRoute_.firstConnection();
    this->targetDir_ = nextConn ? nextConn->getManeuver(*nextConn->getOtherSegment(*this->plannedRoute_.startSeg)).direction : Direction::CENTER;
    LOG_DEBUG("Target direction: %s", to_string(this->targetDir_));
}
