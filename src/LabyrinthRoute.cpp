#include <LabyrinthRoute.hpp>

using namespace micro;

constexpr uint32_t LabyrinthRoute::MAX_LENGTH;

LabyrinthRoute::LabyrinthRoute(const Segment *currentSeg)
    : startSeg(currentSeg)
    , destSeg(currentSeg) {}

void LabyrinthRoute::push_front(const Connection& c) {
    this->connections.push_front(&c);
    this->startSeg = c.getOtherSegment(*this->startSeg);
}

void LabyrinthRoute::push_back(const Connection& c) {
    this->connections.push_back(&c);
    this->destSeg = c.getOtherSegment(*this->destSeg);
}

void LabyrinthRoute::pop_front() {
    if (this->connections.size()) {
        this->startSeg = this->connections[0]->getOtherSegment(*startSeg);
        this->connections.erase(this->connections.begin());
    }
}

const Connection* LabyrinthRoute::firstConnection() const {
    return this->connections.size() > 0 ? this->connections[0] : nullptr;
}

const Connection* LabyrinthRoute::lastConnection() const {
    return this->connections.size() > 0 ? this->connections[this->connections.size() - 1] : nullptr;
}

void LabyrinthRoute::reset(const Segment& currentSeg) {
    this->startSeg = this->destSeg = &currentSeg;
    this->connections.clear();
}

bool LabyrinthRoute::isNewConnectionValid(const Connection& prevConn, const Segment& currentSeg, const Connection& newConn) {
    // does not permit going backwards or navigating through dead-end segments

    const bool isBwd     = newConn.junction == prevConn.junction && newConn.getDecision(currentSeg) == prevConn.getDecision(currentSeg);
    const bool isDeadEnd = newConn.getOtherSegment(currentSeg)->isDeadEnd;

    return !isBwd && !isDeadEnd;
}

LabyrinthRoute LabyrinthRoute::create(const Connection& prevConn, const Segment& currentSeg, const Segment& destSeg) {

    // performs Dijkstra-algorithm (https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm)
    // specifically tuned for forward-moving car in a graph that allows multiple connections between the nodes

    struct SegmentRouteInfo {
        const Segment *seg            = nullptr;
        meter_t dist                  = micro::numeric_limits<meter_t>::infinity();
        const Connection *prevConn    = nullptr;
        bool isDistMinimized          = false;
        SegmentRouteInfo *prevSegInfo = nullptr;
    };
    typedef micro::vec<SegmentRouteInfo, 2 * cfg::NUM_LABYRINTH_SEGMENTS> SegmentRouteInfos;

    SegmentRouteInfos info;

    info.push_back(SegmentRouteInfo{ &currentSeg, meter_t(0), &prevConn, false, nullptr });
    SegmentRouteInfos::iterator segInfo = info.begin();

    while (true) {
        segInfo = std::min_element(info.begin(), info.end(), [](const SegmentRouteInfo& a, const SegmentRouteInfo& b) {
            return a.isDistMinimized == b.isDistMinimized ? a.dist < b.dist : !a.isDistMinimized;
        });

        if (segInfo->seg == &destSeg) {
            break;
        } else if (segInfo->isDistMinimized) {
            // an error has occurred
            segInfo->prevSegInfo = nullptr;
            break;
        }

        for (Connection *newConn : segInfo->seg->edges) {
            if (isNewConnectionValid(*segInfo->prevConn, *segInfo->seg, *newConn)) {

                SegmentRouteInfo newSegInfo = {
                    newConn->getOtherSegment(*segInfo->seg),
                    segInfo->dist + segInfo->seg->length / 2 + newSegInfo.seg->length / 2,
                    newConn,
                    false,
                    segInfo
                };

                SegmentRouteInfos::iterator existingSegInfo = std::find_if(info.begin(), info.end(), [&newSegInfo](const SegmentRouteInfo& element) {
                    return element.seg == newSegInfo.seg                               &&
                           element.prevConn->junction == newSegInfo.prevConn->junction &&
                           element.prevConn->getDecision(*newSegInfo.seg) == newSegInfo.prevConn->getDecision(*newSegInfo.seg);
                });

                if (existingSegInfo != info.end()) {
                    if (newSegInfo.dist < existingSegInfo->dist) {
                        *existingSegInfo = newSegInfo;
                    }
                } else {
                    info.push_back(newSegInfo);
                }
            }
        }

        segInfo->isDistMinimized = true;
    }

    LabyrinthRoute route(&destSeg);

    while (segInfo->prevSegInfo) {
        route.push_front(*segInfo->prevConn);
        segInfo = segInfo->prevSegInfo;
    }

    return route;
}
