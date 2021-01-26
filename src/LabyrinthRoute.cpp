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

bool LabyrinthRoute::isForwardConnection(const Connection& prevConn, const Segment& currentSeg, const Connection& newConn) {
    // does not permit going backwards
    const bool isBwd = newConn.junction == prevConn.junction && newConn.getDecision(currentSeg) == prevConn.getDecision(currentSeg);
    return !isBwd;
}

LabyrinthRoute LabyrinthRoute::create(const Connection& prevConn, const Segment& currentSeg, const Segment& destSeg, const bool allowBackwardNavigation) {

    // performs Dijkstra-algorithm (https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm)
    // specifically tuned for forward-moving car in a graph that allows multiple connections between the nodes

    struct SegmentRouteInfo {
        const Segment *seg            = nullptr;
        meter_t dist                  = micro::numeric_limits<meter_t>::infinity();
        const Connection *prevConn    = nullptr;
        bool isDistMinimized          = false;
        SegmentRouteInfo *prevSegInfo = nullptr;
    };
    typedef micro::vec<SegmentRouteInfo, 2 * cfg::MAX_NUM_LABYRINTH_SEGMENTS> SegmentRouteInfos;

    SegmentRouteInfos info;

    info.push_back(SegmentRouteInfo{ &currentSeg, meter_t(0), &prevConn, false, nullptr });
    SegmentRouteInfos::iterator segInfo = info.begin();

    while (true) {
        segInfo = std::min_element(info.begin(), info.end(), [](const SegmentRouteInfo& a, const SegmentRouteInfo& b) {
            // nodes with already minimized distance cannot be selected as minimum value
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
            if (allowBackwardNavigation || isForwardConnection(*segInfo->prevConn, *segInfo->seg, *newConn)) {

                SegmentRouteInfo newSegInfo;
                newSegInfo.seg             = newConn->getOtherSegment(*segInfo->seg);
                newSegInfo.prevConn        = newConn;
                newSegInfo.isDistMinimized = false;
                newSegInfo.prevSegInfo     = segInfo;

                // when going back to the previous junction, distance is not the same as when passing through the whole segment
                if (segInfo != info.begin() && allowBackwardNavigation && segInfo->prevConn->junction == newConn->junction) {
                    newSegInfo.dist = segInfo->dist - segInfo->seg->length / 2 + meter_t(1.2f) + newSegInfo.seg->length / 2;
                } else {
                    newSegInfo.dist = segInfo->dist + segInfo->seg->length / 2 + newSegInfo.seg->length / 2;
                }

                SegmentRouteInfos::iterator existingSegInfo = std::find_if(info.begin(), info.end(), [&newSegInfo, allowBackwardNavigation](const SegmentRouteInfo& element) {
                    return element.seg == newSegInfo.seg &&
                           (allowBackwardNavigation ||
                               (element.prevConn->junction == newSegInfo.prevConn->junction &&
                                element.prevConn->getDecision(*newSegInfo.seg) == newSegInfo.prevConn->getDecision(*newSegInfo.seg)));
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
