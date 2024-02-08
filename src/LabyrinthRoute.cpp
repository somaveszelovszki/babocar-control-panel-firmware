#include <micro/container/vector.hpp>

#include <LabyrinthRoute.hpp>

using namespace micro;

LabyrinthRoute::LabyrinthRoute(const Segment *currentSeg)
    : startSeg(currentSeg)
    , destSeg(currentSeg) {}

void LabyrinthRoute::push_front(const Connection& c) {
    this->connections.insert(this->connections.begin(), &c);
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

void LabyrinthRoute::reset() {
    this->startSeg = this->destSeg = nullptr;
    this->connections.clear();
}

bool LabyrinthRoute::isForwardConnection(const Connection& prevConn, const Segment& currentSeg, const Connection& newConn) {
    const bool isBwd = newConn.junction == prevConn.junction && newConn.getDecision(currentSeg) == prevConn.getDecision(currentSeg);
    return !isBwd;
}


LabyrinthRoute LabyrinthRoute::create(
    const Connection& prevConn,
    const Segment& currentSeg,
    const Segment& destSeg,
    const Junction& lastJunction,
    const JunctionIds& restrictedJunctions,
    const bool allowBackwardNavigation) {
    // performs Dijkstra-algorithm (https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm)
    // specifically tuned for a car in a graph that allows multiple connections between the nodes

    struct SegmentRouteInfo {
        const Segment *seg            = nullptr;
        const Connection* prevConn    = nullptr;
        SegmentRouteInfo *prevSegInfo = nullptr;
        bool visited                  = false;
        meter_t dist                  = micro::numeric_limits<meter_t>::infinity();
    };
    micro::vector<SegmentRouteInfo, 4 * cfg::MAX_NUM_LABYRINTH_SEGMENTS> segmentInfos;

    segmentInfos.push_back(SegmentRouteInfo{ &currentSeg, &prevConn, nullptr });
    auto segInfo = segmentInfos.begin();
    segInfo->dist = meter_t(0);

    while (true) {
        segInfo = std::min_element(segmentInfos.begin(), segmentInfos.end(), [](const auto& a, const auto& b) {
            // nodes that have already been visited cannot be selected as minimum value
            return a.visited == b.visited ? a.dist < b.dist : !a.visited;
        });

        if (segInfo->seg == &destSeg && segInfo->prevConn->junction == &lastJunction) {
            break;
        } else if (segInfo->visited) {
            // an error has occurred
            segInfo->prevSegInfo = nullptr;
            break;
        }

        for (const auto *newConn : segInfo->seg->edges) {
            const auto isFwd = isForwardConnection(*segInfo->prevConn, *segInfo->seg, *newConn);
            const auto* newSeg = newConn->getOtherSegment(*segInfo->seg);

            if ((!allowBackwardNavigation && !isFwd) ||
                restrictedJunctions.contains(newConn->junction->id)) {
                continue;
            }

            SegmentRouteInfo newSegInfo{newSeg, newConn, segInfo};

            // when going back to the previous junction, distance is not the same as when passing through the whole segment
            if (segInfo != segmentInfos.begin() && !isFwd) {
                newSegInfo.dist = segInfo->dist - segInfo->seg->length / 2 + meter_t(1.2f) + newSegInfo.seg->length / 2;
            } else {
                newSegInfo.dist = segInfo->dist + segInfo->seg->length / 2 + newSegInfo.seg->length / 2;
            }

            auto existingSegInfo = std::find_if(segmentInfos.begin(), segmentInfos.end(), [segInfo, &newSegInfo, allowBackwardNavigation](const auto& s) {
                return s.seg == newSegInfo.seg &&
                    // The existing segment info only matches the new one if either backward navigation is enabled
                    // or the last connection is at the same junction with the same orientation.
                    (allowBackwardNavigation || !isForwardConnection(*s.prevConn, *segInfo->seg, *newSegInfo.prevConn));
            });

            if (existingSegInfo != segmentInfos.end()) {
                if (newSegInfo.dist < existingSegInfo->dist) {
                    *existingSegInfo = newSegInfo;
                }
            } else {
                segmentInfos.push_back(newSegInfo);
            }
        }

        segInfo->visited = true;
    }

    LabyrinthRoute route(segInfo->seg);

    while (segInfo->prevSegInfo) {
        route.push_front(*segInfo->prevConn);
        segInfo = segInfo->prevSegInfo;
    }

    return route;
}
