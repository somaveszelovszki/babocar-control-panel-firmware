#include <micro/container/vector.hpp>

#include <LabyrinthRoute.hpp>

using namespace micro;

constexpr uint32_t LabyrinthRoute::MAX_LENGTH;

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

void LabyrinthRoute::reset(const Segment& currentSeg) {
    this->startSeg = this->destSeg = &currentSeg;
    this->connections.clear();
}

bool LabyrinthRoute::isForwardConnection(const Connection& prevConn, const Segment& currentSeg, const Connection& newConn) {
    // does not permit going backwards
    const bool isBwd = newConn.junction == prevConn.junction && newConn.getDecision(currentSeg) == prevConn.getDecision(currentSeg);
    return !isBwd;
}

LabyrinthRoute LabyrinthRoute::create(
    const Connection& prevConn,
    const Segment& currentSeg,
    const Segment& destSeg, 
    const micro::set<char, cfg::MAX_NUM_LABYRINTH_SEGMENTS>& forbiddenSegments,
    const bool allowBackwardNavigation) {
    // performs Dijkstra-algorithm (https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm)
    // specifically tuned for a car in a graph that allows multiple connections between the nodes

    struct SegmentRouteInfo {
        const Segment *seg            = nullptr;
        meter_t dist                  = micro::numeric_limits<meter_t>::infinity();
        const Connection *prevConn    = nullptr;
        bool visited                  = false;
        SegmentRouteInfo *prevSegInfo = nullptr;
    };
    micro::vector<SegmentRouteInfo, 2 * cfg::MAX_NUM_LABYRINTH_SEGMENTS> segmentInfos;

    segmentInfos.push_back(SegmentRouteInfo{ &currentSeg, meter_t(0), &prevConn, false, nullptr });
    auto segInfo = segmentInfos.begin();

    while (true) {
        segInfo = std::min_element(segmentInfos.begin(), segmentInfos.end(), [](const auto& a, const auto& b) {
            // nodes that have already been visited cannot be selected as minimum value
            return a.visited == b.visited ? a.dist < b.dist : !a.visited;
        });

        if (segInfo->seg == &destSeg) {
            break;
        } else if (segInfo->visited) {
            // an error has occurred
            segInfo->prevSegInfo = nullptr;
            break;
        }

        for (auto *newConn : segInfo->seg->edges) {
            const auto* newSeg = newConn->getOtherSegment(*segInfo->seg);
            if (forbiddenSegments.contains(newSeg->name) ||
                (!allowBackwardNavigation && !isForwardConnection(*segInfo->prevConn, *segInfo->seg, *newConn))) {
                continue;
            }

            SegmentRouteInfo newSegInfo;
            newSegInfo.seg         = newSeg;
            newSegInfo.prevConn    = newConn;
            newSegInfo.visited     = false;
            newSegInfo.prevSegInfo = segInfo;

            // when going back to the previous junction, distance is not the same as when passing through the whole segment
            if (segInfo != segmentInfos.begin() && allowBackwardNavigation && segInfo->prevConn->junction == newConn->junction) {
                newSegInfo.dist = segInfo->dist - segInfo->seg->length / 2 + meter_t(1.2f) + newSegInfo.seg->length / 2;
            } else {
                newSegInfo.dist = segInfo->dist + segInfo->seg->length / 2 + newSegInfo.seg->length / 2;
            }

            auto existingSegInfo = std::find_if(segmentInfos.begin(), segmentInfos.end(), [&newSegInfo, allowBackwardNavigation](const auto& element) {
                return element.seg == newSegInfo.seg &&
                        (allowBackwardNavigation ||
                            (element.prevConn->junction == newSegInfo.prevConn->junction &&
                            element.prevConn->getDecision(*newSegInfo.seg) == newSegInfo.prevConn->getDecision(*newSegInfo.seg)));
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

    LabyrinthRoute route(&destSeg);

    while (segInfo->prevSegInfo) {
        route.push_front(*segInfo->prevConn);
        segInfo = segInfo->prevSegInfo;
    }

    return route;
}
