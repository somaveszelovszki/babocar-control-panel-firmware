#include <micro/task/common.hpp>
#include <micro/utils/units.hpp>
#include <micro/utils/time.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/Line.hpp>

#include <LinePattern.hpp>
#include <LabyrinthGraph.hpp>
#include <DetectedLines.hpp>
#include <ControlData.hpp>
#include <cfg_board.h>
#include <cfg_car.hpp>
#include <cfg_track.hpp>
#include <globals.hpp>

#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_uart.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

using namespace micro;

extern QueueHandle_t detectedLinesQueue;
extern QueueHandle_t controlQueue;

namespace {
constexpr degree_t MERGE_VIRTUAL_LINE_ANGLE(30.0f);

uint8_t startCounterBuffer[1];
volatile char startCounter = '6';   // start counter will count back from 5 to 0

vec<Segment, 5 * cfg::MAX_NUM_LAB_SEGMENTS> segments;               // The segments.
vec<Junction, 5 * cfg::MAX_NUM_LAB_SEGMENTS> junctions;             // The junctions - at most the number of segments.
vec<Connection, 5 * cfg::MAX_NUM_LAB_SEGMENTS * 2> connections;     // The connections - 2 times the number of junctions.

Segment *currentSeg = nullptr;
Connection *lastConn = nullptr;

struct {
    Segment *seg = nullptr;
    Junction *lastJunc = nullptr;
    Maneuver lastManeuver;
} laneChange;

meter_t lineFollowDirStartDist;

millisecond_t endTime;

struct Route {
    static constexpr uint32_t MAX_LENGTH = cfg::MAX_NUM_LAB_SEGMENTS * 2;
    Segment *startSeg;
    Segment *lastSeg;
    vec<Connection*, MAX_LENGTH> connections;

    Route()
        : startSeg(nullptr)
        , lastSeg(nullptr) {}

    void append(Connection *c) {
        this->connections.append(c);
        this->lastSeg = c->getOtherSegment(this->lastSeg);
    }

    Connection* nextConnection() {
        Connection *conn = nullptr;
        if (this->connections.size()) {
            conn = this->connections[0];
            this->startSeg = conn->getOtherSegment(this->startSeg);
            this->connections.erase(this->connections.begin());
        }
        return conn;
    }

    Connection* lastConnection() const {
        return this->connections.size() > 0 ? this->connections[this->connections.size() - 1] : nullptr;
    }

    void reset() {
        this->startSeg = this->lastSeg = currentSeg;
        this->connections.clear();
    }
};

constexpr uint32_t MAX_NUM_ROUTES = 64;
vec<Route, MAX_NUM_ROUTES> routes;
Route plannedRoute;  // Planned route, when there is a given destination - e.g. given floating segment or the segment where the lane-change will happen

template <typename T, uint32_t N>
T* getNew(vec<T, N>& vec) {
    T *elem = nullptr;
    if (vec.append(T())) {
        elem = vec.back();
    } else {
        LOG_ERROR("Pool empty, cannot get new element");
    }
    return elem;
}

Junction* findExistingJunction(const point2m& pos) {
    constexpr meter_t MIN_JUNCTION_POS_ACCURACY = centimeter_t(50);

    Junction *result = nullptr;
    meter_t minDist = meter_t::infinity();

    for(Junction& j : junctions) {
        const meter_t dist = pos.distance(j.pos);

        if (dist < minDist) {
            result = &j;
            minDist = dist;
        }
    }

    if (minDist > MIN_JUNCTION_POS_ACCURACY) {  // result is further from the current position than the max, this junction is a new one
        result = nullptr;
    }

    LOG_DEBUG("pos: %d, %d", (int32_t)static_cast<centimeter_t>(pos.X).get(), (int32_t)static_cast<centimeter_t>(pos.Y).get());
    if (result) {
        LOG_DEBUG("found #%d:  %d, %d", result->idx, (int32_t)static_cast<centimeter_t>(result->pos.X).get(), (int32_t)static_cast<centimeter_t>(result->pos.Y).get());
    }

    return result;
}

bool getRoute(Route& result, const Segment *dest, const Junction *lastRouteJunc = nullptr, const Maneuver lastRouteManeuver = Maneuver()) {
    routes.clear();
    Route * const initialRoute = getNew(routes);
    initialRoute->startSeg = initialRoute->lastSeg = currentSeg;
    uint8_t depth = 0;
    Route *shortestRoute = nullptr;

    do {
        while (routes.size() > 0 && !shortestRoute) {
            Route *currentRoute = &routes[0];
            const Connection *lastRouteConn = currentRoute->lastConnection();
            if (!lastRouteConn) {
                lastRouteConn = lastConn;
            }

            const Maneuver lastManeuver = lastRouteConn->getManeuver(currentRoute->lastSeg);

            for (Connection *c : currentRoute->lastSeg->edges) {
                if (c->junction != lastRouteConn->junction || c->getManeuver(currentRoute->lastSeg) != lastManeuver) { // does not permit going backwards
                    if (routes.size() >= MAX_NUM_ROUTES) {
                        LOG_ERROR("routes.size() >= MAX_NUM_ROUTES");
                        break;
                    }
                    Route *newRoute = getNew(routes);
                    *newRoute = *currentRoute;
                    newRoute->append(c);

                    // if we reached the destination, the shortest route has been found
                    if (newRoute->lastSeg == dest && (!lastRouteJunc ||
                        (lastRouteJunc == newRoute->lastConnection()->junction &&
                        lastRouteManeuver == newRoute->lastConnection()->getManeuver(newRoute->lastSeg)))) {
                        shortestRoute = newRoute;
                        break;
                    }
                }
            }

            routes.erase(routes.begin());
        }
    } while(!shortestRoute && routes.size() > 0 && ++depth < Route::MAX_LENGTH);

    if (shortestRoute) {
        result = *shortestRoute;
    }

    return !!shortestRoute;
}

vec<Segment*, cfg::MAX_NUM_LAB_SEGMENTS> getFloatingSegments() {
    vec<Segment*, cfg::MAX_NUM_LAB_SEGMENTS> floatingSegments;
    for (Segment& seg : segments) {
        if (seg.isActive && seg.isFloating()) {
            floatingSegments.append(&seg);
        }
    }
    return floatingSegments;
}

bool getRoute() {
    bool success = false;
    plannedRoute.reset();

    vec<Segment*, cfg::MAX_NUM_LAB_SEGMENTS> floatingSegments = getFloatingSegments();

    if (floatingSegments.size()) {    // if there are still floating segments in the graph, chooses the closest one as destination
        Route route;
        for (Segment *seg : floatingSegments) {
            if ((success |= getRoute(route, seg)) && (!plannedRoute.connections.size() || (plannedRoute.connections.size() > route.connections.size()))) {
                plannedRoute = route;
            }
        }
    } else if (laneChange.seg && laneChange.lastJunc) { // no floating segments in the graph, new destination will be the lane change segment
        success = getRoute(plannedRoute, laneChange.seg, laneChange.lastJunc, laneChange.lastManeuver);
    }

    return success;
}

Segment* createNewSegment() {
    Segment *seg = getNew(segments);
    if (seg) {
        seg->name      = 'A' + static_cast<char>(segments.size() - 1);
        seg->length    = meter_t(0);
        seg->isDeadEnd = false;
        seg->isActive  = true;
    }
    return seg;
}

void addSegments(Junction * const junc, radian_t fwdOri, radian_t bwdOri, uint8_t numInSegments, Direction inSegmentDir, uint8_t numOutSegments) {

    junc->addSegment(currentSeg, bwdOri, inSegmentDir);

    switch (numInSegments) {
    case 2:
        junc->addSegment(createNewSegment(), bwdOri, inSegmentDir == Direction::LEFT ? Direction::RIGHT : Direction::LEFT);
        break;
    case 3:
        switch (inSegmentDir) {
        case Direction::LEFT:
            junc->addSegment(createNewSegment(), bwdOri, Direction::RIGHT);
            junc->addSegment(createNewSegment(), bwdOri, Direction::CENTER);
            break;
        case Direction::CENTER:
            junc->addSegment(createNewSegment(), bwdOri, Direction::RIGHT);
            junc->addSegment(createNewSegment(), bwdOri, Direction::LEFT);
            break;
        case Direction::RIGHT:
            junc->addSegment(createNewSegment(), bwdOri, Direction::CENTER);
            junc->addSegment(createNewSegment(), bwdOri, Direction::LEFT);
            break;
        }
        break;
    }

    switch (numOutSegments) {
    case 1:
        junc->addSegment(createNewSegment(), fwdOri, Direction::CENTER);
        break;
    case 2:
        junc->addSegment(createNewSegment(), fwdOri, Direction::RIGHT);
        junc->addSegment(createNewSegment(), fwdOri, Direction::LEFT);
        break;
    case 3:
        junc->addSegment(createNewSegment(), fwdOri, Direction::RIGHT);
        junc->addSegment(createNewSegment(), fwdOri, Direction::CENTER);
        junc->addSegment(createNewSegment(), fwdOri, Direction::LEFT);
        break;
    }
}

void addConnection(Connection * const conn, Segment * const seg1, const Maneuver& maneuver1, Segment * const seg2, const Maneuver& maneuver2) {
    conn->node1 = seg1;
    conn->node2 = seg2;
    conn->maneuver1 = maneuver1;
    conn->maneuver2 = maneuver2;
    seg1->edges.append(conn);
    seg2->edges.append(conn);
}

vec<Connection*, 2> getConnections(Segment *seg1, Segment *seg2) {
    vec<Connection*, 2> connections;
    for (Connection *c : seg1->edges) {
        if (c->getOtherSegment(seg1) == seg2) {
            connections.append(c);
        }
    }
    return connections;
}

Junction* onNewJunction(const point2m& pos, radian_t fwdOri, radian_t bwdOri, uint8_t numInSegments, Direction inSegmentDir, uint8_t numOutSegments) {
    LOG_DEBUG("New junction | currentSeg: %c", currentSeg->name);
    Junction * const junc = getNew(junctions);
    if (junc) {
        junc->idx = junctions.size();
        junc->pos = pos;
        addSegments(junc, fwdOri, bwdOri, numInSegments, inSegmentDir, numOutSegments);

        Junction::segment_map::iterator inSegments  = junc->getSideSegments(bwdOri);
        Junction::segment_map::iterator outSegments = junc->getSideSegments(fwdOri);

        for (Junction::side_segment_map::iterator in = inSegments->second.begin(); in != inSegments->second.end(); ++in) {
            for (Junction::side_segment_map::iterator out = outSegments->second.begin(); out != outSegments->second.end(); ++out) {
                Connection *conn = getNew(connections);
                conn->junction = junc;
                addConnection(conn, in->second, { inSegments->first, in->first }, out->second, { outSegments->first, out->first });
            }
        }

        endTime += second_t(10);

    } else {
        LOG_ERROR("Junction pool empty");
    }

    return junc;
}

Status mergeSegments(Segment *oldSeg, Segment *newSeg) {
    Status result = Status::INVALID_DATA;

    if (oldSeg != newSeg) {
        if (newSeg->isFloating()) {
            LOG_DEBUG("New segment floating (OK)");

            // if both segments are floating, merges them
            if (oldSeg->isFloating()) {
                // adds all the connections of this segment to the previously defined floating segment (updates segment pointers)
                for (Connection *c : oldSeg->edges) {
                    c->updateSegment(oldSeg, newSeg);
                    newSeg->edges.append(c);
                    c->junction->updateSegment(oldSeg, newSeg);
                }

                if (laneChange.seg == oldSeg) {
                    laneChange.seg = newSeg;
                }

                oldSeg->isActive = false;
                result = Status::OK;
                LOG_DEBUG("Merged: %c -> %c", oldSeg->name, newSeg->name);
            } else {
                LOG_ERROR("Old segment not floating (NOT OK)");
            }
        } else {
            LOG_ERROR("New segment not floating (NOT OK)");
        }
    }

    return result;
}

Connection* onExistingJunction(Junction *junc, const point2m& pos, radian_t fwdOri, radian_t bwdOri, uint8_t numInSegments, Direction inSegmentDir, uint8_t numOutSegments) {

    Segment * const junctionSeg = junc->getSegment(bwdOri, inSegmentDir);
    Connection *nextConn = nullptr;

    LOG_DEBUG("Junction: %d | currentSeg: %c", junc->idx, currentSeg->name);

    vTaskSuspendAll();
    globals::car.pose.pos += (junc->pos - pos);
    xTaskResumeAll();

    LOG_DEBUG("Car pos updated: (%f, %f)", static_cast<centimeter_t>(globals::car.pose.pos.X).get(), static_cast<centimeter_t>(globals::car.pose.pos.Y).get());

    if (junctionSeg) {
        LOG_DEBUG("junctionSeg: %c", junctionSeg->name);

        if (currentSeg != junctionSeg && !isOk(mergeSegments(currentSeg, junctionSeg))) {
            plannedRoute.reset();
        }

        currentSeg = junctionSeg;

        // If there is no planned route, creates a new route.
        // The destination of the route will be the closest floating segment.
        // If there are no floating segments, it means the labyrinth has been completed.
        // In this case the destination will be the lane change segment.
        if (plannedRoute.connections.size() || getRoute()) {
            nextConn = plannedRoute.nextConnection();
        } else {
            LOG_ERROR("createNewRoute() failed");
        }
    } else {
        LOG_ERROR("Junction segment with orientation [%fdeg] and direction [%s] not found", static_cast<degree_t>(bwdOri).get(), to_string(inSegmentDir));
    }

    return nextConn;
}

Direction onJunctionDetected(const point2m& pos, radian_t angle, uint8_t numInSegments, Direction inSegmentDir, uint8_t numOutSegments) {

    Junction *junc = findExistingJunction(pos);
    Connection *nextConn = nullptr;

    const radian_t fwdOri = micro::normalize360(angle);
    const radian_t bwdOri = micro::normalize360(angle + PI);

    Maneuver nextManeuver = { fwdOri, Direction::CENTER };

    if (!junc) { // new junction found - creates new segments and adds connections
        junc = onNewJunction(pos, fwdOri, bwdOri, numInSegments, inSegmentDir, numOutSegments);
    } else { // arrived at a previously found junction - the corresponding floating segment of the junction (if there is one) must be merged with the current segment
        nextConn = onExistingJunction(junc, pos, fwdOri, bwdOri, numInSegments, inSegmentDir, numOutSegments);
    }

    if (!nextConn) {
        Junction::segment_map::iterator outSegments = junc->getSideSegments(nextManeuver.orientation);
        Segment **pNextSeg = outSegments->second.get((nextManeuver.direction = Direction::RIGHT));
        if (!pNextSeg) {
            pNextSeg = outSegments->second.get((nextManeuver.direction = Direction::CENTER));
        }

        for (Connection *c : getConnections(currentSeg, *pNextSeg)) {
            if (c->junction == junc && c->getManeuver(*pNextSeg) == nextManeuver) {
                nextConn = c;
                break;
            }
        }
    }

    if (nextConn) {

        // If LANE_CHANGE pattern has been found, but last junction before lane change has not been filled,
        // it means that the LANE_CHANGE pattern has been detected with NEGATIVE sign,
        // therefore the current junction (the one detected after the LANE_CHANGE pattern) will be
        // the junction before the lane change when planning the trajectory.
        // This way the LANE_CHANGE pattern will be detected with POSITIVE sign during lane change.
        if (laneChange.seg && !laneChange.lastJunc) {
            laneChange.lastJunc = junc;
            laneChange.lastManeuver = nextConn->getManeuver(currentSeg);
        }

        Segment *nextSeg = nextConn->getOtherSegment(currentSeg);
        currentSeg = nextSeg;
        lastConn = nextConn;
        LOG_DEBUG("Next: %c", nextSeg->name);
    } else {
        LOG_ERROR("No next connection found");
    }

    return nextManeuver.direction;
}

enum {
    ProgSubCntr_WaitStartSignal   = 0,
    ProgSubCntr_NavigateLabyrinth = 1,
    ProgSubCntr_LaneChange        = 2
};

struct TestCase {
    Pose pose;
    LinePattern pattern;
};

const TestCase testCase1[] = {
    { { { meter_t(0), meter_t(0) }, PI_2 }, { LinePattern::Type::SINGLE_LINE, Sign::NEUTRAL,  Direction::CENTER } },

    // 1
    { { { meter_t(0), meter_t(2) }, PI_2 }, { LinePattern::Type::JUNCTION_1,  Sign::NEGATIVE, Direction::CENTER } },
    { { { meter_t(0), meter_t(2) }, PI_2 }, { LinePattern::Type::JUNCTION_2,  Sign::POSITIVE, Direction::RIGHT  } },
    { { { meter_t(0), meter_t(2) }, PI_2 }, { LinePattern::Type::SINGLE_LINE, Sign::NEUTRAL,  Direction::CENTER } },

    // 2
    { { { meter_t(3), meter_t(4) }, PI_2 }, { LinePattern::Type::JUNCTION_3,  Sign::NEGATIVE, Direction::RIGHT  } },
    { { { meter_t(3), meter_t(4) }, PI_2 }, { LinePattern::Type::JUNCTION_2,  Sign::POSITIVE, Direction::RIGHT  } },
    { { { meter_t(3), meter_t(4) }, PI_2 }, { LinePattern::Type::SINGLE_LINE, Sign::NEUTRAL,  Direction::CENTER } },

    // 3
    { { { meter_t(0), meter_t(5) }, -PI_2 }, { LinePattern::Type::JUNCTION_2,  Sign::NEGATIVE, Direction::LEFT   } },
    { { { meter_t(0), meter_t(5) }, -PI_2 }, { LinePattern::Type::JUNCTION_1,  Sign::POSITIVE, Direction::CENTER } },
    { { { meter_t(0), meter_t(5) }, -PI_2 }, { LinePattern::Type::SINGLE_LINE, Sign::NEUTRAL,  Direction::CENTER } },

    // 1
    { { { meter_t(0), meter_t(2) }, -PI_2 }, { LinePattern::Type::JUNCTION_2,  Sign::NEGATIVE, Direction::LEFT   } },
    { { { meter_t(0), meter_t(2) }, -PI_2 }, { LinePattern::Type::JUNCTION_1,  Sign::POSITIVE, Direction::CENTER } },
    { { { meter_t(0), meter_t(2) }, -PI_2 }, { LinePattern::Type::SINGLE_LINE, Sign::NEUTRAL,  Direction::CENTER } },

    // 4
    { { { meter_t(0), meter_t(-2) }, -PI_2 }, { LinePattern::Type::JUNCTION_1,  Sign::NEGATIVE, Direction::CENTER } },
    { { { meter_t(0), meter_t(-2) }, -PI_2 }, { LinePattern::Type::JUNCTION_2,  Sign::POSITIVE, Direction::RIGHT  } },
    { { { meter_t(0), meter_t(-2) }, -PI_2 }, { LinePattern::Type::SINGLE_LINE, Sign::NEUTRAL,  Direction::CENTER } },

    // LANE_CHANGE
    { { { meter_t(-2), meter_t(-4) }, radian_t(0) }, { LinePattern::Type::LANE_CHANGE, Sign::POSITIVE, Direction::RIGHT  } },
    { { { meter_t(-1), meter_t(-4) }, radian_t(0) }, { LinePattern::Type::SINGLE_LINE, Sign::NEUTRAL,  Direction::CENTER } },

    // 4
    { { { meter_t(0), meter_t(-2) }, PI_2 }, { LinePattern::Type::JUNCTION_2,  Sign::NEGATIVE, Direction::LEFT   } },
    { { { meter_t(0), meter_t(-2) }, PI_2 }, { LinePattern::Type::JUNCTION_1,  Sign::POSITIVE, Direction::CENTER } },
    { { { meter_t(0), meter_t(-2) }, PI_2 }, { LinePattern::Type::SINGLE_LINE, Sign::NEUTRAL,  Direction::CENTER } },

    // 1
    { { { meter_t(0), meter_t(2) }, PI_2 }, { LinePattern::Type::JUNCTION_1,  Sign::NEGATIVE, Direction::CENTER } },
    { { { meter_t(0), meter_t(2) }, PI_2 }, { LinePattern::Type::JUNCTION_2,  Sign::POSITIVE, Direction::RIGHT  } },
    { { { meter_t(0), meter_t(2) }, PI_2 }, { LinePattern::Type::SINGLE_LINE, Sign::NEUTRAL,  Direction::CENTER } },

    // 2
    { { { meter_t(3), meter_t(4) }, PI_2 }, { LinePattern::Type::JUNCTION_3,  Sign::NEGATIVE, Direction::RIGHT  } },
    { { { meter_t(3), meter_t(4) }, PI_2 }, { LinePattern::Type::JUNCTION_2,  Sign::POSITIVE, Direction::RIGHT  } },
    { { { meter_t(3), meter_t(4) }, PI_2 }, { LinePattern::Type::SINGLE_LINE, Sign::NEUTRAL,  Direction::CENTER } },

    // 3
    { { { meter_t(0), meter_t(5) }, -PI_2 }, { LinePattern::Type::JUNCTION_2,  Sign::NEGATIVE, Direction::RIGHT  } },
    { { { meter_t(0), meter_t(5) }, -PI_2 }, { LinePattern::Type::JUNCTION_1,  Sign::POSITIVE, Direction::CENTER } },
    { { { meter_t(0), meter_t(5) }, -PI_2 }, { LinePattern::Type::SINGLE_LINE, Sign::NEUTRAL,  Direction::CENTER } },

    // 1
    { { { meter_t(0), meter_t(2) }, -PI_2 }, { LinePattern::Type::JUNCTION_2,  Sign::NEGATIVE, Direction::LEFT   } },
    { { { meter_t(0), meter_t(2) }, -PI_2 }, { LinePattern::Type::JUNCTION_1,  Sign::POSITIVE, Direction::CENTER } },
    { { { meter_t(0), meter_t(2) }, -PI_2 }, { LinePattern::Type::SINGLE_LINE, Sign::NEUTRAL,  Direction::CENTER } },

    // 4
    { { { meter_t(0), meter_t(-2) }, -PI_2 }, { LinePattern::Type::JUNCTION_1,  Sign::NEGATIVE, Direction::CENTER } },
    { { { meter_t(0), meter_t(-2) }, -PI_2 }, { LinePattern::Type::JUNCTION_2,  Sign::POSITIVE, Direction::RIGHT  } },
    { { { meter_t(0), meter_t(-2) }, -PI_2 }, { LinePattern::Type::SINGLE_LINE, Sign::NEUTRAL,  Direction::CENTER } },

    // LANE_CHANGE
    { { { meter_t(-2), meter_t(-4) }, radian_t(0) }, { LinePattern::Type::LANE_CHANGE, Sign::POSITIVE, Direction::RIGHT  } },
    { { { meter_t(-1), meter_t(-4) }, radian_t(0) }, { LinePattern::Type::SINGLE_LINE, Sign::NEUTRAL,  Direction::CENTER } },

    // 4
    { { { meter_t(0), meter_t(-2) }, PI_2 }, { LinePattern::Type::JUNCTION_2,  Sign::NEGATIVE, Direction::LEFT   } },
    { { { meter_t(0), meter_t(-2) }, PI_2 }, { LinePattern::Type::JUNCTION_1,  Sign::POSITIVE, Direction::CENTER } },
    { { { meter_t(0), meter_t(-2) }, PI_2 }, { LinePattern::Type::SINGLE_LINE, Sign::NEUTRAL,  Direction::CENTER } },

    // 1
    { { { meter_t(0), meter_t(2) }, PI_2 }, { LinePattern::Type::JUNCTION_1,  Sign::NEGATIVE, Direction::CENTER } },
    { { { meter_t(0), meter_t(2) }, PI_2 }, { LinePattern::Type::JUNCTION_2,  Sign::POSITIVE, Direction::RIGHT  } },
    { { { meter_t(0), meter_t(2) }, PI_2 }, { LinePattern::Type::SINGLE_LINE, Sign::NEUTRAL,  Direction::CENTER } },

    // 3
    { { { meter_t(0), meter_t(5) }, PI_2 }, { LinePattern::Type::JUNCTION_1,  Sign::NEGATIVE, Direction::CENTER } },
    { { { meter_t(0), meter_t(5) }, PI_2 }, { LinePattern::Type::JUNCTION_2,  Sign::POSITIVE, Direction::LEFT   } },
    { { { meter_t(0), meter_t(5) }, PI_2 }, { LinePattern::Type::SINGLE_LINE, Sign::NEUTRAL,  Direction::CENTER } },

    // 2
    { { { meter_t(3), meter_t(4) }, -PI_2 }, { LinePattern::Type::JUNCTION_2,  Sign::NEGATIVE, Direction::RIGHT  } },
    { { { meter_t(3), meter_t(4) }, -PI_2 }, { LinePattern::Type::JUNCTION_3,  Sign::POSITIVE, Direction::CENTER } },
    { { { meter_t(3), meter_t(4) }, -PI_2 }, { LinePattern::Type::SINGLE_LINE, Sign::NEUTRAL,  Direction::CENTER } },

    // 2
    { { { meter_t(3), meter_t(4) }, PI_2 }, { LinePattern::Type::JUNCTION_3,  Sign::NEGATIVE, Direction::LEFT   } },
    { { { meter_t(3), meter_t(4) }, PI_2 }, { LinePattern::Type::JUNCTION_2,  Sign::POSITIVE, Direction::LEFT   } },
    { { { meter_t(3), meter_t(4) }, PI_2 }, { LinePattern::Type::SINGLE_LINE, Sign::NEUTRAL,  Direction::CENTER } },

    // 3
    { { { meter_t(0), meter_t(5) }, -PI_2 }, { LinePattern::Type::JUNCTION_2,  Sign::NEGATIVE, Direction::LEFT   } },
    { { { meter_t(0), meter_t(5) }, -PI_2 }, { LinePattern::Type::JUNCTION_1,  Sign::POSITIVE, Direction::CENTER } },
    { { { meter_t(0), meter_t(5) }, -PI_2 }, { LinePattern::Type::SINGLE_LINE, Sign::NEUTRAL,  Direction::CENTER } },

    // 1
    { { { meter_t(0), meter_t(2) }, -PI_2 }, { LinePattern::Type::JUNCTION_2,  Sign::NEGATIVE, Direction::LEFT   } },
    { { { meter_t(0), meter_t(2) }, -PI_2 }, { LinePattern::Type::JUNCTION_1,  Sign::POSITIVE, Direction::CENTER } },
    { { { meter_t(0), meter_t(2) }, -PI_2 }, { LinePattern::Type::SINGLE_LINE, Sign::NEUTRAL,  Direction::CENTER } },

    // 4
    { { { meter_t(0), meter_t(-2) }, -PI_2 }, { LinePattern::Type::JUNCTION_1,  Sign::NEGATIVE, Direction::CENTER } },
    { { { meter_t(0), meter_t(-2) }, -PI_2 }, { LinePattern::Type::JUNCTION_2,  Sign::POSITIVE, Direction::RIGHT  } },
    { { { meter_t(0), meter_t(-2) }, -PI_2 }, { LinePattern::Type::SINGLE_LINE, Sign::NEUTRAL,  Direction::CENTER } },

    // LANE_CHANGE
    { { { meter_t(-2), meter_t(-4) }, radian_t(0) }, { LinePattern::Type::LANE_CHANGE, Sign::POSITIVE, Direction::RIGHT  } },
    { { { meter_t(-1), meter_t(-4) }, radian_t(0) }, { LinePattern::Type::NONE,        Sign::NEUTRAL,  Direction::CENTER } }
};

bool waitStartSignal() {
    static bool startSignalRecvStarted = false;
    static char prevStartCounter = startCounter;

    bool finished = false;

    if (!startSignalRecvStarted) {
        HAL_UART_Receive_DMA(uart_RadioModule, startCounterBuffer, 1);
        startSignalRecvStarted = true;
    }

    if (!globals::startSignalEnabled || '0' == startCounter) {
        HAL_UART_DMAStop(uart_RadioModule);
        endTime = getTime() + second_t(20);
        finished = true;
        LOG_DEBUG("Started!");
    } else if (startCounter != prevStartCounter) {
        LOG_DEBUG("Seconds until start: %c", startCounter);
        prevStartCounter = startCounter;
    }

    return finished;
}

bool navigateLabyrinth(const DetectedLines& prevDetectedLines, const DetectedLines& detectedLines, Line& mainLine) {

    // TODO only for testing -------------------------------------------

    static const TestCase *testCase = testCase1;
    static const uint32_t numTestCasePoints = ARRAY_SIZE(testCase1);
    static uint32_t testCasePointIdx = 0;

    if (testCasePointIdx < numTestCasePoints) {
        globals::car.pose = testCase[testCasePointIdx].pose;
        const_cast<DetectedLines&>(detectedLines).pattern = testCase[testCasePointIdx].pattern;
        testCasePointIdx++;
    }

    // -----------------------------------------------------------------

    static uint8_t numInSegments;
    static Direction inSegmentDir;
    static point2m inJunctionPos;

    bool finished = false;

    if (detectedLines.pattern != prevDetectedLines.pattern) {

        switch (detectedLines.pattern.type) {
        case LinePattern::JUNCTION_1:
        case LinePattern::JUNCTION_2:
        case LinePattern::JUNCTION_3:
        {
            const uint8_t numSegments = LinePattern::JUNCTION_1 == detectedLines.pattern.type ? 1 :
                LinePattern::JUNCTION_2 == detectedLines.pattern.type ? 2 : 3;

            if (Sign::NEGATIVE == detectedLines.pattern.dir) {
                numInSegments = numSegments;
                inJunctionPos = globals::car.pose.pos;

                // Line pattern direction indicates on which side of the current line the OTHER lines are,
                // so if the current line is the leftmost line of three lines, pattern direction will be RIGHT.
                // Segment direction in a junction indicates which side the car should steer in order to follow a segment,
                // when leaving a junction.
                // Currently the car is entering a junction, therefore if the current line is the leftmost,
                // the car will need to steer to the right when leaving the junction.
                // So the segment direction is the same as the line pattern direction (in this example, RIGHT).
                inSegmentDir = detectedLines.pattern.side;

            } else if (Sign::POSITIVE == detectedLines.pattern.dir) {
                const point2m junctionPos = avg(inJunctionPos, globals::car.pose.pos);
                const Direction steeringDir = onJunctionDetected(junctionPos, globals::car.pose.angle, numInSegments, inSegmentDir, numSegments);

                // updates the main line (if an error occurs, does not change the default line)
                switch (steeringDir) {
                case Direction::LEFT:
                    if (detectedLines.lines.size() >= 2) {
                        mainLine = detectedLines.lines[0];
                    }
                    break;
                case Direction::CENTER:
                    if (detectedLines.lines.size() == 1) {
                        mainLine = detectedLines.lines[0];
                    } else if (detectedLines.lines.size() == 3) {
                        mainLine = detectedLines.lines[1];
                    }
                    break;
                case Direction::RIGHT:
                    if (detectedLines.lines.size() >= 2) {
                        mainLine = *detectedLines.lines.back();
                    }
                    break;
                }
            }
            break;
        }
        case LinePattern::LANE_CHANGE:
            if (getFloatingSegments().size()) {
                laneChange.seg = currentSeg;
                if (Sign::POSITIVE == detectedLines.pattern.dir) {
                    laneChange.lastJunc = lastConn->junction;
                    laneChange.lastManeuver = lastConn->getManeuver(currentSeg);
                }
            } else {
                finished = true;
            }
            break;
        default:
            break;
        }
    }

    return finished;
}

} // namespace

extern "C" void runProgLabyrinthTask(void const *argument) {

    vTaskDelay(100); // gives time to other tasks and panels to wake up

    DetectedLines prevDetectedLines, detectedLines;
    Line mainLine;
    ControlData controlData;

    currentSeg = createNewSegment();
    endTime = getTime() + second_t(20);

    while (true) {
        switch (globals::programState.activeModule()) {
            case ProgramState::ActiveModule::Labyrinth:

                xQueuePeek(detectedLinesQueue, &detectedLines, 0);
                LineCalculator::updateMainLine(detectedLines.lines, mainLine);

                switch (globals::programState.subCntr()) {
                case ProgSubCntr_WaitStartSignal:
                    if (waitStartSignal()) {
                        globals::programState.set(ProgramState::ActiveModule::Labyrinth, ProgSubCntr_NavigateLabyrinth);
                    }
                    controlData.speed = m_per_sec_t(0);
                    break;

                case ProgSubCntr_NavigateLabyrinth:
                    if (navigateLabyrinth(prevDetectedLines, detectedLines, mainLine)) {
                        globals::programState.set(ProgramState::ActiveModule::Labyrinth, ProgSubCntr_LaneChange);
                    }
                    controlData.speed = m_per_sec_t(1.0f);
                    break;

                case ProgSubCntr_LaneChange:
                    break;
                }

                controlData.baseline = mainLine;
                controlData.angle = degree_t(0);
                controlData.offset = millimeter_t(0);

                xQueueOverwrite(controlQueue, &controlData);
                prevDetectedLines = detectedLines;
                break;

            default:
                vTaskDelay(100);
                break;
        }

        vTaskDelay(1);
    }

    vTaskDelete(nullptr);
}
/* @brief Callback for RadioModule UART RxCplt - called when receive finishes.
 */
void micro_RadioModule_Uart_RxCpltCallback() {
    const uint8_t cntr = static_cast<uint8_t>(startCounterBuffer[0]);
    if (cntr == startCounter - 1) {
        startCounter = cntr;
    }
}
