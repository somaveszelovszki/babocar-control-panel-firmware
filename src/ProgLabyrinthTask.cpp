#include <micro/task/common.hpp>
#include <micro/utils/units.hpp>
#include <micro/utils/time.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/Line.hpp>
#include <micro/utils/trajectory.hpp>

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

#define RANDOM_SEGMENT false

extern QueueHandle_t detectedLinesQueue;
extern QueueHandle_t controlQueue;

namespace {

vec<Segment, 2 * cfg::MAX_NUM_LAB_SEGMENTS> segments;               // The segments.
vec<Junction, 2 * cfg::MAX_NUM_LAB_SEGMENTS> junctions;             // The junctions - at most the number of segments.
vec<Connection, 2 * cfg::MAX_NUM_LAB_SEGMENTS * 2> connections;     // The connections - 2 times the number of junctions.

Segment *currentSeg = nullptr;
Connection *prevConn = nullptr;
Connection *prevPrevConn = nullptr;

millisecond_t endTime;

struct {
    Segment *seg          = nullptr;
    Junction *lastJunc    = nullptr;
    Maneuver lastManeuver = { radian_t(0), Direction::CENTER };
    Trajectory trajectory = Trajectory(cfg::CAR_OPTO_CENTER_DIST);
} laneChange;

enum class Y_turnState {
    INACTIVE,
    BWD_STRAIGHT,
    BWD_RIGHT1,
    PREPARE_FWD_LEFT1,
    FWD_LEFT1,
    PREPARE_BWD_RIGHT2,
    BWD_RIGHT2,
    PREPARE_FWD_LEFT2,
    FWD_LEFT2
};

struct {
    Y_turnState state = Y_turnState::INACTIVE;
    meter_t startDist;
    radian_t startOrientation;
    millisecond_t stateStartTime;
} y_turn;

struct Route {
    static constexpr uint32_t MAX_LENGTH = cfg::MAX_NUM_LAB_SEGMENTS;
    Segment *startSeg;
    Segment *lastSeg;
    vec<Connection*, MAX_LENGTH> connections;

    Route()
        : startSeg(nullptr)
        , lastSeg(nullptr) {}

    void append(Connection *c) {
        this->connections.push_back(c);
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

constexpr uint32_t MAX_NUM_ROUTES = 256;
vec<Route, MAX_NUM_ROUTES> routes;
Route plannedRoute;  // Planned route, when there is a given destination - e.g. given floating segment or the segment where the lane-change will happen

template <typename T, uint32_t N>
T* getNew(vec<T, N>& vec) {
    T *elem = nullptr;
    if (vec.push_back(T())) {
        elem = vec.back();
    } else {
        LOG_ERROR("Pool empty, cannot get new element");
    }
    return elem;
}

Junction* findExistingJunction(const point2m& pos, radian_t inOri, radian_t outOri, uint8_t numInSegments, Direction inSegmentDir, uint8_t numOutSegments, bool& isValid) {

    Junction *result = nullptr;

    struct JunctionDist {
        Junction *junc = nullptr;
        meter_t dist = meter_t::infinity();
    };

    LOG_DEBUG("pos: (%f, %f)", pos.X.get(), pos.Y.get());

    // FIRST:  closest junction to current position
    // SECOND: closest junction to current position with the correct topology
    std::pair<JunctionDist, JunctionDist> closest = {};

    for(Junction& j : junctions) {
        const meter_t dist = pos.distance(j.pos);

        if (dist < closest.first.dist) {
            closest.first.junc = &j;
            closest.first.dist = dist;
        }

        if (dist < closest.second.dist) {
            const Junction::segment_map::const_iterator inSegments = j.getSideSegments(inOri);
            if (inSegments != j.segments.end() && inSegments->second.size() == numInSegments) {

                const Junction::segment_map::const_iterator outSegments = j.getSideSegments(outOri);
                if (outSegments != j.segments.end() && outSegments->second.size() == numOutSegments) {
                    closest.second.junc = &j;
                    closest.second.dist = dist;
                }
            }
        }
    }

    if (closest.first.junc) {
        LOG_DEBUG("closest: (%f, %f)", closest.first.junc->pos.X.get(), closest.first.junc->pos.Y.get());
    }

    if (closest.second.junc) {
        LOG_DEBUG("closest with ori: (%f, %f)", closest.second.junc->pos.X.get(), closest.second.junc->pos.Y.get());
    }

    if (closest.second.dist < centimeter_t(120)) {
        // a junction at the right position and the correct topology has been found
        result = closest.second.junc;
        isValid = true;
    } else if (closest.first.dist < centimeter_t(80)) {
        // a junction at the right position but with incorrect topology has been found
        result = closest.first.junc;
        isValid = false;
    } else {
        // the junction has not been found
        result = nullptr;
        isValid = false;
    }

    return result;
}

bool isValidRouteConnection(const Route *route, const Connection *lastRouteConn, const Maneuver lastManeuver, const Connection *c) {
    bool valid = false;

    // does not permit going backwards
    if (!lastRouteConn ||
        c->junction != lastRouteConn->junction ||
        c->getManeuver(route->lastSeg) != lastManeuver) {

        // does not permit navigating through dead-end segments
        if (!c->getOtherSegment(route->lastSeg)->isDeadEnd) {

            const Segment *newSeg = c->getOtherSegment(route->lastSeg);
            const Maneuver newManeuver = c->getManeuver(newSeg);

            valid = true;

            // does not permit cycles (taking the same junction exit twice)
            const Segment *prevSeg = route->startSeg;
            for (const Connection *routeConn : route->connections) {
                const Segment *nextSeg = routeConn->getOtherSegment(prevSeg);
                if (nextSeg == newSeg && routeConn->getManeuver(nextSeg) == newManeuver) {
                    valid = false;
                    break;
                }
                prevSeg = nextSeg;
            }

        }
    }
    return valid;
}

bool getRoute(Route& result, const Segment *dest, const Junction *lastRouteJunc = nullptr, const Maneuver lastRouteManeuver = Maneuver()) {
    routes.clear();
    Route * const initialRoute = getNew(routes);
    initialRoute->startSeg = initialRoute->lastSeg = currentSeg;
    Route *shortestRoute = nullptr;

    do {
        Route *currentRoute = &routes[0];
        const Connection *lastRouteConn = currentRoute->lastConnection();
        if (!lastRouteConn) {
            lastRouteConn = prevConn;
        }

        const Maneuver lastManeuver = lastRouteConn ? lastRouteConn->getManeuver(currentRoute->lastSeg) : Maneuver();

        for (Connection *c : currentRoute->lastSeg->edges) {
            if (isValidRouteConnection(currentRoute, lastRouteConn, lastManeuver, c)) {
                if (routes.size() == MAX_NUM_ROUTES) {
                    LOG_ERROR("routes.size() == MAX_NUM_ROUTES");
                    routes.clear();
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

        if (routes.size()) {
            routes.erase(routes.begin());
        }
    } while(!shortestRoute && routes.size() > 0 && routes.begin()->connections.size() < Route::MAX_LENGTH);

    if (routes.begin()->connections.size() == Route::MAX_LENGTH) {
        LOG_ERROR("routes.begin()->connections.size() == Route::MAX_LENGTH");
    }

    if (0 == routes.size()) {
        LOG_ERROR("0 == routes.size()");
    }

    if (shortestRoute) {
        result = *shortestRoute;
    }

    return !!shortestRoute;
}

vec<Segment*, cfg::MAX_NUM_LAB_SEGMENTS> getFloatingSegments() {
    vec<Segment*, cfg::MAX_NUM_LAB_SEGMENTS> floatingSegments;
    for (Segment& seg : segments) {
        if (seg.isActive && seg.isFloating()) {
            floatingSegments.push_back(&seg);
        }
    }
    return floatingSegments;
}

bool getRoute() {
    bool success = false;
    plannedRoute.reset();

    vec<Segment*, cfg::MAX_NUM_LAB_SEGMENTS> floatingSegments = getFloatingSegments();

    LOG_DEBUG("Number of floating segments: %d", static_cast<int32_t>(floatingSegments.size()));

    if (floatingSegments.size()) {    // if there are still floating segments in the graph, chooses the closest one as destination
        Route route;
        for (Segment *seg : floatingSegments) {
            LOG_INFO("Planning route to %c", seg->name);
            if ((success |= getRoute(route, seg)) && (!plannedRoute.connections.size() || (plannedRoute.connections.size() > route.connections.size()))) {
                plannedRoute = route;
                if (1 == plannedRoute.connections.size()) {
                    break;
                }
            }
        }
    } else if (laneChange.seg && laneChange.lastJunc) { // no floating segments in the graph, new destination will be the lane change segment
        //success = getRoute(plannedRoute, laneChange.seg, laneChange.lastJunc, laneChange.lastManeuver);
        success = getRoute(plannedRoute, laneChange.seg);
    }

    return success;
}

Segment* createNewSegment() {
    Segment *seg = getNew(segments);
    if (seg) {
        seg->name      = segments.size() >= 27 ? 'a' + static_cast<char>(segments.size() - 27) : 'A' + static_cast<char>(segments.size() - 1);
        seg->length    = meter_t(0);
        seg->isDeadEnd = false;
        seg->isActive  = true;
    }
    return seg;
}

void addSegments(Junction * const junc, radian_t inOri, radian_t outOri, uint8_t numInSegments, Direction inSegmentDir, uint8_t numOutSegments) {

    junc->addSegment(currentSeg, inOri, inSegmentDir);

    switch (numInSegments) {
    case 2:
        junc->addSegment(createNewSegment(), inOri, inSegmentDir == Direction::LEFT ? Direction::RIGHT : Direction::LEFT);
        break;
    case 3:
        switch (inSegmentDir) {
        case Direction::LEFT:
            junc->addSegment(createNewSegment(), inOri, Direction::RIGHT);
            junc->addSegment(createNewSegment(), inOri, Direction::CENTER);
            break;
        case Direction::CENTER:
            junc->addSegment(createNewSegment(), inOri, Direction::RIGHT);
            junc->addSegment(createNewSegment(), inOri, Direction::LEFT);
            break;
        case Direction::RIGHT:
            junc->addSegment(createNewSegment(), inOri, Direction::CENTER);
            junc->addSegment(createNewSegment(), inOri, Direction::LEFT);
            break;
        }
        break;
    }

    switch (numOutSegments) {
    case 1:
        junc->addSegment(createNewSegment(), outOri, Direction::CENTER);
        break;
    case 2:
        junc->addSegment(createNewSegment(), outOri, Direction::RIGHT);
        junc->addSegment(createNewSegment(), outOri, Direction::LEFT);
        break;
    case 3:
        junc->addSegment(createNewSegment(), outOri, Direction::RIGHT);
        junc->addSegment(createNewSegment(), outOri, Direction::CENTER);
        junc->addSegment(createNewSegment(), outOri, Direction::LEFT);
        break;
    }
}

void addConnection(Connection * const conn, Segment * const seg1, const Maneuver& maneuver1, Segment * const seg2, const Maneuver& maneuver2) {
    conn->node1 = seg1;
    conn->node2 = seg2;
    conn->maneuver1 = maneuver1;
    conn->maneuver2 = maneuver2;
    seg1->edges.push_back(conn);
    seg2->edges.push_back(conn);
}

vec<Connection*, 2> getConnections(Segment *seg1, Segment *seg2) {
    vec<Connection*, 2> connections;
    for (Connection *c : seg1->edges) {
        if (c->getOtherSegment(seg1) == seg2) {
            connections.push_back(c);
        }
    }
    return connections;
}

Junction* onNewJunction(radian_t inOri, radian_t outOri, uint8_t numInSegments, Direction inSegmentDir, uint8_t numOutSegments) {
    Junction * const junc = getNew(junctions);
    if (junc) {
        junc->idx = junctions.size();
        junc->pos = globals::car.pose.pos;
        addSegments(junc, inOri, outOri, numInSegments, inSegmentDir, numOutSegments);

        Junction::segment_map::iterator inSegments  = junc->getSideSegments(inOri);
        Junction::segment_map::iterator outSegments = junc->getSideSegments(outOri);

        for (Junction::side_segment_map::iterator in = inSegments->second.begin(); in != inSegments->second.end(); ++in) {
            for (Junction::side_segment_map::iterator out = outSegments->second.begin(); out != outSegments->second.end(); ++out) {
                Connection *conn = getNew(connections);
                conn->junction = junc;
                addConnection(conn, in->second, { inSegments->first, in->first }, out->second, { outSegments->first, out->first });
            }
        }

    } else {
        LOG_ERROR("Junction pool empty");
    }

    return junc;
}

Status mergeSegments(Segment *oldSeg, Segment *newSeg) {
    Status result = Status::INVALID_DATA;

    if (oldSeg != newSeg) {
        if (newSeg->isFloating()) {
            // if both segments are floating, merges them
            if (oldSeg->isFloating()) {
                // adds all the connections of this segment to the previously defined floating segment (updates segment pointers)
                for (Connection *c : oldSeg->edges) {
                    c->updateSegment(oldSeg, newSeg);
                    newSeg->edges.push_back(c);
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

Connection* onExistingJunction(Junction *junc, radian_t inOri, Direction inSegmentDir) {

    LOG_DEBUG("Junction: %d", junc->idx);

    vTaskSuspendAll();
    const point2m prevCarPos = globals::car.pose.pos;
    globals::car.pose.pos = junc->pos;
    xTaskResumeAll();

    LOG_DEBUG("Car pos updated: (%f, %f) -> (%f, %f) | diff: %f [m]",
        prevCarPos.X.get(), prevCarPos.Y.get(), globals::car.pose.pos.X.get(), globals::car.pose.pos.Y.get(),
        globals::car.pose.pos.distance(prevCarPos).get());

    Connection *nextConn = nullptr;

    Segment * const junctionSeg = junc->getSegment(inOri, inSegmentDir);

    if (junctionSeg) {
        LOG_DEBUG("junctionSeg: %c", junctionSeg->name);

        if (currentSeg != junctionSeg) {
            if (!isOk(mergeSegments(currentSeg, junctionSeg))) {
                plannedRoute.reset();
            }
            currentSeg = junctionSeg;
        }

        // If there is no planned route, creates a new route.
        // The destination of the route will be the closest floating segment.
        // If there are no floating segments, it means the labyrinth has been completed.
        // In this case the destination will be the lane change segment.
        if (plannedRoute.connections.size() || getRoute()) {
            LOG_DEBUG("Planned route:");

            const Segment *prev = plannedRoute.startSeg;
            for (const Connection *c : plannedRoute.connections) {
                const Segment *next = c->getOtherSegment(prev);
                LOG_DEBUG("-> %c (%s)", next->name, to_string(c->getManeuver(next).direction));
                prev = next;
            }

            nextConn = plannedRoute.nextConnection();
        } else {
            LOG_ERROR("createNewRoute() failed");
        }
    } else {
        LOG_ERROR("Junction segment with orientation [%fdeg] and direction [%s] not found", static_cast<degree_t>(inOri).get(), to_string(inSegmentDir));
    }

    return nextConn;
}

uint32_t random(uint32_t interval) {
    return static_cast<uint32_t>(static_cast<centimeter_t>(globals::car.distance).get()) % interval;
}

void reset() {
    currentSeg = nullptr;
    segments.clear();
    connections.clear();
    junctions.clear();
    plannedRoute.reset();
    laneChange.seg = nullptr;
    laneChange.lastJunc = nullptr;
    prevConn = prevPrevConn = nullptr;
    currentSeg = createNewSegment();
    endTime = getTime() + millisecond_t(20);
}

Direction onJunctionDetected(radian_t inOri, radian_t outOri, uint8_t numInSegments, Direction inSegmentDir, uint8_t numOutSegments) {

    bool isJuncValid = false;
    Junction *junc = findExistingJunction(globals::car.pose.pos, inOri, outOri, numInSegments, inSegmentDir, numOutSegments, isJuncValid);

    if (junc && !isJuncValid) {
        reset();
        junc = nullptr;
        LOG_ERROR("Unexpected junction topology - labyrinth reset :(");
    }

    Connection *nextConn = nullptr;
    Maneuver nextManeuver = { outOri, Direction::CENTER };

    if (!junc) { // new junction found - creates new segments and adds connections
        LOG_DEBUG("new junction");
        junc = onNewJunction(inOri, outOri, numInSegments, inSegmentDir, numOutSegments);
    } else { // arrived at a previously found junction - the corresponding floating segment of the junction (if any) must be merged with the current segment
        LOG_DEBUG("existing junction");
        nextConn = onExistingJunction(junc, inOri, inSegmentDir);
    }

    LOG_DEBUG("currentSeg: %c, junction: %d (%f, %f)", currentSeg->name, junc->idx, junc->pos.X.get(), junc->pos.Y.get());

    if (nextConn) {
        nextManeuver = nextConn->getManeuver(nextConn->getOtherSegment(currentSeg));
    } else if (junc) {

#if RANDOM_SEGMENT
        const uint32_t numSegs = junc->getSideSegments(nextManeuver.orientation)->second.size();
        const uint32_t r = random(numSegs);
        if (1 == numSegs) {
            nextManeuver.direction = Direction::CENTER;
        } else if (2 == numSegs) {
            nextManeuver.direction = 0 == r ? Direction::LEFT : Direction::RIGHT;
        } else {
            nextManeuver.direction = static_cast<Direction>(r);
        }
        Segment **pNextSeg = outSegments->second.get(nextManeuver.direction);
#else
        Junction::segment_map::iterator outSegments = junc->getSideSegments(nextManeuver.orientation);
        Segment **pNextSeg = outSegments->second.get((nextManeuver.direction = Direction::RIGHT));
        if (!pNextSeg) {
            pNextSeg = outSegments->second.get((nextManeuver.direction = Direction::CENTER));
        }
#endif

        if (pNextSeg) {
            for (Connection *c : getConnections(currentSeg, *pNextSeg)) {
                if (c->junction == junc && c->getManeuver(*pNextSeg) == nextManeuver) {
                    nextConn = c;
                    break;
                }
            }
        } else {
            LOG_ERROR("pNextSeg should not be nullptr at this point");
        }
    } else {
        LOG_ERROR("junc should not be nullptr at this point");
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

        if (!nextSeg) {
            while(true) {
                LOG_ERROR("!nextSeg??");
                vTaskDelay(1000);
            }
        }

        if (nextSeg->isFloating()) {
            endTime = min(static_cast<second_t>(endTime) + second_t(10), second_t(20) + second_t(10) * 17);
        }

        currentSeg = nextSeg;
        prevPrevConn = prevConn;
        prevConn = nextConn;
        LOG_DEBUG("Next: %c (%s)", nextSeg->name, micro::to_string(nextManeuver.direction));
    } else {
        LOG_ERROR("No next connection found");
    }

    return nextManeuver.direction;
}

void updateCarOrientation() {
    static meter_t lastUpdatedOrientedDistance;

    if (globals::car.orientedDistance - lastUpdatedOrientedDistance > centimeter_t(40) &&
        eqWithOverflow360(globals::car.pose.angle, round90(globals::car.pose.angle), degree_t(8))) {
        vTaskSuspendAll();
        globals::car.pose.angle = round90(globals::car.pose.angle);
        xTaskResumeAll();
        lastUpdatedOrientedDistance = globals::car.orientedDistance;
    }
}

bool navigateLabyrinth(const DetectedLines& prevDetectedLines, const DetectedLines& detectedLines, ControlData& controlData) {

    static uint8_t numInSegments;
    static Direction inSegmentDir;
    static meter_t lastOriUpdateDist;

    static struct {
        Direction dir = Direction::CENTER;
        bool enabled = false;
        Line prevLine;
    } forcedManeuver;

    static constexpr millimeter_t FORCED_DISAPPEAR_POS = centimeter_t(8);

    static const bool runOnce = [&controlData]() {
        reset();
        controlData.speed = globals::speed_LAB_FWD;
        controlData.rampTime = millisecond_t(500);
        return true;
    }();
    UNUSED(runOnce);

    bool finished = false;

    updateCarOrientation();

    const bool isSafe = globals::car.orientedDistance > centimeter_t(50) &&
        LinePattern::SINGLE_LINE == detectedLines.pattern.type && !detectedLines.isPending;

    if (globals::speed_LAB_FWD == controlData.speed && isSafe) {
        controlData.speed = globals::speed_LAB_FWD_FAST;
        controlData.rampTime = millisecond_t(0);
    } else if (globals::speed_LAB_FWD_FAST == controlData.speed && !isSafe) {
        controlData.speed = globals::speed_LAB_FWD;
        controlData.rampTime = millisecond_t(0);
    }

    if (LinePattern::SINGLE_LINE == detectedLines.pattern.type && forcedManeuver.enabled) {
        if (abs(forcedManeuver.prevLine.pos) < FORCED_DISAPPEAR_POS) {
            forcedManeuver.enabled = false;
            LOG_DEBUG("Force maneuver disabled");
        }
    }

    if (detectedLines.pattern != prevDetectedLines.pattern) {

        switch (detectedLines.pattern.type) {
        case LinePattern::JUNCTION_1:
        case LinePattern::JUNCTION_2:
        case LinePattern::JUNCTION_3:
        {
            LOG_DEBUG("Junction pattern");
            const uint8_t numSegments = LinePattern::JUNCTION_1 == detectedLines.pattern.type ? 1 :
                LinePattern::JUNCTION_2 == detectedLines.pattern.type ? 2 : 3;

            if (Sign::NEGATIVE == detectedLines.pattern.dir) {
                numInSegments = numSegments;

                // Line pattern direction indicates on which side of the current line the OTHER lines are,
                // so if the current line is the leftmost line of three lines, pattern direction will be RIGHT.
                // Segment direction in a junction indicates which side the car should steer in order to follow a segment,
                // when leaving a junction.
                // Currently the car is entering a junction, therefore if the current line is the leftmost,
                // the car will need to steer to the right when leaving the junction.
                // So the segment direction is the same as the line pattern direction (in this example, RIGHT).
                inSegmentDir = detectedLines.pattern.side;

                // if the car is going backwards, mirrored pattern sides are detected
                if (currentSeg->isDeadEnd) {
                    inSegmentDir = -inSegmentDir;
                    controlData.speed = globals::speed_LAB_FWD;
                    controlData.rampTime = millisecond_t(1000);
                }

                // follows center line in junctions
                if (3 == detectedLines.lines.front.size()) {
                    controlData.baseline = detectedLines.lines.front[1];
                }

            } else if (Sign::POSITIVE == detectedLines.pattern.dir) {

                const radian_t carOri = globals::car.pose.angle;
                const radian_t inOri  = round90(carOri + PI);
                const radian_t outOri = round90(carOri);

                if (currentSeg->isDeadEnd) {
                    if (prevConn) {
                        // when coming back from a dead-end, junction should be handled as if the car
                        // arrived from the segment before the dead-end segment
                        currentSeg = prevConn->getOtherSegment(currentSeg);
                        numInSegments = prevConn->junction->getSideSegments(inOri)->second.size();
                        inSegmentDir = prevConn->getManeuver(currentSeg).direction;
                        globals::car.pose.pos = prevConn->junction->pos;
                        prevConn = prevPrevConn;
                    } else {
                        LOG_ERROR("Current segment is dead-end but there has been no junctions yet");
                    }
                }

                forcedManeuver.dir = onJunctionDetected(inOri, outOri, numInSegments, inSegmentDir, numSegments);
                forcedManeuver.enabled = true;
                forcedManeuver.prevLine = Line();
                LOG_DEBUG("Forced maneuver: %s", to_string(forcedManeuver.dir));
            }
            break;
        }

        case LinePattern::DEAD_END:
            currentSeg->isDeadEnd = true;
            controlData.speed = globals::speed_LAB_BWD;
            controlData.rampTime = millisecond_t(200);
            break;

        case LinePattern::LANE_CHANGE:
            if ((getFloatingSegments().size() && endTime - getTime() > second_t(10)) || (segments.size() - getFloatingSegments().size()) <= 12) {
                laneChange.seg = currentSeg;
                if (Sign::POSITIVE == detectedLines.pattern.dir && prevConn) {
                    laneChange.lastJunc = prevConn->junction;
                    laneChange.lastManeuver = prevConn->getManeuver(currentSeg);
                }
            } else {
                finished = true;
            }
            break;

        default:
            break;
        }
    }

    if (globals::car.orientedDistance > centimeter_t(50) &&
        eqWithOverflow360(globals::car.pose.angle, round90(globals::car.pose.angle), degree_t(10)) &&
        globals::car.distance - lastOriUpdateDist > centimeter_t(50)) {

        globals::car.pose.angle = round90(globals::car.pose.angle);
        lastOriUpdateDist = globals::car.distance;
    }

    if (forcedManeuver.enabled) {
        static constexpr millimeter_t MAX_LINE_JUMP = centimeter_t(3.5f);

        // updates the main line (if an error occurs, keeps previous line)
        switch (forcedManeuver.dir) {
        case Direction::LEFT:
            if (detectedLines.lines.front.size() >= 1) {
                controlData.baseline = detectedLines.lines.front[0];
            }
            if (forcedManeuver.prevLine.pos < -FORCED_DISAPPEAR_POS &&
                abs(forcedManeuver.prevLine.pos - controlData.baseline.pos) > MAX_LINE_JUMP) {
                controlData.baseline = forcedManeuver.prevLine;
            }
            break;
        case Direction::CENTER:
            if (detectedLines.lines.front.size() == 1) {
                controlData.baseline = detectedLines.lines.front[0];
            } else if (detectedLines.lines.front.size() == 3) {
                controlData.baseline = detectedLines.lines.front[1];
            }
            break;
        case Direction::RIGHT:
            if (detectedLines.lines.front.size() >= 2) {
                controlData.baseline = *detectedLines.lines.front.back();
            }
            if (forcedManeuver.prevLine.pos > FORCED_DISAPPEAR_POS &&
                abs(forcedManeuver.prevLine.pos - controlData.baseline.pos) > MAX_LINE_JUMP) {
                controlData.baseline = forcedManeuver.prevLine;
            }
            break;
        }

        forcedManeuver.prevLine = controlData.baseline;
    }

    controlData.rearServoEnabled = controlData.speed < m_per_sec_t(0) || forcedManeuver.enabled;

    return finished;
}

bool changeLane(const DetectedLines& detectedLines, ControlData& controlData) {

    static constexpr meter_t LANE_DISTANCE = centimeter_t(60);

    if (laneChange.trajectory.length() == meter_t(0)) {

        if (LinePattern::LANE_CHANGE == detectedLines.pattern.type) {
            if (Sign::POSITIVE == detectedLines.pattern.dir) {

                laneChange.trajectory.setStartConfig(Trajectory::config_t{
                    globals::car.pose.pos + vec2m(cfg::CAR_OPTO_CENTER_DIST, centimeter_t(0)).rotate(globals::car.pose.angle),
                    globals::speed_LANE_CHANGE
                }, globals::car.distance);

                laneChange.trajectory.appendSineArc(Trajectory::config_t{
                    laneChange.trajectory.lastConfig().pos + vec2m(centimeter_t(80), -LANE_DISTANCE).rotate(globals::car.pose.angle),
                    globals::speed_LANE_CHANGE
                }, globals::car.pose.angle, 30);

            } else if (Sign::NEGATIVE == detectedLines.pattern.dir) {

                laneChange.trajectory.setStartConfig(Trajectory::config_t{
                    globals::car.pose.pos + vec2m(cfg::CAR_OPTO_CENTER_DIST, centimeter_t(0)).rotate(globals::car.pose.angle),
                    globals::speed_LANE_CHANGE
                }, globals::car.distance);

                laneChange.trajectory.appendCircle(
                    laneChange.trajectory.lastConfig().pos + vec2m(centimeter_t(0), LANE_DISTANCE / 2).rotate(globals::car.pose.angle),
                    PI,
                    globals::speed_LANE_CHANGE, 30);

                laneChange.trajectory.appendLine(Trajectory::config_t{
                    laneChange.trajectory.lastConfig().pos + vec2m(centimeter_t(30), centimeter_t(0)).rotate(globals::car.pose.angle + PI),
                    globals::speed_LANE_CHANGE
                });

            }
        } else {
            LOG_ERROR("Current detected line pattern is [%s] (expected: [%s])", to_string(detectedLines.pattern.type), to_string(LinePattern::LANE_CHANGE));
        }

        globals::lineDetectionEnabled = false;
    }

    controlData = laneChange.trajectory.update(globals::car);
    controlData.rearServoEnabled = true;

    if (laneChange.trajectory.length() - laneChange.trajectory.coveredDistance() < centimeter_t(50)) {
        globals::lineDetectionEnabled = true;
    }

    const bool finished = laneChange.trajectory.length() - laneChange.trajectory.coveredDistance() < centimeter_t(40) && LinePattern::NONE != detectedLines.pattern.type;
    if (finished) {
        laneChange.trajectory.clear();
    }
    return finished;
}

Line avg(const Lines& lines) {
    Line line;
    for (const Line& l : lines) {
        line.pos += l.pos;
    }

    if (lines.size()) {
        line.pos /= lines.size();
    }
    return line;
}

} // namespace

extern "C" void runProgLabyrinthTask(void const *argument) {

    vTaskDelay(100); // gives time to other tasks and panels to wake up

    DetectedLines prevDetectedLines, detectedLines;
    ControlData controlData;

    while (true) {
        switch (getActiveTask(globals::programState)) {
            case ProgramTask::Labyrinth:
                globals::distServoEnabled = false;
                globals::distSensorEnabled = false;

                xQueuePeek(detectedLinesQueue, &detectedLines, 0);

                controlData.directControl = false;

                if (globals::car.speed >= m_per_sec_t(0)) {
                    micro::updateMainLine(detectedLines.lines.front, controlData.baseline);
                } else if (3 == detectedLines.lines.rear.size() && LinePatternCalculator::areClose(detectedLines.lines.rear)) {
                    controlData.baseline = avg(detectedLines.lines.rear);
                } else {
                    micro::updateMainLine(detectedLines.lines.rear, controlData.baseline);
                }

                controlData.angle = degree_t(0);
                controlData.offset = millimeter_t(0);

                switch (globals::programState) {
                case ProgramState::NavigateLabyrinth:
                    if (navigateLabyrinth(prevDetectedLines, detectedLines, controlData)) {
                        globals::programState = ProgramState::LaneChange;
                    }
                    break;

                case ProgramState::LaneChange:
                    if (changeLane(detectedLines, controlData)) {
                        globals::programState = ProgramState::ReachSafetyCar;
                    }
                    break;
                default:
                    break;
                }

                xQueueOverwrite(controlQueue, &controlData);
                prevDetectedLines = detectedLines;
                break;

            default:
                vTaskDelay(20);
                break;
        }

        vTaskDelay(2);
    }

    vTaskDelete(nullptr);
}
