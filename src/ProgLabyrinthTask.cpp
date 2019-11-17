#include <cfg_board.h>
#include <micro/task/common.hpp>
#include <micro/utils/units.hpp>
#include <micro/utils/time.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/Line.hpp>

#include <cfg_car.hpp>
#include <cfg_track.hpp>

#include <globals.hpp>
#include <LinePattern.hpp>
#include <LabyrinthGraph.hpp>

#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_uart.h>

#include <FreeRTOS.h>
#include <task.h>

using namespace micro;

namespace {
constexpr centimeter_t MIN_JUNCTION_POS_ACCURACY(100);
constexpr degree_t MERGE_VIRTUAL_LINE_ANGLE(30.0f);

uint8_t startCounterBuffer[1];
volatile char startCounter = '6';   // start counter will count back from 5 to 0

vec<Segment, 5 * cfg::MAX_NUM_LAB_SEGMENTS> segments;               // The segments.
vec<Junction, 5 * cfg::MAX_NUM_LAB_SEGMENTS> junctions;             // The junctions - at most the number of segments.
vec<Connection, 5 * cfg::MAX_NUM_LAB_SEGMENTS * 2> connections;     // The connections - 2 times the number of junctions.

vec<Segment*, cfg::MAX_NUM_LAB_SEGMENTS> floatingSegments;
Segment *currentSeg = nullptr;
Junction *lastJunc = nullptr;

Segment *laneChangeSeg = nullptr;
Junction *lastJuncBeforeLaneChange = nullptr;

bool onStartSeg = true; // indicates if car is on the start segment (needed for floating segment handling)
bool firstSingleLineAfterJunction = false;
bool nextJuncIsLastBeforeLaneChange = false;

meter_t lineFollowDirStartDist;

time_t endTime;

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
        this->lastSeg = reinterpret_cast<Segment*>(c->node1 == this->lastSeg ? c->node2 : c->node1);
    }

    Segment* nextSegment(Direction *pSteeringDir) {

        if (this->connections.size()) {
            Connection *c = this->connections[0];
            if (!c) {
                LOG_DEBUG("E: Route connection is NULL");
                *pSteeringDir = Direction::CENTER;
            } else if (!c->node1 || !c->node2) {
                LOG_DEBUG("E: Route connection node NULL");
                *pSteeringDir = Direction::CENTER;
            } else {
                if (c->node1 == this->startSeg) {
                    *pSteeringDir = c->dir;
                    this->startSeg = reinterpret_cast<Segment*>(c->node2);

                } else {    // startSeg = c.node2 -> direction needs to be inverted
                    *pSteeringDir = c->dir == Direction::LEFT ? Direction::RIGHT : Direction::LEFT;
                    this->startSeg = reinterpret_cast<Segment*>(c->node1);
                }

                this->connections.erase(this->connections.begin());
            }
        }

        return this->startSeg;
    }

    Junction* lastJunction() const {
        Junction *last = this->connections.size() ? this->connections[this->connections.size() - 1]->junction : nullptr;
        return last;
    }

    void reset() {
        this->startSeg = this->lastSeg = currentSeg;
        this->connections.clear();
    }
};

Route plannedRoute;  // Planned route, when there is a given destination - e.g. given floating segment or the segment where the lane-change will happen

template <typename T, uint32_t N>
T* getNew(vec<T, N>& vec) {
    vec.append(T());
    return vec.back();
}

void switchToSegment(Segment *seg, Junction *junc) {
    LOG_DEBUG("Next: %c", seg->name);
    if (seg) {
        currentSeg = seg;
    }
    if (junc) {
        lastJunc = junc;
    }
}

Junction* findExistingJunction(const point2m& pos, radian_t orientation, Sign patternDir) {
    Junction *result = nullptr;
    meter_t minDist = meter_t::ZERO();
    static constexpr radian_t EPS = degree_t(20.0f);

    for(Junction& j : junctions) {
        const meter_t dist = pos.distance(j.pos);

        const radian_t centerOri = micro::round90(patternDir == Sign::POSITIVE ? orientation : orientation + micro::PI);
        if ((minDist == meter_t::ZERO() || dist < minDist) && micro::eqWithOverflow360(j.getCenterOrientation().value(), centerOri, EPS)) {
            result = &j;
            minDist = dist;
        }
    }

    LOG_DEBUG("pos: %d, %d", (int32_t)static_cast<centimeter_t>(pos.X).get(), (int32_t)static_cast<centimeter_t>(pos.Y).get());
    if (result) {
        LOG_DEBUG("#%d:  %d, %d", result->idx, (int32_t)static_cast<centimeter_t>(result->pos.X).get(), (int32_t)static_cast<centimeter_t>(result->pos.Y).get());
    }

    if (minDist > MIN_JUNCTION_POS_ACCURACY) {  // result is further from the current position than the max, this junction is a new one
        result = nullptr;
    }

    return result;
}

bool getRoute(Route& result, const Segment *dest, const Junction *lastJuncBeforeDest = nullptr) {
    static constexpr uint32_t MAX_NUM_ROUTES = 64;

    vec<Route, MAX_NUM_ROUTES> routes;
    Route * const initialRoute = getNew(routes);
    initialRoute->startSeg = initialRoute->lastSeg = currentSeg;
    uint8_t depth = 0;
    Route *shortestRoute = nullptr;

    do {
        while (routes.size() > 0 && !shortestRoute) {
            Route *currentRoute = &routes[0];
            const Junction *_lastJunc = currentRoute->lastJunction();
            if (!_lastJunc) {
                _lastJunc = lastJunc;
            }

            if (currentRoute->lastSeg) {
                for (uint32_t i = 0; i < currentRoute->lastSeg->edges.size(); ++i) {
                    Connection *c = reinterpret_cast<Connection*>(currentRoute->lastSeg->edges[i]);

                    if (c) {
                        if (c->junction != _lastJunc &&    // does not permit going through the last junction again (going backwards)
                            !((c->node1 == currentSeg || c->node2 == currentSeg) && c->junction == lastJunc)) { // prevents circular routes

                            if (routes.size() >= MAX_NUM_ROUTES) {
                                LOG_DEBUG("E: routes.size() >= MAX_NUM_ROUTES");
                                break;
                            }
                            Route *newRoute = getNew(routes);
                            *newRoute = *currentRoute;
                            newRoute->append(c);

                            // if we reached the destination through the desired last junction, the shortest route has been found
                            if (newRoute->lastSeg == dest && (!lastJuncBeforeDest || c->junction == lastJuncBeforeDest)) {
                                shortestRoute = newRoute;
                                break;
                            }
                        }
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

bool createNewRoute() {
    bool success = false;
    plannedRoute.reset();

    if (floatingSegments.size()) {    // if there are still floating segments in the graph, chooses the closest one as destination
        Route route;
        for (const Segment *seg : floatingSegments) {
            if ((success |= getRoute(route, seg)) && (!plannedRoute.connections.size() || (plannedRoute.connections.size() > route.connections.size()))) {
                plannedRoute = route;
            }
        }
    } else if (laneChangeSeg && lastJuncBeforeLaneChange){    // there no floating segments in the graph, so the new destination will be the segment where the lane change must happen
        success = getRoute(plannedRoute, laneChangeSeg, lastJuncBeforeLaneChange);
    }

    return success;
}

Direction generateRandomDirection() {
    Direction dir = static_cast<int32_t>(static_cast<millimeter_t>(globals::car.pose.pos.Y).get()) % 2 ? Direction::LEFT : Direction::RIGHT;
    return dir;
}

Direction onJuntionDetected(const LinePattern& linePattern, const CarProps& car) {

    static constexpr radian_t PATTERN_DELTA_ORI_DEAD_BAND = degree_t(3.0f);
    const bool isCurrentSegCenter = linePattern.dir == Sign::POSITIVE;

    // current orientation must be a multiple of 90 degrees (track rules)
    //const radian_t prevCarOri = car.orientation();
    const radian_t carOri = micro::round90(car.pose.angle);
    //car.orientation_ = carOri;
    //LOG_DEBUG("Junction! pos: %f, %f | Car ori update: %f -> %f", car.pos().X.get<centimeters>(), car.pos().Y.get<centimeters>(), prevCarOri.get<degrees>(), car.orientation().get<degrees>());

    point2m junctionPos;
    if (isCurrentSegCenter) {
        junctionPos = car.pose.pos;
    } else {
        const point2m carPos = car.pose.pos;
        if (carOri == radian_t::ZERO()) {
            junctionPos = { carPos.X - cfg::MIN_JUNCTION_LENGTH, carPos.Y };
        } else if (carOri == micro::PI_2) {
            junctionPos = { carPos.X, carPos.Y - cfg::MIN_JUNCTION_LENGTH };
        } else if (carOri == micro::PI) {
            junctionPos = { carPos.X + cfg::MIN_JUNCTION_LENGTH, carPos.Y };
        } else if (carOri == 3 * micro::PI_2) {
            junctionPos = { carPos.X, carPos.Y + cfg::MIN_JUNCTION_LENGTH };
        }
    }

    Junction *junc = findExistingJunction(junctionPos, car.pose.angle, linePattern.dir);
    Segment *nextSeg;
    Direction steeringDir;    // Type of the junction (STRAIGHT or CURVE) where car should continue its route

    bool keepSegFloating = onStartSeg;  // will be used later when current segment is removed from the list of floating segments (startSeg will not be removed)
    onStartSeg = false;

    if (!junc) {    // new junction found - creates 2 new segments and adds connections
        LOG_DEBUG("New junction | currentSeg: %c", currentSeg->name);
        junc = getNew(junctions);
        junc->idx = junctions.size();
        junc->pos = junctionPos;

        Segment *seg1 = getNew(segments), *seg2 = getNew(segments);
        seg1->name = 'A' + static_cast<char>(segments.size() - 2);
        seg2->name = seg1->name + static_cast<char>(1);

        junc->addSegment(currentSeg);
        junc->addSegment(seg1);
        junc->addSegment(seg2);
        junc->setOrientation(currentSeg, car.pose.angle);

        Connection *conn1 = getNew(connections);
        Connection *conn2 = getNew(connections);
        conn1->junction = junc;
        conn2->junction = junc;

        if (isCurrentSegCenter) {
            junc->setCenterSegment(currentSeg);
            conn1->node1 = conn2->node1 = currentSeg;  // node1 is the center segment

            conn1->node2 = seg1;
            conn1->dir = Direction::LEFT;

            conn2->node2 = seg2;
            conn2->dir = Direction::RIGHT;

        } else {
            junc->setCenterSegment(seg1);
            conn1->node1 = conn2->node1 = seg1;  // seg1 is the center segment

            conn1->node2 = currentSeg;
            conn1->dir = linePattern.side;   // If negative pattern is to the LEFT, it means currentSeg->seg1 is a RIGHT change. Center seg is seg1: seg1->currentSeg is a LEFT change. Same for RIGHT.

            conn2->node2 = seg2;
            conn2->dir = linePattern.side == Direction::LEFT ? Direction::RIGHT : Direction::LEFT; // LEFT negative pattern -> seg1->currentSeg: LEFT change (see above) -> seg1->seg2: RIGHT change
        }

        conn1->updateNodes();
        conn2->updateNodes();
        floatingSegments.append(seg1);
        floatingSegments.append(seg2);

        LOG_DEBUG("New segments: %c %c", seg1->name, seg2->name);
        LOG_DEBUG("CENTER: %c", isCurrentSegCenter ? currentSeg->name : seg1->name);

        if (isCurrentSegCenter) {
            steeringDir = generateRandomDirection();
            nextSeg = (steeringDir == Direction::LEFT) ? seg1 : seg2;
        } else {
            steeringDir = (linePattern.side == Direction::LEFT) ? Direction::RIGHT : Direction::LEFT;
            nextSeg = seg1;
        }

        endTime += second_t(10);

    } else {    // arrived at a previously found junction - the floating segment of the junction (if there is one) must be merged with the current segment, if the current segment is floating as well

        LOG_DEBUG("Junction: %d | currentSeg: %c", junc->idx, currentSeg->name);

        point2m prevCarPos = globals::car.pose.pos;
        taskENTER_CRITICAL();
        globals::car.pose.pos += (junc->pos - junctionPos);
        taskEXIT_CRITICAL();
        LOG_DEBUG("Car pos update: (%d, %d) -> (%d, %d)",
            (int32_t)static_cast<centimeter_t>(prevCarPos.X).get(), (int32_t)static_cast<centimeter_t>(prevCarPos.Y).get(),
            (int32_t)static_cast<centimeter_t>(globals::car.pose.pos.X).get(), (int32_t)static_cast<centimeter_t>(globals::car.pose.pos.Y).get());

        Segment *junctionSeg = junc->getSegment(globals::car.pose.angle);

        if (junctionSeg) {
            LOG_DEBUG("junctionSeg: %c", junctionSeg->name);

            if (junctionSeg != currentSeg) {
                if (floatingSegments.find(junctionSeg) != floatingSegments.end()) {
                    LOG_DEBUG("junctionSeg floating (OK)");

                    // if both segments are floating, merges them
                    if (floatingSegments.find(currentSeg) != floatingSegments.end()) {
                        // adds all the connections of this segment to the previously defined floating segment (updates segment pointers)
                        for (Edge *e : currentSeg->edges) {
                            if (e) {
                                Connection *c = reinterpret_cast<Connection*>(e);
                                if (c->node1 == currentSeg) {
                                    c->node1 = junctionSeg;
                                } else {
                                    c->node2 = junctionSeg;
                                }

                                junctionSeg->edges.append(c);
                                c->junction->updateSegment(currentSeg, junctionSeg);
                            }
                        }

                        LOG_DEBUG("Merge: %c -> %c", currentSeg->name, junctionSeg->name);

                        floatingSegments.erase(junctionSeg);
                    }
                } else {
                    // junction segment not floating - probably an error has happened
                    // sets current segment to the new one, and creates new route

                    LOG_DEBUG("junctionSeg not floating (NOT OK)");
                    keepSegFloating = true;
                    plannedRoute.reset();
                }

                currentSeg = junctionSeg;
            }

            // If there is no planned route, creates a new route.
            // The destination of the route will be the closest floating segment.
            // If there are no floating segments, it means the labyrinth has been completed. In this case the destination will be the segment where the lane-change must happen.
            if (!plannedRoute.connections.size() && !createNewRoute()) {
                LOG_DEBUG("createNewRoute failed -> RANDOM");
                steeringDir = Direction::CENTER;

            } else {
                if (plannedRoute.lastSeg) {
                    LOG_DEBUG("Route dest: %c", plannedRoute.lastSeg->name);
                    nextSeg = plannedRoute.nextSegment(&steeringDir);

                    if (!nextSeg || steeringDir == Direction::CENTER) {
                        LOG_DEBUG("nextSegment error -> RANDOM");
                        steeringDir = Direction::CENTER;
                    }

                } else {
                    LOG_DEBUG("Route dest: NULL -> RANDOM");
                    steeringDir = Direction::CENTER;
                }
            }
        } else {
            LOG_DEBUG("junctionSeg not found");
            steeringDir = Direction::CENTER;
        }


    }

    // does not remove start segment from the floating segment list, because it is not finished when the car reaches the first junction
    // also, does not remove current segment if it has not been merged with another one (due to some measurement errors)
    if (!keepSegFloating) {
        floatingSegments.erase(currentSeg);
    }

    if (steeringDir == Direction::CENTER) {
        nextSeg = currentSeg;
        steeringDir = generateRandomDirection();
        plannedRoute.reset();
    }

    switchToSegment(nextSeg, junc);
    firstSingleLineAfterJunction = true;

    return steeringDir;
}

void waitStartSignal(void) {
    HAL_UART_Receive_DMA(uart_RadioModule, startCounterBuffer, 1);

    while(globals::startSignalEnabled && startCounter != '0') {
        LOG_DEBUG("Seconds until start: %c", startCounter);
        vTaskDelay(50);
    }

    HAL_UART_DMAStop(uart_RadioModule);
}

} // namespace

extern "C" void runProgLabyrinthTask(void const *argument) {

    vTaskDelay(100); // gives time to other tasks and panels to wake up

    while(true) { vTaskDelay(100); }

    taskENTER_CRITICAL();
    // TODO send speed to ControlTask
    taskEXIT_CRITICAL();

    waitStartSignal();

    LOG_DEBUG("Started!");

    endTime = getTime() + second_t(20);

    // initializes start segment
    currentSeg = getNew(segments);
    currentSeg->name = 'A';
    floatingSegments.append(currentSeg);
    onStartSeg = true;

    constexpr radian_t angle_LIMIT_SPEED_FAST = degree_t(3.0f);
    constexpr meter_t d_line_pos_LIMIT_SPEED_FAST = centimeter_t(0.5f);
    constexpr m_per_sec_t speed_SINGLE(1.1f), speed_SINGLE_FAST(1.1f), speed_JUNCTION(1.1f);

    radian_t laneChangeOrientation;  // desired orientation for car during lane change
    Line laneChangeLine;            // line to follow in order to reach lane change orientation

    enum class JunctionHandling {
        NEW,
        DELAYED,
        DONE
    };
    JunctionHandling juncHandling = JunctionHandling::NEW;

    bool laneChangeHandled = false;

    while (true) {
        switch (globals::programState.activeModule()) {
            case ProgramState::ActiveModule::Labyrinth:
//            { // TODO switch (globals::programState.subCntr())
//                if (linesLocal.hasValue()) {
//                    controlPropsLocal.line = linesLocal->centerLine;
//
//                    if (linesLocal->pattern.type == LinePattern::Type::JUNCTION || (linesLocal->leftLine.hasValue() && linesLocal->rightLine.hasValue())) {
//                        controlPropsLocal.speed = speed_JUNCTION;
//                    } else {
//                        static constexpr radian_t ORI_EPS(degrees(), 2.0f);
//                        if ((micro::isMultipleOf90(car_.orientation(), ORI_EPS) && abs(linesLocal->centerLine.angle) < angle_LIMIT_SPEED_FAST) ||
//                                (abs(prevLines.centerLine.pos - linesLocal->centerLine.pos) < d_line_pos_LIMIT_SPEED_FAST)) {
//                            controlPropsLocal.speed = speed_SINGLE_FAST;
//                        } else {
//                            controlPropsLocal.speed = speed_SINGLE;
//                        }
//                    }
//
//                    micro::enterCritical();
//                    controlProps = controlPropsLocal;
//                    micro::exitCritical();
//
//                    if (linesLocal->pattern.type == LinePattern::Type::JUNCTION) {
//
//                        static constexpr meter_t MIN_JUNCTION_DIST(centimeters(), 80);   // prevents re-detection
//
//                        if ((!lastJunc || carLocal.pos().distance(lastJunc->pos) > MIN_JUNCTION_DIST) && !firstSingleLineAfterJunction) {
//
//                            if (linesLocal->pattern.dir == Sign::NEGATIVE || abs(carLocal.distance() - linesLocal->pattern.startDist) > cfg::MIN_JUNCTION_LENGTH) {
//                                if (juncHandling == JunctionHandling::NEW) {
//                                    if (linesLocal->pattern.dir == Sign::POSITIVE) { // POSITIVE junctions are handled immediately, NEGATIVE junctions will be handled at the first SINGLE line (other end of junction)
//                                        lineFollowDir = onJuntionDetected(linesLocal.value(), carLocal);
//                                        lineFollowDirStartDist = carLocal.distance();
//                                        LOG_DEBUG("Keep %s", lineFollowDir == Direction::LEFT ? "LEFT" : "RIGHT");
//
//                                        if (nextJuncIsLastBeforeLaneChange) {
//                                            lastJuncBeforeLaneChange = lastJunc;
//                                            nextJuncIsLastBeforeLaneChange = false;
//                                        }
//                                        juncHandling = JunctionHandling::DONE;
//                                    } else {
//                                        juncHandling = JunctionHandling::DELAYED;
//                                    }
//                                }
//                            }
//                        }
//                    } else {
//
//                        if (juncHandling == JunctionHandling::DELAYED) {  // handles previous NEGATIVE junction (need to be handled after car has passed it for better localization)
//                            onJuntionDetected(prevLines, carLocal);   // does not set lineFollowDir (NEGATIVE junction -> direction is obvious)
//                        }
//
//                        juncHandling = JunctionHandling::NEW;
//
//                        static constexpr meter_t LINE_FOLLOW_FORCE_DISABLE_MAX_LINE_DIST(centimeters(), 2.0f);
//                        static constexpr meter_t LINE_FOLLOW_FORCE_MIN_DIST(centimeters(), 50);
//                        if (lineFollowDir.hasValue() &&
//                                abs(linesLocal->centerLine.pos) < LINE_FOLLOW_FORCE_DISABLE_MAX_LINE_DIST &&
//                                carLocal.distance() - lineFollowDirStartDist > LINE_FOLLOW_FORCE_MIN_DIST) {
//                            lineFollowDir.reset();  // disables forced line follow after car has found it
//                            LOG_DEBUG("Forced line follow disabled");
//                        }
//
//                        if (linesLocal->pattern.type == LinePattern::Type::SINGLE_LINE) {
//
//                            if (firstSingleLineAfterJunction) {
//                                LOG_DEBUG("firstSingleLineAfterJunction");
//                                optional<radian_t> lineOri;
//                                if (isOk(lastJunc->getOrientation(currentSeg, lineOri))) {
//                                    if (!lineOri.hasValue()) {
//
//                                        // CENTER segment's orientation is always a multiple of 90 degrees (track rules)
//                                        //car.orientation_ = micro::round90(car.orientation());
//
//                                        lastJunc->setOrientation(currentSeg, micro::round90(carLocal.orientation() + micro::PI));
//                                        LOG_DEBUG("Line: (%d, %d) %ddeg", (int32_t)carLocal.pos_.X.get<centimeters>(), (int32_t)carLocal.pos_.Y.get<centimeters>(), (int32_t)carLocal.orientation_.get<degrees>());
//
//                                    } else if (micro::isMultipleOf90(*lineOri, radian_t::ZERO())) {
//
//                                        const radian_t prevCarOri = car_.orientation();
//                                        //car.orientation_ = *lineOri;
//                                        //LOG_DEBUG("Car ori update: %f -> %f", prevCarOri.get<degrees>(), carLocal.orientation().get<degrees>());
//                                    }
//                                }
//
//                                firstSingleLineAfterJunction = false;
//                            }
//
//                        } else if (linesLocal->pattern.type == LinePattern::Type::LANE_CHANGE) {
//
//                            bool change = false;
//                            if (!laneChangeSeg || !lastJuncBeforeLaneChange) {   // if this is the first time we are at this section, we need to save it
//                                laneChangeSeg = currentSeg;
//                                if (linesLocal->pattern.dir == Sign::POSITIVE) {
//                                    lastJuncBeforeLaneChange = lastJunc;
//
//                                    if (segments.size() >= 10 && carLocal.distance() - startDist > meter_t::from<meters>(80)) {
//                                        change = true;
//                                    }
//
//                                } else {
//                                    nextJuncIsLastBeforeLaneChange = true;
//                                }
//
//                            } else {
//                                // if our destination was this segment (the lane change segment), then the car has finished the labyrinth, it needs to change lanes
//                                if (!plannedRoute.connections.size() && plannedRoute.lastSeg == currentSeg) {
//                                    change = true;
//                                }
//                            }
//
//                            if (change && false) {
//                                LOG_DEBUG("LaneChange started");
//
//                                micro::enterCritical();
//                                carLocal.orientation_ = car_.orientation_ = micro::PI_2;  // TODO currently lane change only works when initial orientation is 90 degrees. This ruins the map, but lane change happens after the labyrinth
//                                micro::exitCritical();
//
//                                micro::changeLane(Direction::RIGHT, cfg::LANE_CHANGE_LINE_DIST, carLocal.orientation(), speed_t::from<m_per_sec>(0.5f));
//                                LOG_DEBUG("LaneChange over");
//                                PROGRAM_TASK = ProgramTask::SAFETY_CAR_FOLLOW;
//                            }
//                        }
//                    }
//
//                    prevLines = linesLocal.value();
//                }
//
//                vTaskDelay(2);
//                break;
//            }
            default:
                vTaskDelay(100);
                break;
        }
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
