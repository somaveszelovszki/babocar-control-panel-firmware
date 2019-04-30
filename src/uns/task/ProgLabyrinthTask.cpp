#include <uns/task/common.hpp>
#include <config/cfg_board.hpp>
#include <config/cfg_car.hpp>
#include <uns/util/debug.hpp>
#include <uns/util/unit_utils.hpp>
#include <uns/bsp/task.hpp>
#include <uns/LineData.hpp>
#include <uns/CarProps.hpp>
#include <uns/ControlProps.hpp>
#include <uns/LineController.hpp>
#include <uns/LabyrinthGraph.hpp>

using namespace uns;

extern ProgramTask PROGRAM_TASK;
extern CarProps car;

namespace {
volatile bool x = false;
constexpr distance_t MIN_JUNCTION_POS_ACCURACY(centimeters(), 40);
constexpr angle_t MERGE_LINE_ANGLE(degrees(), 45.0f);

char startCounterBuffer;
volatile char startCounter = '6';   // start counter will count back from 5 to 0
volatile char cntrs[100];
volatile int idx = 0;

struct Route {
    static constexpr uint32_t MAX_LENGTH = cfg::MAX_NUM_LAB_SEGMENTS * 2;
    Segment *startSeg;
    Segment *lastSeg;
    Vec<Connection*, MAX_LENGTH> connections;

    void append(Connection *c) {
        this->connections.append(c);
        this->lastSeg = reinterpret_cast<Segment*>(c->node1 == this->lastSeg ? c->node2 : c->node1);
    }

    Segment *nextSegment(Connection::Type *pType) {
        Connection *c = this->connections[0];
        *pType = c->type;
        this->startSeg = reinterpret_cast<Segment*>(c->node1 == this->startSeg ? c->node2 : c->node1);
        this->connections.remove(c);
        return this->startSeg;
    }

    Junction* lastJunction() const {
        Junction *last = this->connections.size ? this->connections[this->connections.size - 1]->junction : nullptr;
        return last;
    }
};

Vec<Segment, cfg::MAX_NUM_LAB_SEGMENTS> segments;                       // The segments.
Vec<Junction, cfg::MAX_NUM_LAB_SEGMENTS> junctions;                     // The junctions - at most the number of segments.
Vec<Connection, cfg::MAX_NUM_LAB_SEGMENTS * 2> connections;    // The connections - 2 times the number of junctions.

Vec<Segment*, cfg::MAX_NUM_LAB_SEGMENTS> floatingSegments;
Segment *currentSeg = nullptr;
Junction *lastJunc = nullptr;
Route plannedRoute;  // Planned route, when there is a given destination - e.g. given floating segment or the segment where the lane-change will happen

Segment *laneChangeSeg = nullptr;
Junction *lastJuncBeforeLaneChange = nullptr;

bool onStartSeg;    // indicates if car is on the start segment (needed for floating segment handling)

LineData lines;
ControlProps controlProps;

bool randomChange = false;

template <typename T, uint32_t N>
T* getNew(Vec<T, N>& vec) {
    return &(vec[vec.size++] = T());
}

void switchToSegment(Segment *seg, Junction *junc) {
    currentSeg = seg;
    lastJunc = junc;
}

bool junctionOrientationsMatch(const Junction *junc, angle_t oriCenter, angle_t oriLeft, angle_t oriRight) {
    static constexpr angle_t EPS(degrees(), 20.0f);

    angle_t ori;
    return isOk(junc->getOrientation(RotationDir::CENTER, &ori)) && uns::eqWithOverflow360(oriCenter, ori, EPS)
        && isOk(junc->getOrientation(RotationDir::LEFT, &ori)) && uns::eqWithOverflow360(oriLeft, ori, EPS)
        && isOk(junc->getOrientation(RotationDir::RIGHT, &ori)) && uns::eqWithOverflow360(oriRight, ori, EPS);
}

Junction* findExistingJunction(const Point2<distance_t>& pos, angle_t oriCenter, angle_t oriLeft, angle_t oriRight) {
    Junction *result = nullptr;
    distance_t minDist = distance_t::ZERO();

    for(Junction& j : junctions) {
        const distance_t dist = pos.distance(j.pos);

        if ((minDist == distance_t::ZERO() || dist < minDist) && junctionOrientationsMatch(&j, oriCenter, oriLeft, oriRight)) {
            result = &j;
            minDist = dist;
        }
    }

    //debug::printlog("pos: %d, %d", (int32_t)pos.X.get<centimeters>(), (int32_t)pos.Y.get<centimeters>());
    if (result) {
        //debug::printlog("#%d:  %d, %d", (int32_t)result->pos.X.get<centimeters>(), (int32_t)result->pos.Y.get<centimeters>());
    }

    if (minDist > MIN_JUNCTION_POS_ACCURACY) {  // result is further from the current position than the max, this junction is a new one
        result = nullptr;
    }

    return result;
}

bool getRoute(Route& result, const Segment *dest, const Junction *lastJuncBeforeDest = nullptr) {
    static constexpr uint32_t MAX_NUM_ROUTES = 64;

    Vec<Route, MAX_NUM_ROUTES> routes;
    Route * const initialRoute = getNew(routes);
    initialRoute->startSeg = initialRoute->lastSeg = currentSeg;
    uint8_t depth = 0;
    Route *shortestRoute = nullptr;

    do {
        uint32_t i = 0, end = routes.size;
        while (i != end && !shortestRoute) {
            Route *currentRoute = &routes[i];
            const Junction *_lastJunc = currentRoute->lastJunction();
            if (!_lastJunc) {
                _lastJunc = lastJunc;
            }

            for (uint32_t i = 0; i < currentRoute->lastSeg->edges.size && !shortestRoute; ++i) {
                Connection *c = reinterpret_cast<Connection*>(currentRoute->lastSeg->edges[i]);

                if (c->junction != _lastJunc &&    // does not permit going through the last junction again (going backwards)
                    !((c->node1 == currentSeg || c->node2 == currentSeg) && c->junction == lastJunc)) { // prevents circular routes

                    Route *newRoute = getNew(routes);
                    *newRoute = *currentRoute;
                    newRoute->append(c);

                    // if we reached the destination through the desired last junction, the shortest route has been found
                    if (newRoute->lastSeg == dest && (!lastJuncBeforeDest || c->junction == lastJuncBeforeDest)) {
                        shortestRoute = newRoute;
                    }
                }
            }
            routes.remove(i);
            --end;
        }
    } while(!shortestRoute && routes.size && ++depth < Route::MAX_LENGTH);

    if (shortestRoute) {
        result = *shortestRoute;
    }

    return !!shortestRoute;
}

bool createNewRoute() {
    bool success = false;
    plannedRoute.connections.clear();

    if (floatingSegments.size) {    // if there are still floating segments in the graph, chooses the closest one as destination
        Route route;
        for (const Segment *seg : floatingSegments) {
            if ((success |= getRoute(route, seg)) && (!plannedRoute.connections.size || (plannedRoute.connections.size > route.connections.size))) {
                plannedRoute = route;
            }
        }
    } else {    // there no floating segments in the graph, so the new destination will be the segment where the lane change must happen
        getRoute(plannedRoute, laneChangeSeg, lastJuncBeforeLaneChange);
    }

    return success;
}

Connection::Type onJuntionDetected(const Point2<distance_t>& pos, Connection::Type type, RotationDir side) {

    const PosOri currentPosOri = car.getPosOri();

    // sets segment orientations
    angle_t oriCenter, oriLeft, oriRight;

    if (type == Connection::Type::STRAIGHT) {
        oriCenter = uns::normalize360(currentPosOri.orientation_ + uns::PI);

        if (side == RotationDir::LEFT) {    // forward only (lane from left)
            oriLeft = currentPosOri.orientation_;
            oriRight = uns::normalize360(currentPosOri.orientation_ - MERGE_LINE_ANGLE);
        } else {    // forward only (lane from right)
            oriRight = currentPosOri.orientation_;
            oriLeft = uns::normalize360(currentPosOri.orientation_ + MERGE_LINE_ANGLE);
        }

    } else {    // CURVE
        oriCenter = currentPosOri.orientation_;

        if (side == RotationDir::LEFT) {    // forward / left
            oriRight = uns::normalize360(currentPosOri.orientation_ + uns::PI);
            oriLeft = uns::normalize360(currentPosOri.orientation_ + uns::PI + MERGE_LINE_ANGLE);
        } else {    // forward / right
            oriLeft = uns::normalize360(currentPosOri.orientation_ + uns::PI);
            oriRight = uns::normalize360(currentPosOri.orientation_ - uns::PI + MERGE_LINE_ANGLE);
        }
    }

    Junction *junc = findExistingJunction(pos, oriCenter, oriLeft, oriRight);
    Segment *nextSeg;
    Connection::Type continueType;    // Type of the junction (STRAIGHT or CURVE) where car should continue its route

    if (!junc) {    // new junction found - creates 2 new segments and adds connections
        //debug::printlog("New junction | currentSeg: %c", currentSeg->name);
        junc = getNew(junctions);
        junc->idx = junctions.size;
        junc->pos = pos;

        Segment *segStraight = getNew(segments), *segCurve = getNew(segments);
        segStraight->name = 'A' + static_cast<char>(segments.size - 2);
        segCurve->name = 'A' + static_cast<char>(segments.size - 1);

        Connection *connStraight = getNew(connections);
        connStraight->node1 = currentSeg;
        connStraight->node2 = segStraight;
        connStraight->junction = junc;
        connStraight->type = Connection::Type::STRAIGHT;
        connStraight->updateNodes();
        floatingSegments.append(segStraight);

        Connection *connCurve = getNew(connections);
        connCurve->node1 = type == Connection::Type::CURVE ? currentSeg : segStraight;
        connCurve->node2 = segCurve;
        connCurve->junction = junc;
        connCurve->type = Connection::Type::CURVE;
        connCurve->updateNodes();
        floatingSegments.append(segCurve);

        // sets segment orientations
        Segment *center, *left, *right;
        if (type == Connection::Type::STRAIGHT) {
            center = segStraight;

            if (side == RotationDir::LEFT) {    // forward only (lane from left)
                left = currentSeg;
                right = segCurve;
            } else {    // forward only (lane from right)
                right = currentSeg;
                left = segCurve;
            }

        } else {    // CURVE
            center = currentSeg;

            if (side == RotationDir::LEFT) {    // forward / left
                right = segStraight;
                left = segCurve;
            } else {    // forward / right
                left = segStraight;
                right = segCurve;
            }
        }

        junc->setOrientation(center, RotationDir::CENTER, oriCenter);
        junc->setOrientation(left, RotationDir::LEFT, oriLeft);
        junc->setOrientation(right, RotationDir::RIGHT, oriRight);

        //debug::printlog("New segments: %c %c", segStraight->name, segCurve->name);
        //debug::printlog("CENTER: %c", junc->getSegment(RotationDir::CENTER)->name);
        //debug::printlog("LEFT: %c", junc->getSegment(RotationDir::LEFT)->name);
        //debug::printlog("RIGHT: %c", junc->getSegment(RotationDir::RIGHT)->name);

        nextSeg = segStraight;  // continues to the straight segment by default
        continueType = Connection::Type::STRAIGHT;

    } else {    // arrived at a previously found junction - the floating segment of the junction (if there is one) must be merged with the current segment, if the current segment is floating as well

        //debug::printlog("Junction: %d | currentSeg: %c", junc->idx, currentSeg->name);

//        junc->pos = junc->pos.average(pos);
        Point2<distance_t> prevCarPos = car.pos();
        car.pos_ += (junc->pos - pos);

        //debug::printlog("Car pos update: (%d, %d) -> (%d, %d)", (int32_t)prevCarPos.X.get<centimeters>(), (int32_t)prevCarPos.Y.get<centimeters>(), (int32_t)car.pos_.X.get<centimeters>(), (int32_t)car.pos_.Y.get<centimeters>());

        if (floatingSegments.find(currentSeg) != floatingSegments.end()) {
            Segment *junctionSeg = junc->getSegment(type, side);

            // If by the current line orientation, the estimated junction segment differs from the current segment, it can mean 3 things:
            // 1.) The junction segment is floating and needs to be merged with the current segment. This is the normal behaviour.
            // 2.) The pattern detection failed and ruined the segment search.
            // 3.) The junction segment is not floating and the current segment is not part of the junction. This happens when the car position drift is so large, that another junction was detected.
            if (junctionSeg != currentSeg) {
                if (floatingSegments.find(junctionSeg) != floatingSegments.end()) { // junction segment is floating - normal behaviour

                    // adds all the connections of this segment to the previously defined floating segment (updates segment pointers)
                    for (Edge *e : currentSeg->edges) {
                        Connection *c = reinterpret_cast<Connection*>(e);
                        if (c->node1 == currentSeg) {
                            c->node1 = junctionSeg;
                        } else {
                            c->node2 = junctionSeg;
                        }

                        junctionSeg->edges.append(c);
                        c->junction->updateSegment(currentSeg, junctionSeg);
                    }

                    //debug::printlog("Merge: %c -> %c", currentSeg->name, junctionSeg->name);

                    floatingSegments.remove(junctionSeg);
                }

                currentSeg = junctionSeg;
            }
        }

        // If there is no planned route, creates a new route.
        // The destination of the route will be the closest floating segment.
        // If there are no floating segments, it means the labyrinth has been completed. In this case the destination will be the segment where the lane-change must happen.
        if (!plannedRoute.connections.size && !createNewRoute()) {
            // program should never get here, this is only a safety precaution (without this, the program fails)
            nextSeg = currentSeg;
            uint32_t random = uns::getTimerCompare(cfg::tim_DC_Motor, cfg::tim_chnl_DC_Fwd);
            continueType = random % 2 ? Connection::Type::STRAIGHT : Connection::Type::CURVE;
            randomChange = true;
            //debug::printlog("Random control :(");
        } else {
            //debug::printlog("Route dest: %c", plannedRoute.lastSeg->name);
            nextSeg = plannedRoute.nextSegment(&continueType);
        }

//        angle_t junctionSegLineOri;
//        if (isOk(junc->getOrientation(currentSeg, &junctionSegLineOri))) {
//            car.orientation_ += (junctionSegLineOri - currentLineOri); // compensates orientation error
//            car.orientation_ = uns::normalize360(car.orientation_);
//        }
    }

    // does not remove start segment from the floating segment list, because it is not finished when the car reaches the first junction
    if (onStartSeg) {
        onStartSeg = false;
    } else {
        floatingSegments.remove(currentSeg);
    }

    // TODO calculate currentSeg's length
    currentSeg->length = distance_t::from<meters>(0);

    //debug::printlog("Next: %c", nextSeg->name);

    switchToSegment(nextSeg, junc);

    return continueType;
}

} // namespace

extern "C" void runProgLabyrinthTask(void const *argument) {
    Status status;

    //test();

    if (cfg::START_SIGNAL_ENABLED) {
        uns::UART_Receive_DMA(cfg::uart_RadioModule, reinterpret_cast<uint8_t*>(&startCounterBuffer), 1);
        while(startCounter != '0') {
            //debug::printlog("Seconds until start: %c", startCounter);
            uns::nonBlockingDelay(time_t::from<milliseconds>(50));
        }
    }

    //debug::printlog("Started!");

    // initializes start segment
    currentSeg = getNew(segments);
    floatingSegments.append(currentSeg);
    onStartSeg = true;

    bool junctionHandled = false;
    bool firstSingleLineAfterJunction = false;
    bool nextJuncIsLastBeforeLaneChange = false;
    int32_t junctionLineIdx = -1;   // index of the junction line to follow

    controlProps.speed = speed_t::from<m_per_sec>(1.2f); // TODO 1.2 m/sec

    angle_t virtualLineAngle = angle_t::ZERO();   // When car needs to change lane (follow the CURVE), we send a virtual line orientation to the controller for fast change.

    bool laneChangeActive = false;
    angle_t laneChangeOrientation;  // desired orientation for car during lane change
    Line laneChangeLine;            // line to follow in order to reach lane change orientation

    while (!uns::hasErrorHappened()) {
        switch (PROGRAM_TASK) {
            case ProgramTask::LABYRINTH:
            {
                if (isOk(status = uns::queueReceive(cfg::queue_DetectedLines, &lines))) {
                    if (laneChangeActive) {

                    } else {
                        if (lines.pattern.type == LinePattern::Type::JUNCTION) {

                            if (lines.pattern.startPos != LinePattern::UNKNOWN_POS) {   // junction pattern over -> we need to handle the junction (if haven't already)
                                if (!junctionHandled) {

                                    const Point2<distance_t> pos = lines.pattern.startPos;  // equal to lines.pattern.endPos
                                    const Connection::Type type = lines.pattern.dir == Sign::POSITIVE ? Connection::Type::CURVE : Connection::Type::STRAIGHT;

                                    Connection::Type continueType = onJuntionDetected(pos, type, lines.pattern.side);

                                    // sets ControlProps (that will be sent to the ControlTask)
                                    if (continueType == Connection::Type::CURVE) {    // follow the curve

                                        if (lines.pattern.side == RotationDir::LEFT) {
                                            junctionLineIdx = 0;
                                            if (type == Connection::Type::CURVE) {
                                                virtualLineAngle = MERGE_LINE_ANGLE;
                                                //debug::printlog("CURVE -> Left");
                                            } else {
                                                virtualLineAngle = angle_t::ZERO();
                                                //debug::printlog("STRAIGHT ONLY -> Left");
                                            }
                                        }else {
                                            junctionLineIdx = 1;
                                            if (type == Connection::Type::CURVE) {
                                                virtualLineAngle = -MERGE_LINE_ANGLE;
                                                //debug::printlog("CURVE -> Right");
                                            } else {
                                                virtualLineAngle = angle_t::ZERO();
                                                //debug::printlog("STRAIGHT ONLY -> Right");
                                            }
                                        }

                                    } else {    // follow the straight (main) line
                                        virtualLineAngle = angle_t::ZERO();

                                        if (lines.pattern.side == RotationDir::LEFT) {
                                            junctionLineIdx = 1;
                                            //debug::printlog("STRAIGHT -> Right");
                                        }else {
                                            junctionLineIdx = 0;
                                            controlProps.line.pos = lines.coarsePositions[0];
                                            //debug::printlog("STRAIGHT -> Left");
                                        }
                                    }

                                    if (nextJuncIsLastBeforeLaneChange) {
                                        lastJuncBeforeLaneChange = lastJunc;
                                        nextJuncIsLastBeforeLaneChange = false;
                                    }

                                    junctionHandled = true;
                                    firstSingleLineAfterJunction = true;
                                } else {    // junction already handled
                                    // Note: Does not change virtualLineAngle!
                                }
                            } else {    // pattern has not been finished yet
                                virtualLineAngle = angle_t::ZERO();
                            }

                            if (junctionLineIdx == -1) {
                                controlProps.line = lines.centerLine;
                            } else {
                                controlProps.line.pos = lines.coarsePositions[junctionLineIdx];
                                //controlProps.line.angle = lines.centerLine.angle;
                                controlProps.line.angle = virtualLineAngle;
                            }

                        } else {
                            junctionHandled = false;
                            junctionLineIdx = -1;
                            virtualLineAngle = angle_t::ZERO();

                            if (lines.pattern.type == LinePattern::Type::SINGLE_LINE) {

                                if (firstSingleLineAfterJunction) {

                                    PosOri linePosOri;
                                    const PosOri currentPosOri = car.getPosOri();

                                    if (isOk(status = currentSeg->getEndPosOri(lastJunc, &linePosOri))) { // gets position and orientation of the new line
                                        const PosOri prevCarPosOri = car.getPosOri();
                                        car.pos_ += (linePosOri.pos_ - currentPosOri.pos_);
                                        car.orientation_ += uns::normalize360(2 * car.orientation_ - currentPosOri.orientation_); // compensates orientation error
                                        //debug::printlog("Car: (%d, %d) %ddeg -> (%d, %d) %ddeg", (int32_t)prevCarPosOri.pos_.X.get<centimeters>(), (int32_t)prevCarPosOri.pos_.Y.get<centimeters>(), (int32_t)prevCarPosOri.orientation_.get<degrees>(), (int32_t)car.pos().X.get<centimeters>(), (int32_t)car.pos().Y.get<centimeters>(), (int32_t)car.orientation().get<degrees>());
                                    } else  {   // if line orientation read is unsuccessful, it is because no orientation has been saved yet
                                        currentSeg->setEndPosOri(lastJunc, currentPosOri);
                                        //debug::printlog("Line: (%d, %d) %ddeg", (int32_t)currentPosOri.pos_.X.get<centimeters>(), (int32_t)currentPosOri.pos_.Y.get<centimeters>(), (int32_t)currentPosOri.orientation_.get<degrees>());
                                    }

                                    firstSingleLineAfterJunction = false;
                                }

                            } else if (lines.pattern.type == LinePattern::Type::LANE_CHANGE) {
                                if (!laneChangeSeg) {   // if this is the first time we are at this section, we need to save it
                                    laneChangeSeg = currentSeg;
                                    if (lines.pattern.dir == Sign::POSITIVE) {
                                        lastJuncBeforeLaneChange = lastJunc;
                                    } else {
                                        nextJuncIsLastBeforeLaneChange = true;
                                    }
                                } else {
                                    // if our destination was this segment (the lane change segment), then the car has finished the labyrinth, it needs to change lanes
                                    if (!plannedRoute.connections.size && plannedRoute.lastSeg == currentSeg) {
                                        laneChangeActive = true;
                                        laneChangeLine.pos = distance_t::ZERO();

                                        if (lines.pattern.side == RotationDir::LEFT) {
                                            laneChangeLine.angle = MERGE_LINE_ANGLE;
                                            laneChangeOrientation = car.orientation() + MERGE_LINE_ANGLE;
                                        } else {
                                            laneChangeLine.angle = -MERGE_LINE_ANGLE;
                                            laneChangeOrientation = car.orientation() - MERGE_LINE_ANGLE;
                                        }
                                    }
                                }
                            }

                            controlProps.line = lines.centerLine;
                        }
                    }

                    controlProps.line = lines.centerLine;
                    status = uns::queueSend(cfg::queue_ControlProps, &controlProps);
                    if (!isOk(status)) {
                        ////debug::printerr(status, "Error while receiving detected lines!");
                    }
                }
                break;
            }
            default:
                break;
        }

        uns::nonBlockingDelay(time_t::from<milliseconds>(5));
    }

    uns::deleteCurrentTask();
}

/* @brief Callback for RadioModule UART RxCplt - called when receive finishes.
 */
void uns_RadioModule_Uart_RxCpltCallback() {
    if (startCounterBuffer == startCounter - 1) {
        startCounter = startCounterBuffer;
    }
}
