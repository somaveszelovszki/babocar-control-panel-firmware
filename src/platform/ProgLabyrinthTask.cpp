#include <micro/debug/SystemManager.hpp>
#include <micro/port/queue.hpp>
#include <micro/port/task.hpp>
#include <micro/math/numeric.hpp>
#include <micro/math/unit_utils.hpp>
#include <micro/utils/CarProps.hpp>
#include <micro/utils/ControlData.hpp>
#include <micro/utils/LinePattern.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/timer.hpp>
#include <micro/utils/trajectory.hpp>
#include <micro/utils/units.hpp>

#include <cfg_board.hpp>
#include <cfg_car.hpp>
#include <cfg_track.hpp>
#include <LaneChangeManeuver.hpp>
#include <LabyrinthGraphBuilder.hpp>
#include <LabyrinthNavigator.hpp>

using namespace micro;

extern queue_t<CarProps, 1> carPropsQueue;
extern queue_t<point2m, 1> carPosUpdateQueue;
extern queue_t<radian_t, 1> carOrientationUpdateQueue;
extern queue_t<linePatternDomain_t, 1> linePatternDomainQueue;
extern queue_t<LineInfo, 1> lineInfoQueue;
extern queue_t<ControlData, 1> controlQueue;
extern queue_t<radian_t, 1> carOrientationUpdateQueue;
extern queue_t<state_t<char>, 1> radioRecvQueue;
extern Sign safetyCarFollowSpeedSign;

namespace {

m_per_sec_t LABYRINTH_FORWARD_SPEED      = m_per_sec_t(1.2f);
m_per_sec_t LABYRINTH_FORWARD_SLOW_SPEED = m_per_sec_t(0.9f);
m_per_sec_t LABYRINTH_BACKWARD_SPEED     = m_per_sec_t(-0.8f);
m_per_sec_t LANE_CHANGE_SPEED            = m_per_sec_t(0.8f);

constexpr meter_t LANE_DISTANCE = centimeter_t(60);

#if LABYRINTH == TEST_LABYRINTH
#define START_SEGMENT   'A'
#define PREV_SEGMENT    'X'
#elif LABYRINTH == RACE_LABYRINTH
#define START_SEGMENT   'A'
#define PREV_SEGMENT    'X'
#endif

const LabyrinthGraph graph = buildLabyrinthGraph();
const LabyrinthGraph::Segments::const_iterator startSeg = graph.findSegment(START_SEGMENT);
LabyrinthNavigator navigator(graph, *graph.findConnection(*startSeg, *graph.findSegment(PREV_SEGMENT)), *startSeg);
LabyrinthGraph::Segments foundSegments;
millisecond_t endTime;

struct JunctionPatternInfo {
    Sign dir            = Sign::NEUTRAL;
    Direction side      = Direction::CENTER;
    uint8_t numSegments = 0;
};

LaneChangeManeuver laneChange;
Segment *laneChangeSeg = nullptr; // TODO

bool isJunction(const LinePattern& pattern) {
    return LinePattern::JUNCTION_1 == pattern.type ||
           LinePattern::JUNCTION_2 == pattern.type ||
           LinePattern::JUNCTION_3 == pattern.type;
}

uint8_t getNumSegments(const LinePattern& pattern) {
    return isJunction(pattern) ? static_cast<uint8_t>(pattern.type) - static_cast<uint8_t>(LinePattern::JUNCTION_1) + 1 : 0;
}

void resetNavigator(const Junction& junc, radian_t negOri) {
    // Finds a valid current segment - may be any of the segments behind the car, connecting to the current junction.
    // This will be the start segment for the next route.
    const Segment *currentSeg      = junc.getSideSegments(negOri)->second.begin()->second;

    // Finds a valid junction connection - may be any of the current segment's connections that are linked to the current junction.
    // This is only necessary temporarily, in order to find a valid previous connection.
    const Connection *junctionConn = *std::find_if(currentSeg->edges.begin(), currentSeg->edges.end(), [&junc](const Connection *c) {
        return c->junction->id == junc.id;
    });

    // Finds a valid previous connection.
    const Connection *prevConn = *std::find_if(currentSeg->edges.begin(), currentSeg->edges.end(), [junctionConn, currentSeg](const Connection *c) {
        return LabyrinthRoute::isNewConnectionValid(*c, *currentSeg, *junctionConn);
    });

    // Resets the navigator's previous connection and current segment, and recreates the route.
    navigator.reset(*prevConn, *currentSeg);
}

void onJunctionDetected(const CarProps& car, uint8_t numInSegments, uint8_t numOutSegments) {

    const radian_t negOri = round90(car.pose.angle + PI);
    const radian_t posOri = round90(car.pose.angle);

    const micro::vec<std::pair<micro::radian_t, uint8_t>, 2> numSegments = {
        { negOri, numInSegments  },
        { posOri, numOutSegments }
    };

    LOG_DEBUG("Junction detected (car pos: (%f, %f), angle: %f deg, segments: (in: %u, out: %u))",
        car.pose.pos.X.get(),
        car.pose.pos.Y.get(),
        static_cast<degree_t>(car.pose.angle).get(),
        static_cast<uint32_t>(numInSegments),
        static_cast<uint32_t>(numOutSegments));

    const LabyrinthGraph::Junctions::const_iterator junc = graph.findJunction(car.pose.pos, numSegments);

    if (graph.junctions.end() != junc) {
        LOG_DEBUG("Junction found: %u (%f, %f), current segment: %c",
            static_cast<uint32_t>(junc->id),
            junc->pos.X.get(),
            junc->pos.Y.get(),
            navigator.currentSegment()->name);

        carPosUpdateQueue.overwrite(junc->pos);

        const Connection *nextConn = navigator.nextConnection();
        if (!nextConn || junc != nextConn->junction) {
            LOG_ERROR("An unknown error has occurred, resets navigator");
            resetNavigator(*junc, negOri);   // an error has occurred, resets navigator
        }

        navigator.onJunctionDetected(car.distance);

    } else {
        LOG_ERROR("Junction not found");
    }
}

void updateCarOrientation(const CarProps& car, const LineInfo& lineInfo) {
    static meter_t lastUpdateDist;

    if (car.orientedDistance > centimeter_t(60)                             &&
        car.distance - lastUpdateDist > centimeter_t(100)                   &&
        LinePattern::SINGLE_LINE == lineInfo.front.pattern.type             &&
        car.distance - lineInfo.front.pattern.startDist > centimeter_t(100) &&
        eqWithOverflow360(car.pose.angle, round90(car.pose.angle), degree_t(10))) {

        carOrientationUpdateQueue.overwrite(round90(car.pose.angle));
        lastUpdateDist = car.orientedDistance;
    }
}

void updateTargetSegment() {
    static LabyrinthGraph::Segments::const_iterator prevSeg = startSeg;

    if (foundSegments.size() == cfg::NUM_LABYRINTH_SEGMENTS) {
        navigator.setTargetSegment(*laneChangeSeg);
    } else {
        state_t<char> segId('\0', millisecond_t(0));
        radioRecvQueue.peek(segId, millisecond_t(0));

        const LabyrinthGraph::Segments::const_iterator targetSeg =
            foundSegments.size() == cfg::NUM_LABYRINTH_SEGMENTS - 1 && getTime() - segId.timestamp() > millisecond_t(1500) ? laneChangeSeg :
            isBtw(segId.value(), 'A', 'Z') ? graph.findSegment(segId.value()) :
            startSeg;

        if (targetSeg != prevSeg) {
            navigator.setTargetSegment(*targetSeg);
            endTime += second_t(10);
            foundSegments.push_back(*prevSeg);
            prevSeg = targetSeg;
        }
    }
}

void setNavigationControl(const Direction targetDir, const CarProps& car, const LineInfo& lineInfo, MainLine& mainLine, ControlData& controlData) {

    const meter_t distSinceLastJunc = car.distance - navigator.lastJunctionDistance();
    const meter_t distUntilNextJunc = navigator.currentSegment()->length - distSinceLastJunc;

    //  starts going forward when in a junction:
    //     if the speed was slow, starts acceleration
    //     if the car was going backwards from a dead-end segment, starts going forward
    if (isJunction(lineInfo.front.pattern)) {
        controlData.speed = LABYRINTH_FORWARD_SPEED;

    // start going backward when a dead-end sign is detected by the front sensor
    } else if (LinePattern::DEAD_END == lineInfo.front.pattern.type) {
        controlData.speed = LABYRINTH_BACKWARD_SPEED;

    // start going forward when a dead-end sign is detected by the rear sensor
    } else if (LinePattern::DEAD_END == lineInfo.rear.pattern.type) {
        controlData.speed = LABYRINTH_FORWARD_SPEED;

    // starts going backward when in a dead-end segment and the gate has been passed (so a new target segment has been received)
    } else if (navigator.currentSegment()->isDeadEnd && navigator.targetSegment() != navigator.currentSegment()) {
        controlData.speed = LABYRINTH_BACKWARD_SPEED;

    // slows down when waiting for the next target segment and getting close to the next junction
    } else if (!navigator.nextConnection() && distUntilNextJunc < centimeter_t(100)) {
        controlData.speed = LABYRINTH_FORWARD_SLOW_SPEED;

    } // else: does not change current speed value

    controlData.rampTime = millisecond_t(700);

    if (distSinceLastJunc < centimeter_t(50) && lineInfo.front.lines.size() && lineInfo.rear.lines.size()) {
        switch (targetDir) {
        case Direction::LEFT:
            mainLine.frontLine = lineInfo.front.lines[0];
            mainLine.rearLine  = *lineInfo.rear.lines.back();
            break;
        case Direction::CENTER:
            mainLine.frontLine = lineInfo.front.lines.size() == 1 ? lineInfo.front.lines[0] : lineInfo.front.lines[1];
            mainLine.rearLine  = lineInfo.rear.lines.size()  == 1 ? lineInfo.rear.lines[0]  : lineInfo.rear.lines[1];
            break;
        case Direction::RIGHT:
            mainLine.frontLine = *lineInfo.front.lines.back();
            mainLine.rearLine  = lineInfo.rear.lines[0];
            break;
        }
    }

    mainLine.updateCenterLine(micro::sgn(car.speed));

    controlData.controlType        = ControlData::controlType_t::Line;
    controlData.lineControl.actual = mainLine.centerLine;
    controlData.lineControl.target = { millimeter_t(0), radian_t(0) };
}

bool navigateLabyrinth(const CarProps& car, const LineInfo& lineInfo, MainLine& mainLine, ControlData& controlData) {

    static LineInfo prevLineInfo;

    updateCarOrientation(car, lineInfo);
    updateTargetSegment();

    if (lineInfo.front.pattern != prevLineInfo.front.pattern) {
        // checks if the car is coming out of a junction
        if (isJunction(lineInfo.front.pattern) && Sign::POSITIVE == lineInfo.front.pattern.dir) {
            onJunctionDetected(car, getNumSegments(prevLineInfo.front.pattern), getNumSegments(lineInfo.front.pattern));
        }

        if (LinePattern::DEAD_END == lineInfo.front.pattern.type || LinePattern::DEAD_END == lineInfo.rear.pattern.type) {
            LOG_ERROR("DEAD_END pattern detected! Something's wrong...");
        }
    }

    const Direction targetDir = navigator.update(car.distance);

    setNavigationControl(targetDir, car, lineInfo, mainLine, controlData);

    prevLineInfo = lineInfo;

    const bool finished = LinePattern::LANE_CHANGE == lineInfo.front.pattern.type && foundSegments.size() == cfg::NUM_LABYRINTH_SEGMENTS;
    return finished;
}

bool shouldHandle(const cfg::ProgramState programState) {
    return isBtw(enum_cast(programState), enum_cast(cfg::ProgramState::NavigateLabyrinth), enum_cast(cfg::ProgramState::LaneChange));
}

} // namespace

extern "C" void runProgLabyrinthTask(void const *argument) {

    SystemManager::instance().registerTask();

    LineInfo lineInfo;
    ControlData controlData;
    MainLine mainLine(cfg::CAR_FRONT_REAR_SENSOR_ROW_DIST);

    controlData.speed    = LABYRINTH_FORWARD_SPEED;
    controlData.rampTime = millisecond_t(1000);

    cfg::ProgramState prevProgramState = cfg::ProgramState::INVALID;

    while (true) {
        const cfg::ProgramState programState = static_cast<cfg::ProgramState>(SystemManager::instance().programState());
        if (shouldHandle(programState)) {

            CarProps car;
            carPropsQueue.peek(car, millisecond_t(0));

            linePatternDomainQueue.overwrite(linePatternDomain_t::Labyrinth);
            lineInfoQueue.peek(lineInfo, millisecond_t(0));
            micro::updateMainLine(lineInfo.front.lines, lineInfo.rear.lines, mainLine, micro::sgn(car.speed));

            switch (programState) {
            case cfg::ProgramState::NavigateLabyrinth:
                if (programState != prevProgramState) {
                    endTime = getTime() + second_t(20);
                    carOrientationUpdateQueue.overwrite(radian_t(0));
                    controlData.speed = LABYRINTH_FORWARD_SPEED;
                    controlData.rampTime = millisecond_t(500);
                }

                if (navigateLabyrinth(car, lineInfo, mainLine, controlData)) {
                    SystemManager::instance().setProgramState(enum_cast(cfg::ProgramState::LaneChange));
                }
                break;

            case cfg::ProgramState::LaneChange:
                if (programState != prevProgramState) {
                    laneChange.initialize(car, lineInfo.front.pattern.dir, safetyCarFollowSpeedSign, LANE_CHANGE_SPEED, LANE_DISTANCE);
                }

                laneChange.update(car, lineInfo, mainLine, controlData);

                if (laneChange.finished()) {
                    SystemManager::instance().setProgramState(enum_cast(cfg::ProgramState::ReachSafetyCar));
                }
                break;
            default:
                break;
            }

            controlQueue.overwrite(controlData);
        }

        prevProgramState = programState;
        SystemManager::instance().notify(true);
        os_sleep(millisecond_t(2));
    }
}
