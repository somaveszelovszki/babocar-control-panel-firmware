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

#define RANDOM_SEGMENT true

extern queue_t<CarProps, 1> carPropsQueue;
extern queue_t<point2m, 1> carPosUpdateQueue;
extern queue_t<radian_t, 1> carOrientationUpdateQueue;
extern queue_t<linePatternDomain_t, 1> linePatternDomainQueue;
extern queue_t<LineInfo, 1> lineInfoQueue;
extern queue_t<ControlData, 1> controlQueue;
extern queue_t<radian_t, 1> carOrientationUpdateQueue;
extern queue_t<char, 1> radioRecvQueue;

extern Sign safetyCarFollowSpeedSign;

namespace {

m_per_sec_t LABYRINTH_FORWARD_SPEED      = m_per_sec_t(1.2f);
m_per_sec_t LABYRINTH_FORWARD_SLOW_SPEED = m_per_sec_t(0.9f);
m_per_sec_t LABYRINTH_BACKWARD_SPEED     = m_per_sec_t(-0.8f);
m_per_sec_t LANE_CHANGE_SPEED            = m_per_sec_t(0.8f);

constexpr meter_t LANE_DISTANCE = centimeter_t(60);

const LabyrinthGraph graph = buildLabyrinthGraph();
const LabyrinthGraph::Segments::const_iterator startSeg = graph.findSegment('A');
LabyrinthNavigator navigator(graph, *graph.findFirstConnection(*startSeg, *graph.findSegment('X')), *startSeg);

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
    const Segment *currentSeg      = junc.getSideSegments(negOri)->second.begin()->second;
    const Connection *junctionConn = *std::find_if(currentSeg->edges.begin(), currentSeg->edges.end(), [&junc](const Connection *c) {
        return c->junction->id == junc.id;
    });

    const Connection *prevConn = *std::find_if(currentSeg->edges.begin(), currentSeg->edges.end(), [junctionConn, currentSeg](const Connection *c) {
        return LabyrinthRoute::isNewConnectionValid(*junctionConn, *currentSeg, *c);
    });

    navigator.reset(*prevConn, *currentSeg);
}

void onJunctionDetected(const CarProps& car, uint8_t numInSegments, uint8_t numOutSegments) {

    const radian_t negOri = round90(car.pose.angle + PI);
    const radian_t posOri = round90(car.pose.angle);

    const micro::vec<std::pair<micro::radian_t, uint8_t>, 2> numSegments = {
        { negOri, numInSegments  },
        { posOri, numOutSegments }
    };

    LabyrinthGraph::Junctions::const_iterator junc = graph.findJunction(car.pose.pos, numSegments);

    if (graph.junctions.end() != junc) {
        LOG_DEBUG("currentSeg: %c, junction: %u (%f, %f)", navigator.currentSegment()->name, static_cast<uint32_t>(junc->id), junc->pos.X.get(), junc->pos.Y.get());

        carPosUpdateQueue.overwrite(junc->pos);

        const Connection *nextConn = navigator.nextConnection();
        if (!nextConn || junc != nextConn->junction) {
            resetNavigator(*junc, negOri);   // an error has occurred, resets navigator
        }

        navigator.onJunctionDetected(car.distance);

    } else {
        LOG_ERROR("Junction not found");
    }
}

void updateCarOrientation(const CarProps& car, const LineInfo& lineInfo) {
    static meter_t lastUpdatedOrientedDistance;

    if (car.orientedDistance - lastUpdatedOrientedDistance > centimeter_t(100)   &&
        LinePattern::SINGLE_LINE == lineInfo.front.pattern.type             &&
        car.distance - lineInfo.front.pattern.startDist > centimeter_t(100) &&
        eqWithOverflow360(car.pose.angle, round90(car.pose.angle), degree_t(10))) {

        carOrientationUpdateQueue.overwrite(round90(car.pose.angle));
        lastUpdatedOrientedDistance = car.orientedDistance;
    }
}

LabyrinthGraph::Segments::const_iterator getTargetSegment(bool labyrinthDiscovered) {
    LabyrinthGraph::Segments::const_iterator seg = graph.segments.end();
    if (labyrinthDiscovered) {
        seg = laneChangeSeg;
    } else {
        static char prevSegId = '\0';
        char segId = prevSegId;
        radioRecvQueue.peek(segId, millisecond_t(0));
        seg = segId != prevSegId ? graph.findSegment(segId) : navigator.targetSegment();
        prevSegId = segId;
    }
    return seg;
}

void setNavigationControl(const Direction targetDir, const CarProps& car, const LineInfo& lineInfo, MainLine& mainLine, ControlData& controlData) {

    const meter_t distSinceLastJunc = car.distance - navigator.lastJunctionDistance();
    const meter_t distUntilNextJunc = navigator.currentSegment()->length - distSinceLastJunc;

    if (isJunction(lineInfo.front.pattern)) {
        controlData.speed = LABYRINTH_FORWARD_SPEED;

    } else if (LinePattern::DEAD_END == lineInfo.front.pattern.type) {
        controlData.speed = LABYRINTH_BACKWARD_SPEED;

    } else if (LinePattern::DEAD_END == lineInfo.rear.pattern.type) {
        controlData.speed = LABYRINTH_FORWARD_SPEED;

    } else if (navigator.currentSegment()->isDeadEnd && navigator.targetSegment() != navigator.currentSegment()) {
        controlData.speed = LABYRINTH_BACKWARD_SPEED;

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
    static uint8_t numFoundSegments = 0;
    static millisecond_t endTime;

    static const bool runOnce = [&controlData]() {
        endTime = getTime() + millisecond_t(20);
        carOrientationUpdateQueue.overwrite(radian_t(0));
        controlData.speed = LABYRINTH_FORWARD_SPEED;
        controlData.rampTime = millisecond_t(500);
        return true;
    }();
    UNUSED(runOnce);

    const bool labyrinthDiscovered = numFoundSegments == cfg::NUM_LABYRINTH_SEGMENTS;

    updateCarOrientation(car, lineInfo);

    const LabyrinthGraph::Segments::const_iterator targetSeg = getTargetSegment(labyrinthDiscovered);
    if (targetSeg != navigator.targetSegment()) {
        navigator.setTargetSegment(*targetSeg);
        endTime += second_t(10);
        ++numFoundSegments;
    }

    if (lineInfo.front.pattern != prevLineInfo.front.pattern) {
        // checks if the car is coming out of a junction
        if (isJunction(lineInfo.front.pattern) && Sign::POSITIVE == lineInfo.front.pattern.dir) {
            onJunctionDetected(car, getNumSegments(prevLineInfo.front.pattern), getNumSegments(lineInfo.front.pattern));
        }

        if (LinePattern::DEAD_END == lineInfo.front.pattern.type || LinePattern::DEAD_END == lineInfo.rear.pattern.type) {
            LOG_ERROR("DEAD_END pattern detected! Something's wrong...");
        }

        prevLineInfo = lineInfo;
    }

    const Direction targetDir = navigator.update(car.distance);

    setNavigationControl(targetDir, car, lineInfo, mainLine, controlData);

    const bool finished = LinePattern::LANE_CHANGE == lineInfo.front.pattern.type && labyrinthDiscovered;
    return finished;
}

} // namespace

extern "C" void runProgLabyrinthTask(void const *argument) {

    SystemManager::instance().registerTask();

    LineInfo lineInfo;
    ControlData controlData;
    MainLine mainLine(cfg::CAR_FRONT_REAR_SENSOR_ROW_DIST);

    controlData.speed    = LABYRINTH_FORWARD_SPEED;
    controlData.rampTime = millisecond_t(1000);

    while (true) {
        const cfg::ProgramState programState = static_cast<cfg::ProgramState>(SystemManager::instance().programState());
        if (isBtw(enum_cast(programState), enum_cast(cfg::ProgramState::NavigateLabyrinth), enum_cast(cfg::ProgramState::LaneChange))) {

            CarProps car;
            carPropsQueue.peek(car, millisecond_t(0));

            linePatternDomainQueue.overwrite(linePatternDomain_t::Labyrinth);
            lineInfoQueue.peek(lineInfo, millisecond_t(0));
            micro::updateMainLine(lineInfo.front.lines, lineInfo.rear.lines, mainLine, micro::sgn(car.speed));

            switch (programState) {
            case cfg::ProgramState::NavigateLabyrinth:
                if (navigateLabyrinth(car, lineInfo, mainLine, controlData)) {
                    SystemManager::instance().setProgramState(enum_cast(cfg::ProgramState::LaneChange));
                }
                break;

            case cfg::ProgramState::LaneChange:
                laneChange = LaneChangeManeuver(car, lineInfo.front.pattern.dir, safetyCarFollowSpeedSign, LANE_CHANGE_SPEED, LANE_DISTANCE);
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

        SystemManager::instance().notify(true);
        os_sleep(millisecond_t(2));
    }
}
