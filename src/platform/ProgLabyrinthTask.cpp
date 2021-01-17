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
#include <LabyrinthNavigator.hpp>
#include <track.hpp>

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
#define START_SEGMENT       'W'
#define PREV_SEGMENT        'M'
#define LANE_CHANGE_SEGMENT 'N'
#elif LABYRINTH == RACE_LABYRINTH
#define START_SEGMENT       'A'
#define PREV_SEGMENT        ' '
#define LANE_CHANGE_SEGMENT ' '
#endif

const LabyrinthGraph graph = buildLabyrinthGraph();
const Segment *startSeg = graph.findSegment(START_SEGMENT);
const Connection *prevConn = graph.findConnection(*graph.findSegment(PREV_SEGMENT), *startSeg);
const Segment *laneChangeSeg = graph.findSegment(LANE_CHANGE_SEGMENT);
LabyrinthNavigator navigator(graph, startSeg, prevConn, LABYRINTH_FORWARD_SPEED, LABYRINTH_FORWARD_SLOW_SPEED, LABYRINTH_BACKWARD_SPEED);
vec<const Segment*, cfg::NUM_LABYRINTH_GATE_SEGMENTS> foundSegments;
millisecond_t endTime;

struct JunctionPatternInfo {
    Sign dir            = Sign::NEUTRAL;
    Direction side      = Direction::CENTER;
    uint8_t numSegments = 0;
};

LaneChangeManeuver laneChange;

void updateTargetSegment() {
    state_t<char> segId('\0', millisecond_t(0));
    radioRecvQueue.peek(segId, millisecond_t(0));

    const Segment *targetSeg =
        foundSegments.size() == cfg::NUM_LABYRINTH_GATE_SEGMENTS - 1 && getTime() - segId.timestamp() > millisecond_t(1500) ? laneChangeSeg :
        isBtw(segId.value(), 'A', 'Z') ? graph.findSegment(segId.value()) :
        startSeg;

    if (targetSeg != navigator.targetSegment()) {
        foundSegments.push_back(navigator.currentSegment());
        navigator.setTargetSegment(targetSeg, foundSegments.size() == cfg::NUM_LABYRINTH_GATE_SEGMENTS);
        endTime += second_t(10);
    }
}

bool shouldHandle(const cfg::ProgramState programState) {
    return isBtw(enum_cast(programState), enum_cast(cfg::ProgramState::NavigateLabyrinth), enum_cast(cfg::ProgramState::LaneChange));
}

void enforceGraphValidity() {
    if (!graph.valid()) {
        while (true) {
            LOG_ERROR("Labyrinth graph is invalid!");
            SystemManager::instance().notify(false);
            os_sleep(millisecond_t(100));
        }
    }
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

    enforceGraphValidity();

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
            {
                if (programState != prevProgramState) {
                    carOrientationUpdateQueue.overwrite(radian_t(0));
                    controlData.speed = LABYRINTH_FORWARD_SPEED;
                    controlData.rampTime = millisecond_t(500);
                    endTime = getTime() + second_t(20);

                    navigator.initialize();
                }

                updateTargetSegment();
                navigator.update(car, lineInfo, mainLine, controlData);

                const Pose correctedCarPose = navigator.correctedCarPose();
                if (correctedCarPose.angle != car.pose.angle) {
                    carOrientationUpdateQueue.overwrite(correctedCarPose.angle);
                }
                if (correctedCarPose.pos != car.pose.pos) {
                    carPosUpdateQueue.overwrite(correctedCarPose.pos);
                }

                if (navigator.finished()) {
                    SystemManager::instance().setProgramState(enum_cast(cfg::ProgramState::LaneChange));
                }
                break;
            }

            case cfg::ProgramState::LaneChange:
                if (programState != prevProgramState) {
                    laneChange.initialize(car, lineInfo.front.pattern.dir, lineInfo.front.pattern.side, safetyCarFollowSpeedSign, LANE_CHANGE_SPEED, LANE_DISTANCE);
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
