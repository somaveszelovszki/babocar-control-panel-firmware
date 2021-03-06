#include <micro/debug/params.hpp>
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
extern queue_t<LineDetectControl, 1> lineDetectControlQueue;
extern queue_t<LineInfo, 1> lineInfoQueue;
extern queue_t<ControlData, 1> controlQueue;
extern queue_t<radian_t, 1> carOrientationUpdateQueue;
extern queue_t<char, 1> radioRecvQueue;
extern Sign safetyCarFollowSpeedSign;

namespace {

m_per_sec_t LABYRINTH_SPEED          = m_per_sec_t(1.0f);
m_per_sec_t LABYRINTH_FAST_SPEED     = m_per_sec_t(1.2f);
m_per_sec_t LABYRINTH_DEAD_END_SPEED = m_per_sec_t(0.85f);
m_per_sec_t LANE_CHANGE_SPEED        = m_per_sec_t(0.65f);

constexpr meter_t LANE_DISTANCE = centimeter_t(60);

#if TRACK == RACE_TRACK
#define START_SEGMENT       'U'
#define PREV_SEGMENT        'N'
#define LANE_CHANGE_SEGMENT 'B'
#define LAST_VALID_SEGMENT  'U'
#elif TRACK == TEST_TRACK
#define START_SEGMENT       'W'
#define PREV_SEGMENT        'M'
#define LANE_CHANGE_SEGMENT 'N'
#define LAST_VALID_SEGMENT  'W'
#endif

const LabyrinthGraph graph = buildLabyrinthGraph();
const Segment *startSeg = graph.findSegment(START_SEGMENT);
const Connection *prevConn = graph.findConnection(*graph.findSegment(PREV_SEGMENT), *startSeg);
const Segment *laneChangeSeg = graph.findSegment(LANE_CHANGE_SEGMENT);
LabyrinthNavigator navigator(graph, startSeg, prevConn, laneChangeSeg, LABYRINTH_SPEED, LABYRINTH_FAST_SPEED, LABYRINTH_DEAD_END_SPEED);
vec<const Segment*, cfg::NUM_LABYRINTH_GATE_SEGMENTS> foundSegments;
millisecond_t endTime;

struct JunctionPatternInfo {
    Sign dir            = Sign::NEUTRAL;
    Direction side      = Direction::CENTER;
    uint8_t numSegments = 0;
};

LaneChangeManeuver laneChange;

char nextSegment = START_SEGMENT;

void updateTargetSegment() {
    char segId = '\0';
    radioRecvQueue.peek(segId, millisecond_t(0));
    const bool isLabyrinthFinished = 'X' == segId;

    const Segment *targetSeg =
        isLabyrinthFinished                   ? laneChangeSeg            :
        isBtw(segId, 'A', LAST_VALID_SEGMENT) ? graph.findSegment(segId) :
        startSeg;

    if (targetSeg != navigator.targetSegment() || (isLabyrinthFinished && !navigator.isLastTarget())) {
        foundSegments.push_back(navigator.currentSegment());
        navigator.setTargetSegment(targetSeg, isLabyrinthFinished);
        endTime += second_t(15);
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
    LineDetectControl lineDetectControlData;
    MainLine mainLine(cfg::CAR_FRONT_REAR_SENSOR_ROW_DIST);

    cfg::ProgramState prevProgramState = cfg::ProgramState::INVALID;

    REGISTER_READ_WRITE_PARAM(safetyCarFollowSpeedSign);
    REGISTER_WRITE_ONLY_PARAM(nextSegment);

    while (true) {
        const cfg::ProgramState programState = static_cast<cfg::ProgramState>(SystemManager::instance().programState());
        if (shouldHandle(programState)) {

            CarProps car;
            carPropsQueue.peek(car, millisecond_t(0));

            lineInfoQueue.peek(lineInfo, millisecond_t(0));
            micro::updateMainLine(lineInfo.front.lines, lineInfo.rear.lines, mainLine);

            lineDetectControlData.domain = linePatternDomain_t::Labyrinth;
            lineDetectControlData.isReducedScanRangeEnabled = false;

            switch (programState) {
            case cfg::ProgramState::NavigateLabyrinth:
            {
                if (programState != prevProgramState) {
                    enforceGraphValidity();
                    carOrientationUpdateQueue.overwrite(radian_t(0));
                    endTime = getTime() + second_t(20);
                    navigator.initialize();
                }

                updateTargetSegment();
                navigator.update(car, lineInfo, mainLine, controlData);

                lineDetectControlData.isReducedScanRangeEnabled = false;

                const Pose correctedCarPose = navigator.correctedCarPose();
                if (correctedCarPose.angle != car.pose.angle) {
                    carOrientationUpdateQueue.overwrite(correctedCarPose.angle);
                    LOG_DEBUG("Car orientation updated: %f -> %f [deg]", static_cast<degree_t>(car.pose.angle).get(), static_cast<degree_t>(correctedCarPose.angle).get());
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
                    const LinePattern& pattern = (LinePattern::LANE_CHANGE == lineInfo.front.pattern.type ? lineInfo.front : lineInfo.rear).pattern;
                    laneChange.initialize(car, sgn(car.speed), pattern.dir, pattern.side, safetyCarFollowSpeedSign, LANE_CHANGE_SPEED, LANE_DISTANCE);
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
            lineDetectControlQueue.overwrite(lineDetectControlData);
        }

        prevProgramState = programState;
        SystemManager::instance().notify(true);
        os_sleep(millisecond_t(2));
    }
}
