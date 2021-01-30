#include <cfg_board.hpp>
#include <micro/container/infinite_buffer.hpp>
#include <micro/debug/params.hpp>
#include <micro/debug/SystemManager.hpp>
#include <micro/math/numeric.hpp>
#include <micro/sensor/Filter.hpp>
#include <micro/port/queue.hpp>
#include <micro/port/task.hpp>
#include <micro/utils/CarProps.hpp>
#include <micro/utils/ControlData.hpp>
#include <micro/utils/LinePattern.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/state.hpp>
#include <micro/utils/timer.hpp>
#include <micro/utils/trajectory.hpp>

#include <cfg_car.hpp>
#include <cfg_track.hpp>
#include <Distances.hpp>
#include <track.hpp>
#include <OvertakeManeuver.hpp>
#include <RaceTrackInfo.hpp>
#include <TestManeuver.hpp>
#include <TurnAroundManeuver.hpp>

using namespace micro;

extern queue_t<CarProps, 1> carPropsQueue;
extern queue_t<linePatternDomain_t, 1> linePatternDomainQueue;
extern queue_t<LineInfo, 1> lineInfoQueue;
extern queue_t<ControlData, 1> controlQueue;
extern queue_t<Distances, 1> distancesQueue;

Sign safetyCarFollowSpeedSign = Sign::POSITIVE;

namespace {

constexpr centimeter_t MAX_VALID_SAFETY_CAR_DISTANCE = centimeter_t(120);

m_per_sec_t SAFETY_CAR_SLOW_MAX_SPEED     = m_per_sec_t(1.3f);
m_per_sec_t SAFETY_CAR_FAST_MAX_SPEED     = m_per_sec_t(1.7f);
m_per_sec_t REACH_SAFETY_CAR_SPEED        = m_per_sec_t(0.6f);
m_per_sec_t OVERTAKE_BEGIN_SPEED          = m_per_sec_t(1.0f);
m_per_sec_t OVERTAKE_STRAIGHT_START_SPEED = m_per_sec_t(1.5f);
m_per_sec_t OVERTAKE_STRAIGHT_SPEED       = m_per_sec_t(3.0f);
m_per_sec_t OVERTAKE_END_SPEED            = m_per_sec_t(1.8f);
m_per_sec_t TURN_AROUND_SPEED             = m_per_sec_t(1.2f);

meter_t OVERTAKE_SECTION_LENGTH           = centimeter_t(800);
meter_t OVERTAKE_PREPARE_DISTANCE         = centimeter_t(70);
meter_t OVERTAKE_BEGIN_SINE_ARC_LENGTH    = centimeter_t(180);
meter_t OVERTAKE_END_SINE_ARC_LENGTH      = centimeter_t(180);
meter_t OVERTAKE_SIDE_DISTANCE            = centimeter_t(50);

meter_t TURN_AROUND_RADIUS                = centimeter_t(40);
meter_t TURN_AROUND_SINE_ARC_LENGTH       = centimeter_t(60);

RaceTrackInfo trackInfo(trackSegments);
OvertakeManeuver overtake;
TurnAroundManeuver turnAround;
TestManeuver testManeuver;

m_per_sec_t safetyCarFollowSpeed(meter_t distFromSafetyCar, const Sign targetSpeedSign, bool isFast) {
    return targetSpeedSign * map(distFromSafetyCar, meter_t(0.3f), meter_t(0.8f), m_per_sec_t(0), isFast ? SAFETY_CAR_FAST_MAX_SPEED : SAFETY_CAR_SLOW_MAX_SPEED);
}

TrackSegments::const_iterator getFastSegment(const RaceTrackInfo& trackInfo, const uint32_t fastSeg) {
    TrackSegments::const_iterator it = trackInfo.segments.begin();
    uint32_t numFastSegs = 0;
    for (; it != trackInfo.segments.end(); ++it) {
        if (it->isFast && ++numFastSegs == fastSeg) {
            break;
        }
    }
    return it;
}

TrackSegments::const_iterator getFastSegment(const RaceTrackInfo& trackInfo, const cfg::ProgramState programState) {
    return getFastSegment(trackInfo.segments,
        cfg::ProgramState::Race          == programState ? 1 :
        cfg::ProgramState::Race_segFast2 == programState ? 2 :
        cfg::ProgramState::Race_segFast3 == programState ? 3 :
        cfg::ProgramState::Race_segFast4 == programState ? 4 : micro::numeric_limits<uint32_t>::max()
    );
}

bool shouldHandle(const cfg::ProgramState programState) {
    return isBtw(enum_cast(programState), enum_cast(cfg::ProgramState::ReachSafetyCar), enum_cast(cfg::ProgramState::Test));
}

ControlData getControl(const micro::CarProps& car, const RaceTrackInfo& trackInfo, const micro::MainLine& mainLine, const Sign targetSpeedSign)
{
    ControlData controlData = trackInfo.seg->getControl(car, trackInfo, mainLine);
    controlData.speed = targetSpeedSign * controlData.speed;
    return controlData;
}

} // namespace

extern "C" void runProgRaceTrackTask(void) {

    SystemManager::instance().registerTask();

    CarProps car;
    LineInfo lineInfo;
    ControlData controlData;
    Distances distances;

    MainLine mainLine(cfg::CAR_FRONT_REAR_SENSOR_ROW_DIST);

    const TrackSegments::const_iterator overtakeSeg = getFastSegment(trackInfo.segments, 3);

    meter_t lastDistWithValidLine;
    meter_t lastDistWithSafetyCar;

    cfg::ProgramState prevProgramState = cfg::ProgramState::INVALID;

    Sign targetSpeedSign;

    uint8_t lastOvertakeLap = 0;

    while (true) {
        const cfg::ProgramState programState = static_cast<cfg::ProgramState>(SystemManager::instance().programState());
        if (shouldHandle(programState)) {

            carPropsQueue.peek(car, millisecond_t(0));
            lineInfoQueue.peek(lineInfo, millisecond_t(0));
            distancesQueue.peek(distances, millisecond_t(0));

            // runs for the first time that this task handles the program state
            if (!shouldHandle(prevProgramState)) {
                if ((trackInfo.seg = getFastSegment(trackInfo.segments, programState)) != trackInfo.segments.end()) { // race
                    trackInfo.lap = 3;
                    targetSpeedSign = Sign::POSITIVE;
                    SystemManager::instance().setProgramState(enum_cast(cfg::ProgramState::Race));
                } else { // reach safety car
                    trackInfo.lap = 1;
                    trackInfo.seg = trackInfo.segments.begin();
                    targetSpeedSign = safetyCarFollowSpeedSign;
                }

                lastDistWithValidLine = car.distance;
                lastDistWithSafetyCar = car.distance;

                trackInfo.lapStartTime = getTime();
                trackInfo.segStartCarProps = car;
                trackInfo.segStartLine = mainLine.centerLine;
            }

            micro::updateMainLine(lineInfo.front.lines, lineInfo.rear.lines, mainLine);

            // when in turn-around state, does not update track info to prevent logic errors caused by possible line pattern detections
            if (cfg::ProgramState::TurnAround != programState) {
                trackInfo.update(car, lineInfo, mainLine, controlData);
            }

            // sets default lateral control
            controlData.rearSteerEnabled    = true;
            controlData.lineControl.actual  = mainLine.centerLine;
            controlData.lineControl.target  = { millimeter_t(0), radian_t(0) };

            const meter_t distFromSafetyCar = Sign::POSITIVE == targetSpeedSign ? distances.front : distances.rear;
            if (distFromSafetyCar < centimeter_t(100)) {
                lastDistWithSafetyCar = car.distance;
            }

            if (LinePattern::NONE != lineInfo.front.pattern.type || LinePattern::NONE != lineInfo.rear.pattern.type) {
                lastDistWithValidLine = car.distance;
            }

            switch (programState) {
            case cfg::ProgramState::ReachSafetyCar:
                controlData.speed = targetSpeedSign * REACH_SAFETY_CAR_SPEED;
                controlData.rampTime = millisecond_t(500);
                if (distFromSafetyCar != centimeter_t(0) && abs(safetyCarFollowSpeed(distFromSafetyCar, targetSpeedSign, trackInfo.seg->isFast)) < abs(controlData.speed)) {
                    SystemManager::instance().setProgramState(enum_cast(cfg::ProgramState::FollowSafetyCar));
                    LOG_DEBUG("Reached safety car, starts following");
                }
                break;

            case cfg::ProgramState::FollowSafetyCar:
                controlData.speed = safetyCarFollowSpeed(distFromSafetyCar, targetSpeedSign, trackInfo.seg->isFast);
                controlData.rampTime = millisecond_t(0);

                if (overtakeSeg == trackInfo.seg && (1 == trackInfo.lap || 3 == trackInfo.lap) && trackInfo.lap != lastOvertakeLap) {
                    SystemManager::instance().setProgramState(enum_cast(cfg::ProgramState::OvertakeSafetyCar));
                    LOG_DEBUG("Starts overtake");
                } else if (car.distance - lastDistWithSafetyCar > MAX_VALID_SAFETY_CAR_DISTANCE && trackInfo.seg->isFast) {
                    SystemManager::instance().setProgramState(enum_cast(cfg::ProgramState::Race));
                    LOG_DEBUG("Safety car left the track, starts race");
                }
                break;

            case cfg::ProgramState::OvertakeSafetyCar:
                if (programState != prevProgramState) {
                    lastOvertakeLap = trackInfo.lap;
                    overtake.initialize(car, targetSpeedSign,
                        OVERTAKE_BEGIN_SPEED, OVERTAKE_STRAIGHT_START_SPEED, OVERTAKE_STRAIGHT_SPEED, OVERTAKE_END_SPEED,
                        OVERTAKE_SECTION_LENGTH, OVERTAKE_PREPARE_DISTANCE, OVERTAKE_BEGIN_SINE_ARC_LENGTH, OVERTAKE_END_SINE_ARC_LENGTH,
                        OVERTAKE_SIDE_DISTANCE);
                }

                controlData.speed = safetyCarFollowSpeed(distFromSafetyCar, targetSpeedSign, trackInfo.seg->isFast);
                overtake.update(car, lineInfo, mainLine, controlData);

                if (overtake.finished()) {
                    SystemManager::instance().setProgramState(enum_cast(cfg::ProgramState::Race));
                    LOG_DEBUG("Overtake finished, starts race");
                }
                break;

            case cfg::ProgramState::Race:
                controlData = getControl(car, trackInfo, mainLine, targetSpeedSign);

                // the first 3 laps are dedicated to the safety-car follow task,
                // changing the line offset and angle might ruin the car's capability to detect the safety car
                if (trackInfo.lap <= 3) {
                    controlData.lineControl.target = { millimeter_t(0), radian_t(0) };
                }

                if (trackInfo.lap > cfg::NUM_RACE_LAPS) {
                    SystemManager::instance().setProgramState(enum_cast(cfg::ProgramState::Finish));
                    LOG_DEBUG("Race finished");

                } else if (trackInfo.lap <= 3 && distFromSafetyCar < (trackInfo.seg->isFast ? MAX_VALID_SAFETY_CAR_DISTANCE - centimeter_t(2) : centimeter_t(80))) {
                    SystemManager::instance().setProgramState(enum_cast(cfg::ProgramState::FollowSafetyCar));
                    LOG_DEBUG("Reached safety car, starts following");

                } else if (Sign::NEGATIVE == targetSpeedSign                      &&
                           trackInfo.lap == 2                                     &&
                           trackInfo.seg == getFastSegment(trackInfo.segments, 1) &&
                           car.distance - trackInfo.segStartCarProps.distance > meter_t(4)) {

                    SystemManager::instance().setProgramState(enum_cast(cfg::ProgramState::TurnAround));
                    LOG_DEBUG("Starts turn-around.");
                }

                if (car.distance - lastDistWithValidLine > meter_t(2)) {
                    SystemManager::instance().setProgramState(enum_cast(cfg::ProgramState::Error));
                    LOG_ERROR("An error has occurred. Car stopped.");
                }
                break;

            case cfg::ProgramState::TurnAround:
                if (programState != prevProgramState) {
                    turnAround.initialize(car, -targetSpeedSign, TURN_AROUND_SPEED, TURN_AROUND_SINE_ARC_LENGTH, TURN_AROUND_RADIUS);
                }

                turnAround.update(car, lineInfo, mainLine, controlData);

                if (turnAround.finished()) {
                    targetSpeedSign = -targetSpeedSign;
                    SystemManager::instance().setProgramState(enum_cast(cfg::ProgramState::Race));
                }
                break;

            case cfg::ProgramState::Finish:
                if (car.distance - trackInfo.segStartCarProps.distance < meter_t(2.0f)) {
                    controlData = getControl(car, trackInfo, mainLine, targetSpeedSign);
                } else {
                    controlData.speed = m_per_sec_t(0);
                    controlData.rampTime = millisecond_t(1000);
                }
                break;

            case cfg::ProgramState::Error:
                controlData.speed = m_per_sec_t(0);
                controlData.rampTime = millisecond_t(1000);
                break;

            case cfg::ProgramState::Test:
                controlData.speed = m_per_sec_t(1.0f);
                controlData.rampTime = millisecond_t(500);
                break;

            default:
                LOG_ERROR("Invalid program state counter: [%u]", enum_cast(programState));
                break;
            }

            controlQueue.overwrite(controlData);
            linePatternDomainQueue.overwrite(linePatternDomain_t::Race);
        }

        prevProgramState = programState;
        SystemManager::instance().notify(true);
        os_sleep(millisecond_t(1));
    }
}
