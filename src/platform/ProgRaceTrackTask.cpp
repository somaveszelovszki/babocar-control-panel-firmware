#include <cfg_board.hpp>
#include <micro/container/infinite_buffer.hpp>
#include <micro/debug/params.hpp>
#include <micro/debug/SystemManager.hpp>
#include <micro/hw/SteeringServo.hpp>
#include <micro/math/numeric.hpp>
#include <micro/sensor/Filter.hpp>
#include <micro/port/queue.hpp>
#include <micro/port/task.hpp>
#include <micro/utils/CarProps.hpp>
#include <micro/utils/ControlData.hpp>
#include <micro/utils/LinePattern.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/timer.hpp>
#include <micro/utils/trajectory.hpp>

#include <cfg_car.hpp>
#include <cfg_track.hpp>
#include <DetectedLines.hpp>
#include <Distances.hpp>
#include <track.hpp>

using namespace micro;

extern queue_t<CarProps, 1> carPropsQueue;
extern queue_t<linePatternDomain_t, 1> linePatternDomainQueue;
extern queue_t<DetectedLines, 1> detectedLinesQueue;
extern queue_t<ControlData, 1> controlQueue;
extern queue_t<Distances, 1> distancesQueue;

Sign safetyCarFollowSpeedSign = Sign::POSITIVE;

namespace {

constexpr meter_t PREV_CAR_PROPS_RESOLUTION = centimeter_t(5);

m_per_sec_t maxSpeed_SAFETY_CAR_SLOW = m_per_sec_t(1.3f);
m_per_sec_t maxSpeed_SAFETY_CAR_FAST = m_per_sec_t(1.8f);
m_per_sec_t speed_REACH_SAFETY_CAR   = m_per_sec_t(0.8f);
m_per_sec_t speed_OVERTAKE_BEGIN     = m_per_sec_t(2.6f);
m_per_sec_t speed_OVERTAKE_STRAIGHT  = m_per_sec_t(3.5f);
m_per_sec_t speed_OVERTAKE_END       = m_per_sec_t(1.8f);
meter_t     dist_OVERTAKE_SIDE       = centimeter_t(60);

struct {
    TrackSegments::const_iterator segment;
    meter_t startDist;
    radian_t orientation;
    Trajectory trajectory;
    uint8_t cntr = 0;
} overtake;

enum class turnaroundState_t : uint8_t {
    Inactive,
    Stop,
    FollowTrajectory,
    ReachBrakePattern
};

struct {
    turnaroundState_t state = turnaroundState_t::Inactive;
    Trajectory trajectory;
} turnaround;

infinite_buffer<CarProps, static_cast<uint32_t>(meter_t(5) / PREV_CAR_PROPS_RESOLUTION)> prevCarProps;

TrackSegments::const_iterator nextSegment(const TrackSegments::const_iterator currentSeg) {
    return trackSegments.back() == currentSeg ? trackSegments.begin() : currentSeg + 1;
}

m_per_sec_t safetyCarFollowSpeed(meter_t distFromSafetyCar, const Sign speedSign, bool isFast) {
    return speedSign * map(distFromSafetyCar, meter_t(0.3f), meter_t(0.8f), m_per_sec_t(0), isFast ? maxSpeed_SAFETY_CAR_FAST : maxSpeed_SAFETY_CAR_SLOW);
}

bool overtakeSafetyCar(const CarProps& car, const DetectedLines& detectedLines, ControlData& controlData) {

    static constexpr meter_t OVERTAKE_SECTION_LENGTH = centimeter_t(700);
    static constexpr meter_t ORIENTATION_FILTER_DIST = centimeter_t(50);
    static constexpr meter_t BEGIN_SINE_ARC_LENGTH   = centimeter_t(180);
    static constexpr meter_t END_SINE_ARC_LENGTH     = centimeter_t(150);
    static constexpr meter_t FAST_SECTION_MAX_LENGTH = OVERTAKE_SECTION_LENGTH - ORIENTATION_FILTER_DIST - BEGIN_SINE_ARC_LENGTH - END_SINE_ARC_LENGTH;

    bool finished = false;

    if (overtake.trajectory.length() == meter_t(0)) {

        if (overtake.startDist == meter_t(0)) {
            overtake.startDist = car.distance;
        }

        const meter_t overtakeDist = car.distance - overtake.startDist;
        const Sign targetSpeedSign = sgn(car.speed);

        if (overtakeDist > ORIENTATION_FILTER_DIST) {

            const point2m posDiff           = car.pose.pos - prevCarProps.peek_back(static_cast<uint32_t>(overtakeDist / PREV_CAR_PROPS_RESOLUTION)).pose.pos;
            const meter_t fastSectionLength = FAST_SECTION_MAX_LENGTH - overtakeDist;
            overtake.orientation            = posDiff.getAngle();

            overtake.trajectory.setStartConfig(Trajectory::config_t{
                Pose{ car.pose.pos, overtake.orientation },
                targetSpeedSign * clamp(abs(car.speed), m_per_sec_t(1.0f), speed_OVERTAKE_BEGIN)
            }, car.distance);

            overtake.trajectory.appendSineArc(Trajectory::config_t{
                Pose{
                    overtake.trajectory.lastConfig().pose.pos + vec2m{ BEGIN_SINE_ARC_LENGTH, dist_OVERTAKE_SIDE }.rotate(overtake.orientation),
                    overtake.orientation
                },
                targetSpeedSign * speed_OVERTAKE_BEGIN
            }, car.pose.angle, Trajectory::orientationUpdate_t::FIX_ORIENTATION, 50, radian_t(0), PI);

            overtake.trajectory.appendLine(Trajectory::config_t{
                Pose{
                    overtake.trajectory.lastConfig().pose.pos + vec2m{ fastSectionLength, centimeter_t(0) }.rotate(overtake.orientation),
                    overtake.orientation
                },
                targetSpeedSign * speed_OVERTAKE_STRAIGHT
            });

            overtake.trajectory.appendSineArc(Trajectory::config_t{
                Pose{
                    overtake.trajectory.lastConfig().pose.pos + vec2m{ END_SINE_ARC_LENGTH, -dist_OVERTAKE_SIDE }.rotate(overtake.orientation),
                    overtake.orientation
                },
                targetSpeedSign * speed_OVERTAKE_END
            }, car.pose.angle, Trajectory::orientationUpdate_t::FIX_ORIENTATION, 50, radian_t(0), PI_2);
        }
    }

    if (overtake.trajectory.length() > meter_t(0)) {
        controlData = overtake.trajectory.update(car);

        finished = overtake.trajectory.length() - overtake.trajectory.coveredDistance() < centimeter_t(30) && LinePattern::NONE != detectedLines.front.pattern.type;
        if (finished) {
            overtake.trajectory.clear();
            overtake.startDist = meter_t(0);
        }
    }

    return finished;
}

bool turnAround(const CarProps& car, const DetectedLines& detectedLines, ControlData& controlData) {

    static constexpr meter_t TURN_AROUND_RADIUS = centimeter_t(35);
    static constexpr meter_t SINE_ARC_LENGTH    = centimeter_t(100);

    static constexpr m_per_sec_t speed_TURN_AROUND         = m_per_sec_t(0.6f);
    static constexpr m_per_sec_t speed_REACH_BRAKE_PATTERN = m_per_sec_t(1.8f);

    switch (turnaround.state) {
    case turnaroundState_t::Inactive:
        turnaround.state = turnaroundState_t::Stop;
        break;

    case turnaroundState_t::Stop:
        controlData.speed    = m_per_sec_t(0);
        controlData.rampTime = second_t(1);

        if (abs(car.speed) < m_per_sec_t(0.05f)) {
            turnaround.state = turnaroundState_t::FollowTrajectory;
        }
        break;

    case turnaroundState_t::FollowTrajectory:
        if (turnaround.trajectory.length() == meter_t(0)) {

            const Sign targetSpeedSign = -sgn(car.speed); // changes speed sign in order to turn around

            turnaround.trajectory.setStartConfig(Trajectory::config_t{
                car.pose,
                targetSpeedSign * speed_TURN_AROUND
            }, car.distance);

            turnaround.trajectory.appendSineArc(Trajectory::config_t{
                Pose{
                    overtake.trajectory.lastConfig().pose.pos + vec2m{ SINE_ARC_LENGTH, -TURN_AROUND_RADIUS }.rotate(car.pose.angle),
                    car.pose.angle
                },
                targetSpeedSign * speed_TURN_AROUND
            }, car.pose.angle, Trajectory::orientationUpdate_t::FIX_ORIENTATION, 30, radian_t(0), PI);

            turnaround.trajectory.appendCircle(
                turnaround.trajectory.lastConfig().pose.pos + vec2m{ centimeter_t(0), TURN_AROUND_RADIUS }.rotate(car.pose.angle),
                PI,
                targetSpeedSign * speed_TURN_AROUND, 30);

            turnaround.trajectory.appendSineArc(Trajectory::config_t{
                Pose{
                    overtake.trajectory.lastConfig().pose.pos + vec2m{ SINE_ARC_LENGTH, TURN_AROUND_RADIUS }.rotate(car.pose.angle + PI),
                    car.pose.angle
                },
                targetSpeedSign * speed_TURN_AROUND
            }, car.pose.angle, Trajectory::orientationUpdate_t::FIX_ORIENTATION, 30, radian_t(0), PI);
        }

        controlData = turnaround.trajectory.update(car);

        if (turnaround.trajectory.length() - turnaround.trajectory.coveredDistance() < centimeter_t(30) && LinePattern::NONE != detectedLines.front.pattern.type) {
            turnaround.trajectory.clear();
            turnaround.state = turnaroundState_t::ReachBrakePattern;
        }
        break;

    case turnaroundState_t::ReachBrakePattern:
        controlData.speed = sgn(car.speed) * speed_REACH_BRAKE_PATTERN;
        controlData.rampTime = millisecond_t(500);

        if (LinePattern::BRAKE == detectedLines.front.pattern.type) {
            turnaround.state = turnaroundState_t::Inactive;
        }
        break;
    }

    return turnaroundState_t::Inactive == turnaround.state;
}

TrackSegments::const_iterator getFastSegment(const TrackSegments& trackSegments, const uint32_t fastSeg) {
    TrackSegments::const_iterator it = trackSegments.begin();
    uint32_t numFastSegs = 0;
    for (; it != trackSegments.end(); ++it) {
        if (it->isFast && ++numFastSegs == fastSeg) {
            break;
        }
    }
    return it;
}

TrackSegments::const_iterator getFastSegment(const TrackSegments& trackSegments, const cfg::ProgramState programState) {
    return getFastSegment(trackSegments,
        cfg::ProgramState::Race          == programState ? 1 :
        cfg::ProgramState::Race_segFast2 == programState ? 2 :
        cfg::ProgramState::Race_segFast3 == programState ? 3 :
        cfg::ProgramState::Race_segFast4 == programState ? 4 : micro::numeric_limits<uint32_t>::max()
    );
}

} // namespace

extern "C" void runProgRaceTrackTask(void) {

    SystemManager::instance().registerTask();

    DetectedLines prevDetectedLines, detectedLines;
    ControlData controlData;
    Distances distances;

    MainLine mainLine(cfg::CAR_FRONT_REAR_SENSOR_ROW_DIST);

    TrackInfo trackInfo;

    overtake.segment = getFastSegment(trackSegments, 3);

    meter_t lastDistWithValidLine;
    meter_t lastDistWithSafetyCar;
    millisecond_t lapStartTime;

    REGISTER_READ_WRITE_PARAM(safetyCarFollowSpeedSign);

    while (true) {
        const cfg::ProgramState programState = static_cast<cfg::ProgramState>(SystemManager::instance().programState());
        if (isBtw(enum_cast(programState), enum_cast(cfg::ProgramState::ReachSafetyCar), enum_cast(cfg::ProgramState::Error))) {

            CarProps car;
            carPropsQueue.peek(car, millisecond_t(0));

            const Sign speedSign = sgn(car.speed);

            static const bool runOnce = [&car, &trackInfo, &lastDistWithValidLine, &lastDistWithSafetyCar, &lapStartTime, &mainLine, &programState]() {

                if ((trackInfo.seg = getFastSegment(trackSegments, programState)) != trackSegments.end()) { // race
                    trackInfo.lap = 3;
                    SystemManager::instance().setProgramState(enum_cast(cfg::ProgramState::Race));
                } else { // reach safety car
                    trackInfo.lap = 1;
                    trackInfo.seg = trackSegments.begin();
                }

                lastDistWithValidLine = car.distance;
                lastDistWithSafetyCar = car.distance;
                lapStartTime = getTime();

                trackInfo.segStartCarProps = car;
                trackInfo.segStartLine = mainLine.centerLine;
                prevCarProps.push_back(trackInfo.segStartCarProps);
                return true;
            }();
            UNUSED(runOnce);

            linePatternDomainQueue.overwrite(linePatternDomain_t::Race);
            detectedLinesQueue.peek(detectedLines, millisecond_t(0));
            distancesQueue.peek(distances, millisecond_t(0));

            const meter_t distFromSafetyCar = Sign::POSITIVE == speedSign ? distances.front : distances.rear;

            micro::updateMainLine(detectedLines.front.lines, detectedLines.rear.lines, mainLine, car.speed >= m_per_sec_t(0));

            // sets default lateral control
            controlData.controlType          = ControlData::controlType_t::Line;
            controlData.lineControl.actual   = mainLine.centerLine;
            controlData.lineControl.desired  = { millimeter_t(0), radian_t(0) };

            if (car.distance - prevCarProps.peek_back(0).distance >= PREV_CAR_PROPS_RESOLUTION) {
                prevCarProps.push_back(car);
            }

            if (LinePattern::NONE != detectedLines.front.pattern.type) {
                lastDistWithValidLine = car.distance;
            }

            TrackSegments::const_iterator nextSeg = nextSegment(trackInfo.seg);
            if (nextSeg->hasBecomeActive(car, trackInfo, detectedLines.front.pattern)) {
                trackInfo.seg = nextSeg;
                trackInfo.segStartCarProps = car;
                trackInfo.segStartLine = mainLine.centerLine;

                if (trackSegments.begin() == trackInfo.seg) {
                    LOG_INFO("Lap %d finished (time: %f seconds)", static_cast<int32_t>(trackInfo.lap), static_cast<second_t>(getTime() - lapStartTime).get());
                    ++trackInfo.lap;
                    lapStartTime = getTime();
                }
                LOG_INFO("Segment %d became active (lap: %d)", static_cast<int32_t>(trackInfo.seg - trackSegments.begin()), static_cast<int32_t>(trackInfo.lap));
            }

            switch (programState) {
            case cfg::ProgramState::ReachSafetyCar:
                controlData.speed = speed_REACH_SAFETY_CAR;
                controlData.rampTime = millisecond_t(0);
                if (safetyCarFollowSpeed(distFromSafetyCar, speedSign, true) < controlData.speed) {
                    SystemManager::instance().setProgramState(enum_cast(cfg::ProgramState::FollowSafetyCar));
                    LOG_DEBUG("Reached safety car, starts following");
                }
                break;

            case cfg::ProgramState::FollowSafetyCar:
                controlData.speed = safetyCarFollowSpeed(distFromSafetyCar, speedSign, trackInfo.seg->isFast);
                controlData.rampTime = millisecond_t(0);

                if (distFromSafetyCar < centimeter_t(100)) {
                    lastDistWithSafetyCar = car.distance;
                }

                if (overtake.segment == trackInfo.seg && ((0 == overtake.cntr && 1 == trackInfo.lap) || (1 == overtake.cntr && 3 == trackInfo.lap))) {
                    SystemManager::instance().setProgramState(enum_cast(cfg::ProgramState::OvertakeSafetyCar));
                    LOG_DEBUG("Starts overtake");
                } else if ((trackSegments.begin() == trackInfo.seg && trackInfo.lap > 3) || car.distance - lastDistWithSafetyCar > centimeter_t(80)) {
                    SystemManager::instance().setProgramState(enum_cast(cfg::ProgramState::Race));
                    LOG_DEBUG("Safety car left the track, starts race");
                }
                break;

            case cfg::ProgramState::OvertakeSafetyCar:
                controlData.speed = safetyCarFollowSpeed(distFromSafetyCar, speedSign, trackInfo.seg->isFast);
                if (overtakeSafetyCar(car, detectedLines, controlData)) {
                    ++overtake.cntr;
                    SystemManager::instance().setProgramState(enum_cast(cfg::ProgramState::Race));
                    LOG_DEBUG("Overtake finished, starts race");
                }
                break;

            case cfg::ProgramState::Race:
                controlData = trackInfo.seg->getControl(car, trackInfo, mainLine);

                // the first 3 laps are dedicated to the safety-car follow task,
                // changing the line offset and angle might ruin the car's capability to detect the safety car
                if (trackInfo.lap <= 3) {
                    controlData.lineControl.desired = { millimeter_t(0), radian_t(0) };
                }

                if (trackInfo.lap > cfg::NUM_RACE_LAPS) {
                    SystemManager::instance().setProgramState(enum_cast(cfg::ProgramState::Finish));
                    LOG_DEBUG("Race finished");

                } else if (trackInfo.lap <= 3 &&
                           overtake.cntr < 2 &&
                           distFromSafetyCar < (trackInfo.seg->isFast ? centimeter_t(120) : centimeter_t(60))) {
                    SystemManager::instance().setProgramState(enum_cast(cfg::ProgramState::FollowSafetyCar));
                    LOG_DEBUG("Reached safety car, starts following");

                } else if (Sign::NEGATIVE == speedSign &&
                           trackInfo.lap == 3 &&
                           trackInfo.seg == getFastSegment(trackSegments, 3) &&
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
                if (turnAround(car, detectedLines, controlData)) {
                    SystemManager::instance().setProgramState(enum_cast(cfg::ProgramState::Race));
                }
                break;

            case cfg::ProgramState::Finish:
            {
                static const meter_t startDist = car.distance;
                if (car.distance - startDist < meter_t(2.0f)) {
                    controlData = trackInfo.seg->getControl(car, trackInfo, mainLine);
                } else {
                    controlData.speed = m_per_sec_t(0);
                    controlData.rampTime = millisecond_t(1500);
                }
                break;
            }

            case cfg::ProgramState::Error:
                controlData.speed = m_per_sec_t(0);
                controlData.rampTime = millisecond_t(100);
                break;

            default:
                LOG_ERROR("Invalid program state counter: [%u]", enum_cast(programState));
                break;
            }

            controlQueue.overwrite(controlData);
            prevDetectedLines = detectedLines;
        }

        SystemManager::instance().notify(true);
        os_sleep(millisecond_t(2));
    }
}
