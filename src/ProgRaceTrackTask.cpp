#include <micro/task/common.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/updatable.hpp>
#include <micro/hw/SteeringServo.hpp>
#include <micro/sensor/Filter.hpp>
#include <micro/utils/Line.hpp>
#include <micro/utils/timer.hpp>
#include <micro/utils/trajectory.hpp>
#include <micro/container/infinite_buffer.hpp>

#include <DetectedLines.hpp>
#include <ControlData.hpp>
#include <DistancesData.hpp>
#include <cfg_board.h>
#include <cfg_car.hpp>
#include <globals.hpp>

#include <FreeRTOS.h>
#include <micro/panel/LineDetectPanelLink.hpp>
#include <micro/panel/MotorPanelLink.hpp>
#include <task.h>
#include <queue.h>

using namespace micro;

extern QueueHandle_t detectedLinesQueue;
extern QueueHandle_t controlQueue;
extern QueueHandle_t distancesQueue;

namespace {

constexpr m_per_sec_t maxSpeed_SAFETY_CAR_SLOW = m_per_sec_t(1.3f);
constexpr m_per_sec_t maxSpeed_SAFETY_CAR_FAST = m_per_sec_t(1.8f);

constexpr uint8_t NUM_LAPS = 6;

struct TrackSegment {
    bool isFast;
    std::function<bool(const LinePattern&, const Line&)> hasBecomeActive;
    std::function<ControlData(const LinePattern&, const Line&, uint8_t, const CarProps&)> getControl;
};

typedef vec<TrackSegment, 10> TrackSegments;

struct {
    TrackSegments::const_iterator segment;
    meter_t startDist;
    radian_t orientation  = radian_t(0);
    Trajectory trajectory = Trajectory(cfg::CAR_OPTO_REAR_PIVOT_DIST);
    uint8_t cntr = 0;
    bool forcedEndManeuverActive = false;
    meter_t forcedEndManeuverStartDist;
} overtake;

bool forceInstantBrake = true;
bool forceSlowSpeed = false;

bool hasBecomeActive_Fast(const LinePattern& pattern, const Line& mainLine) {
    static bool signDetected = false;

    bool active = false;
    if (LinePattern::ACCELERATE == pattern.type) {
        signDetected = true;
    }
    if (signDetected && globals::car.orientedDistance > centimeter_t(50)) {
        signDetected = false;
        active = true;
    }
    return active;
}

bool hasBecomeActive_Slow(const LinePattern& pattern, const Line&) {
    static constexpr m_per_sec2_t MAX_DECELERATION = m_per_sec2_t(4);

    static bool signDetected = false;
    static meter_t signStartDist;

    const meter_t brakeDist = meter_t(0.5f * globals::car.speed.get() * globals::car.speed.get() / MAX_DECELERATION.get());

    bool active = false;
    if (LinePattern::BRAKE == pattern.type && !signDetected) {
        signDetected = true;
        signStartDist = globals::car.distance;
    }
    if (signDetected && (forceInstantBrake || globals::car.distance - signStartDist > meter_t(3) - globals::dist_BRAKE_OFFSET - brakeDist)) {

        signDetected = false;
        active = true;
    }
    return active;
}

std::pair<m_per_sec_t, m_per_sec_t> getSpeeds(uint8_t lap) {
    std::pair<m_per_sec_t, m_per_sec_t> speeds;
    switch(lap) {
    case 1: speeds = { globals::speed_SLOW1, globals::speed_FAST1 }; break;
    case 2: speeds = { globals::speed_SLOW2, globals::speed_FAST2 }; break;
    case 3: speeds = { globals::speed_SLOW3, globals::speed_FAST3 }; break;
    case 4: speeds = { globals::speed_SLOW4, globals::speed_FAST4 }; break;
    case 5: speeds = { globals::speed_SLOW5, globals::speed_FAST5 }; break;
    case 6: speeds = { globals::speed_SLOW6, globals::speed_FAST6 }; break;
    case 7: speeds = { m_per_sec_t(0),       m_per_sec_t(7)       }; break;
    }
    return speeds;
}

ControlData getControl_Fast(const LinePattern&, const Line& mainLine, uint8_t lap, const CarProps&) {
    const std::pair<m_per_sec_t, m_per_sec_t> speeds = getSpeeds(lap);

    static bool enabled = true;

    if (enabled && abs(mainLine.pos) > centimeter_t(8)) {
        enabled = false;
    } else if (!enabled && globals::car.orientedDistance > centimeter_t(50)) {
        enabled = true;
    }

    ControlData controlData;
    controlData.speed = enabled ? speeds.second : speeds.first;
    controlData.rampTime = millisecond_t(800);
    controlData.baseline = mainLine;
    controlData.offset = millimeter_t(0);
    controlData.angle = radian_t(0);
    controlData.rearServoEnabled = false;
    return controlData;
}

ControlData getControl_Slow(const LinePattern& pattern, const Line& mainLine, uint8_t lap, const CarProps&) {

    static meter_t startDist = globals::car.distance;

    if (LinePattern::BRAKE == pattern.type) {
        startDist = globals::car.distance;
    }

    const bool isSectionStart = globals::car.distance - startDist < centimeter_t(150) ||
        globals::car.distance - startDist > centimeter_t(450);

    ControlData controlData;
    controlData.speed = isSectionStart ? globals::speed_SLOW_START : getSpeeds(lap).first;
    controlData.rampTime = millisecond_t(500);
    controlData.baseline = mainLine;
    controlData.offset = millimeter_t(0);
    controlData.angle = radian_t(0);
    controlData.rearServoEnabled = globals::car.distance - startDist > centimeter_t(2);
    return controlData;
}

constexpr meter_t PREV_CAR_PROPS_RESOLUTION = centimeter_t(5);
infinite_buffer<CarProps, static_cast<uint32_t>(meter_t(5) / PREV_CAR_PROPS_RESOLUTION)> prevCarProps;

const TrackSegments trackSegments = {
    { true,  hasBecomeActive_Fast, getControl_Fast },
    { false, hasBecomeActive_Slow, getControl_Slow },
    { true,  hasBecomeActive_Fast, getControl_Fast },
    { false, hasBecomeActive_Slow, getControl_Slow },
    { true,  hasBecomeActive_Fast, getControl_Fast },
    { false, hasBecomeActive_Slow, getControl_Slow },
    { true,  hasBecomeActive_Fast, getControl_Fast },
    { false, hasBecomeActive_Slow, getControl_Slow }
};

TrackSegments::const_iterator nextSegment(const TrackSegments::const_iterator currentSeg) {
    return trackSegments.back() == currentSeg ? trackSegments.begin() : currentSeg + 1;
}

m_per_sec_t safetyCarFollowSpeed(meter_t frontDist, bool isFast) {
    return map(frontDist.get(), meter_t(0.3f).get(), meter_t(0.8f).get(), m_per_sec_t(0), isFast ? maxSpeed_SAFETY_CAR_FAST : maxSpeed_SAFETY_CAR_SLOW);
}

bool overtakeSafetyCar(const DetectedLines& detectedLines, ControlData& controlData) {

    static constexpr meter_t OVERTAKE_SECTION_LENGTH = centimeter_t(900);

    static constexpr meter_t ORIENTATION_FILTER_DIST = centimeter_t(150);
    static constexpr meter_t BEGIN_SINE_ARC_LENGTH   = centimeter_t(120);
    static constexpr meter_t ACCELERATION_LENGTH     = centimeter_t(100);
    static constexpr meter_t BRAKE_LENGTH            = centimeter_t(50);
    static constexpr meter_t FAST_SECTION_LENGTH     = OVERTAKE_SECTION_LENGTH - ORIENTATION_FILTER_DIST - BEGIN_SINE_ARC_LENGTH - ACCELERATION_LENGTH - BRAKE_LENGTH;

    bool finished = false;

    if (overtake.trajectory.length() == meter_t(0)) {

        if (overtake.startDist == meter_t(0)) {
            overtake.startDist = globals::car.distance;
        }

        const meter_t overtakeDist = globals::car.distance - overtake.startDist;

        if (overtakeDist > ORIENTATION_FILTER_DIST) {

            const point2m posDiff = globals::car.pose.pos - prevCarProps.peek_back(static_cast<uint32_t>(overtakeDist / PREV_CAR_PROPS_RESOLUTION)).pose.pos;
            overtake.orientation = posDiff.getAngle();

            overtake.trajectory.setStartConfig(Trajectory::config_t{
                globals::car.pose.pos + vec2m(cfg::CAR_OPTO_REAR_PIVOT_DIST, centimeter_t(0)).rotate(overtake.orientation),
                clamp(globals::car.speed, m_per_sec_t(0.5f), globals::speed_OVERTAKE_CURVE)
            }, globals::car.distance);

            overtake.trajectory.appendSineArc(Trajectory::config_t{
                overtake.trajectory.lastConfig().pos + vec2m(BEGIN_SINE_ARC_LENGTH, globals::dist_OVERTAKE_SIDE).rotate(overtake.orientation),
                globals::speed_OVERTAKE_CURVE
            }, globals::car.pose.angle, 50);

            overtake.trajectory.appendLine(Trajectory::config_t{
                overtake.trajectory.lastConfig().pos + vec2m(ACCELERATION_LENGTH, centimeter_t(0)).rotate(overtake.orientation),
                globals::speed_OVERTAKE_STRAIGHT
            });

            for (uint8_t i = 0; i < 10; ++i) {
                overtake.trajectory.appendLine(Trajectory::config_t{
                    overtake.trajectory.lastConfig().pos + vec2m(FAST_SECTION_LENGTH / 10, centimeter_t(0)).rotate(overtake.orientation),
                    globals::speed_OVERTAKE_STRAIGHT
                });
            }

            overtake.trajectory.appendLine(Trajectory::config_t{
                overtake.trajectory.lastConfig().pos + vec2m(BRAKE_LENGTH, centimeter_t(0)).rotate(overtake.orientation),
                globals::speed_OVERTAKE_CURVE
            });
        }
    }

    if (overtake.trajectory.length() > meter_t(0)) {
        controlData = overtake.trajectory.update(globals::car);
        controlData.rearServoEnabled = false;

        if (overtake.trajectory.length() - overtake.trajectory.coveredDistance() < centimeter_t(10)) {
            overtake.forcedEndManeuverActive = true;
            overtake.forcedEndManeuverStartDist = globals::car.distance;
            overtake.trajectory.clear();
        }
    }

    if (overtake.forcedEndManeuverActive) {
        controlData.speed = globals::speed_OVERTAKE_CURVE;
        controlData.rearServoEnabled = true;
        controlData.directControl = true;
        controlData.frontWheelAngle = degree_t(-17);
        controlData.rearWheelAngle = degree_t(-17);
    }

    finished = overtake.forcedEndManeuverActive &&
               globals::car.distance - overtake.forcedEndManeuverStartDist > centimeter_t(50) &&
               LinePattern::NONE != detectedLines.pattern.type;
    if (finished) {
        overtake.trajectory.clear();
        overtake.forcedEndManeuverActive = false;
        overtake.forcedEndManeuverStartDist = meter_t(0);
        overtake.startDist = meter_t(0);
    }

    return finished;
}

} // namespace

extern "C" void runProgRaceTrackTask(const void *argument) {

    vTaskDelay(500); // gives time to other tasks to wake up

    DetectedLines prevDetectedLines, detectedLines;
    ControlData controlData;
    DistancesData distances;

    uint8_t lap;
    TrackSegments::const_iterator currentSeg;
    CarProps currentSegStartCarProps;

    overtake.segment = trackSegments.begin() + 4;

    meter_t lastDistWithValidLine;
    millisecond_t lapStartTime;

    while (true) {
        switch(getActiveTask(globals::programState)) {
        case ProgramTask::RaceTrack:
        {
            static const bool runOnce = [&lap, &currentSeg, &currentSegStartCarProps, &lastDistWithValidLine]() {

                if (ProgramState::Race          == globals::programState ||
                    ProgramState::Race_segFast2 == globals::programState ||
                    ProgramState::Race_segFast3 == globals::programState ||
                    ProgramState::Race_segFast4 == globals::programState) {

                    lap = 3;
                    currentSeg = ProgramState::Race          == globals::programState ? trackSegments.begin() :
                                 ProgramState::Race_segFast2 == globals::programState ? trackSegments.begin() + 2 :
                                 ProgramState::Race_segFast3 == globals::programState ? trackSegments.begin() + 5 :
                                 trackSegments.begin() + 8;
                    forceSlowSpeed = true;
                } else {
                    lap = 1;
                    currentSeg = trackSegments.begin();
                    forceSlowSpeed = false;
                }

                lastDistWithValidLine = globals::car.distance;
                lapStartTime = getTime();

                currentSegStartCarProps = globals::car;
                prevCarProps.push_back(currentSegStartCarProps);
                forceInstantBrake = true;
                return true;
            }();
            UNUSED(runOnce);

            globals::distServoEnabled = true;
            globals::distSensorEnabled = true;

            xQueuePeek(detectedLinesQueue, &detectedLines, 0);
            xQueuePeek(distancesQueue, &distances, 0);

            controlData.directControl = false;
            micro::updateMainLine(detectedLines.lines.front, controlData.baseline);
            controlData.angle = degree_t(0);
            controlData.offset = millimeter_t(0);

            if (globals::car.distance - prevCarProps.peek_back(0).distance >= PREV_CAR_PROPS_RESOLUTION) {
                prevCarProps.push_back(globals::car);
            }

            if (LinePattern::NONE != detectedLines.pattern.type) {
                lastDistWithValidLine = globals::car.distance;
            }

            TrackSegments::const_iterator nextSeg = nextSegment(currentSeg);
            if (nextSeg->hasBecomeActive(detectedLines.pattern, controlData.baseline)) {
                forceSlowSpeed = false;
                currentSeg = nextSeg;
                currentSegStartCarProps = globals::car;
                if (trackSegments.begin() == currentSeg) {
                    LOG_INFO("Lap %d finished (time: %f seconds)", static_cast<int32_t>(lap), static_cast<second_t>(getTime() - lapStartTime).get());
                    ++lap;
                    lapStartTime = getTime();
                }
                LOG_INFO("Segment %d became active (lap: %d)", static_cast<int32_t>(currentSeg - trackSegments.begin()), static_cast<int32_t>(lap));
            }

            switch (globals::programState) {
            case ProgramState::ReachSafetyCar:
                controlData.speed = globals::speed_REACH_SAFETY_CAR;
                controlData.rampTime = millisecond_t(0);
                controlData.rearServoEnabled = false;
                forceInstantBrake = true;
                if (distances.front > meter_t(0) && safetyCarFollowSpeed(distances.front, true) < controlData.speed) {
                    globals::programState = ProgramState::FollowSafetyCar;
                    LOG_DEBUG("Reached safety car, starts following");
                }
                break;

            case ProgramState::FollowSafetyCar:
                static meter_t lastDistWithSafetyCar;
                controlData.speed = safetyCarFollowSpeed(distances.front, currentSeg->isFast);
                controlData.rampTime = millisecond_t(0);
                controlData.rearServoEnabled = false;
                forceInstantBrake = true;

                if (distances.front < centimeter_t(100)) {
                    lastDistWithSafetyCar = globals::car.distance;
                }

                if (overtake.segment == currentSeg &&
                    ((0 == overtake.cntr && 1 == lap) || (1 == overtake.cntr && 3 == lap)) &&
                    globals::car.distance - detectedLines.pattern.startDist < centimeter_t(50)) {
                    globals::programState = ProgramState::OvertakeSafetyCar;
                    LOG_DEBUG("Starts overtake");
                } else if ((trackSegments.begin() == currentSeg && lap > 3) || globals::car.distance - lastDistWithSafetyCar > centimeter_t(80)) {
                    globals::programState = ProgramState::Race;
                    LOG_DEBUG("Safety car left the track, starts race");
                }
                break;

            case ProgramState::OvertakeSafetyCar:
                controlData.speed = safetyCarFollowSpeed(distances.front, currentSeg->isFast);
                forceInstantBrake = true;
                if (overtakeSafetyCar(detectedLines, controlData)) {
                    ++overtake.cntr;
                    globals::programState = ProgramState::Race;
                    LOG_DEBUG("Overtake finished, starts race");
                }
                break;

            case ProgramState::Race:
                forceInstantBrake = lap < 4;
                controlData = currentSeg->getControl(detectedLines.pattern, controlData.baseline, lap, currentSegStartCarProps);
                if (lap > NUM_LAPS) {
                    globals::programState = ProgramState::Finish;
                    LOG_DEBUG("Race finished");
                } else if (lap <= 3 && overtake.cntr < 2 && distances.front < (currentSeg->isFast ? centimeter_t(120) : centimeter_t(60))) {
                    globals::programState = ProgramState::FollowSafetyCar;
                    LOG_DEBUG("Reached safety car, starts following");
                }

                if (globals::car.distance - lastDistWithValidLine > meter_t(2)) {
                    globals::programState = ProgramState::Error;
                    LOG_ERROR("An error has occurred. Car stopped.");
                }

                break;
            case ProgramState::Finish:
            {
                static const meter_t startDist = globals::car.distance;
                if (globals::car.distance - startDist < meter_t(5.0f)) {
                    controlData = currentSeg->getControl(detectedLines.pattern, controlData.baseline, lap, currentSegStartCarProps);
                } else {
                    controlData.speed = m_per_sec_t(0);
                    controlData.rampTime = millisecond_t(1500);
                }
                break;
            }

            case ProgramState::Error:
                controlData.speed = m_per_sec_t(0);
                controlData.rampTime = millisecond_t(100);
                break;

            default:
                LOG_ERROR("Invalid program state counter: [%u]", globals::programState);
                break;
            }

            if (forceSlowSpeed) {
                controlData.speed = min(controlData.speed, globals::speed_SLOW1);
            }

            xQueueOverwrite(controlQueue, &controlData);
            prevDetectedLines = detectedLines;

            vTaskDelay(2);
            break;
        }

        default:
            vTaskDelay(20);
            break;
        }
    }

    vTaskDelete(nullptr);
}
