#include <micro/task/common.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/updatable.hpp>
#include <micro/hw/SteeringServo.hpp>
#include <micro/sensor/Filter.hpp>
#include <micro/utils/Line.hpp>
#include <micro/utils/timer.hpp>
#include <micro/utils/trajectory.hpp>

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

constexpr m_per_sec_t maxSpeed_SAFETY_CAR_FAST = m_per_sec_t(1.6f);
constexpr m_per_sec_t maxSpeed_SAFETY_CAR_SLOW = m_per_sec_t(1.2f);

struct TrackSegment {
    std::function<bool(const LinePattern&, const Line&, const CarProps&, meter_t)> hasBecomeActive;
    std::function<ControlData(const Line&, uint8_t, const CarProps&, const CarProps&)> getControl;
};

bool hasBecomeActive_Fast(const LinePattern&, const Line&, const CarProps&, meter_t orientedDist) {
    return orientedDist > centimeter_t(40);
}

bool hasBecomeActive_SlowStart(const LinePattern& pattern, const Line&, const CarProps& car, meter_t) {
    static constexpr m_per_sec2_t MAX_DECELERATION = m_per_sec2_t(5);
    const meter_t brakeDist = meter_t(0.5f * car.speed.get() * car.speed.get() / MAX_DECELERATION.get());

    return LinePattern::BRAKE == pattern.type && car.distance - pattern.startDist > meter_t(3) + centimeter_t(20) - brakeDist;
}

bool hasBecomeActive_SlowRoundLeft(const LinePattern& pattern, const Line& mainLine, const CarProps&, meter_t) {
    return LinePattern::SINGLE_LINE == pattern.type && mainLine.pos < centimeter_t(5);
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
    }
    return speeds;
}

ControlData getControl_Fast(const Line& mainLine, uint8_t lap, const CarProps&, const CarProps&) {
    const std::pair<m_per_sec_t, m_per_sec_t> speeds = getSpeeds(lap);
    ControlData controlData;
    controlData.speed = abs(mainLine.pos) < centimeter_t(5) ? speeds.second : speeds.first;
    controlData.baseline = mainLine;
    controlData.offset = millimeter_t(0);
    controlData.angle = radian_t(0);
    return controlData;
}

ControlData getControl_SlowStart(const Line& mainLine, uint8_t lap, const CarProps&, const CarProps&) {
    ControlData controlData;
    controlData.speed = getSpeeds(lap).first;
    controlData.baseline = mainLine;
    controlData.offset = millimeter_t(0);
    controlData.angle = radian_t(0);
    return controlData;
}

ControlData getControl_SlowRoundLeft(const Line& mainLine, uint8_t lap, const CarProps& segStartCarProps, const CarProps& car) {
    const radian_t angleDiff = abs(normalizePM180(car.pose.angle - segStartCarProps.pose.angle));
    ControlData controlData;
    controlData.speed = getSpeeds(lap).first;
    controlData.baseline = mainLine;
    controlData.offset = angleDiff < PI_2 ? angleDiff / PI_2 * centimeter_t(10) : (PI - angleDiff) / PI_2 * centimeter_t(10);
    controlData.angle = radian_t(0);
    return controlData;
}

const vec<TrackSegment, 10> trackSegments = {
    { hasBecomeActive_Fast,          getControl_Fast },
    { hasBecomeActive_SlowStart,     getControl_SlowStart },
    { hasBecomeActive_Fast,          getControl_Fast },
    { hasBecomeActive_SlowStart,     getControl_SlowStart },
    { hasBecomeActive_SlowRoundLeft, getControl_SlowRoundLeft },
    { hasBecomeActive_Fast,          getControl_Fast },
    { hasBecomeActive_SlowStart,     getControl_SlowStart },
    { hasBecomeActive_SlowRoundLeft, getControl_SlowRoundLeft },
    { hasBecomeActive_Fast,          getControl_Fast },
    { hasBecomeActive_SlowStart,     getControl_SlowStart }
};

uint8_t lap = 1;
vec<TrackSegment, 10>::const_iterator currentSeg = trackSegments.begin();
CarProps currentSegStartCarProps;

struct {
    radian_t sectionOrientation = radian_t(0);
    Trajectory trajectory       = Trajectory(cfg::CAR_OPTO_CENTER_DIST);
} overtake;

vec<TrackSegment, 10>::const_iterator nextSegment() {
    return trackSegments.back() == currentSeg ? trackSegments.begin() : currentSeg + 1;
}

m_per_sec_t safetyCarFollowSpeed(meter_t frontDist, bool isFastSection) {
    return map(frontDist.get(), meter_t(0.3f).get(), meter_t(0.8f).get(), m_per_sec_t(0),
        isFastSection ? maxSpeed_SAFETY_CAR_FAST : maxSpeed_SAFETY_CAR_SLOW);
}

bool overtakeSafetyCar(const DetectedLines& detectedLines, ControlData& controlData) {

    static constexpr meter_t OVERTAKE_SECTION_LENGTH = centimeter_t(900);

    static constexpr meter_t SIDE_DISTANCE         = centimeter_t(60);
    static constexpr meter_t BEGIN_SINE_ARC_LENGTH = centimeter_t(150);
    static constexpr meter_t ACCELERATION_LENGTH   = centimeter_t(10);
    static constexpr meter_t BRAKE_LENGTH          = centimeter_t(100);
    static constexpr meter_t END_SINE_ARC_LENGTH   = centimeter_t(150);
    static constexpr meter_t FAST_SECTION_LENGTH   = OVERTAKE_SECTION_LENGTH - meter_t(2) - BEGIN_SINE_ARC_LENGTH - ACCELERATION_LENGTH - BRAKE_LENGTH - END_SINE_ARC_LENGTH;

    if (overtake.trajectory.length() == meter_t(0)) {

        overtake.trajectory.setStartConfig(Trajectory::config_t{
            globals::car.pose.pos + vec2m(cfg::CAR_OPTO_CENTER_DIST, centimeter_t(0)).rotate(overtake.sectionOrientation),
                    globals::speed_OVERTAKE_CURVE
        }, globals::car.distance);

        overtake.trajectory.appendSineArc(Trajectory::config_t{
            overtake.trajectory.lastConfig().pos + vec2m(BEGIN_SINE_ARC_LENGTH, -SIDE_DISTANCE).rotate(overtake.sectionOrientation),
            globals::speed_OVERTAKE_CURVE
        }, globals::car.pose.angle, 30);

        overtake.trajectory.appendLine(Trajectory::config_t{
            overtake.trajectory.lastConfig().pos + vec2m(ACCELERATION_LENGTH, centimeter_t(0)).rotate(overtake.sectionOrientation),
            globals::speed_OVERTAKE_STRAIGHT
        });

        overtake.trajectory.appendLine(Trajectory::config_t{
            overtake.trajectory.lastConfig().pos + vec2m(FAST_SECTION_LENGTH, centimeter_t(0)).rotate(overtake.sectionOrientation),
            globals::speed_OVERTAKE_STRAIGHT
        });

        overtake.trajectory.appendLine(Trajectory::config_t{
            overtake.trajectory.lastConfig().pos + vec2m(BRAKE_LENGTH, centimeter_t(0)).rotate(overtake.sectionOrientation),
            globals::speed_OVERTAKE_CURVE
        });

        overtake.trajectory.appendSineArc(Trajectory::config_t{
            overtake.trajectory.lastConfig().pos + vec2m(END_SINE_ARC_LENGTH, SIDE_DISTANCE + centimeter_t(5)).rotate(overtake.sectionOrientation),
            globals::speed_OVERTAKE_CURVE
        }, globals::car.pose.angle, 30);
    }

    controlData = overtake.trajectory.update(globals::car);

    const bool finished = overtake.trajectory.length() - overtake.trajectory.coveredDistance() < centimeter_t(40) && LinePattern::NONE != detectedLines.pattern.type;
    if (finished) {
        overtake.trajectory.clear();
    }
    return finished;
}

} // namespace

extern "C" void runProgRaceTrackTask(const void *argument) {

    vTaskDelay(500); // gives time to other tasks to wake up

    DetectedLines prevDetectedLines, detectedLines;
    ControlData controlData;
    DistancesData distances;

    bool isFastSection = false;

    meter_t startDist = globals::car.distance;
    meter_t sectionStartDist = startDist;
    meter_t lastDistWithActiveSafetyCar = startDist;

    while (true) {
        switch(getActiveTask(globals::programState)) {
        case ProgramTask::RaceTrack:
        {
            globals::distServoEnabled = true;
            globals::distSensorEnabled = true;

            xQueuePeek(detectedLinesQueue, &detectedLines, 0);
            xQueuePeek(distancesQueue, &distances, 0);

            controlData.directControl = false;
            micro::updateMainLine(detectedLines.lines.front, controlData.baseline);
            controlData.angle = degree_t(0);
            controlData.offset = millimeter_t(0);

            if (detectedLines.pattern.type != prevDetectedLines.pattern.type) {
                if (LinePattern::ACCELERATE == detectedLines.pattern.type) {
                    isFastSection = true;
                    sectionStartDist = globals::car.distance;
                } else if (LinePattern::BRAKE == detectedLines.pattern.type) {
                    isFastSection = false;
                    sectionStartDist = globals::car.distance;
                }
            }

            switch (globals::programState) {
            case ProgramState::ReachSafetyCar:
                controlData.speed = globals::speed_REACH_SAFETY_CAR;

                if (distances.front > meter_t(0) && safetyCarFollowSpeed(distances.front, false) < controlData.speed) {
                    globals::programState = ProgramState::FollowSafetyCar;
                }

//                if (distances.front < centimeter_t(60)) {
//                    globals::programState.set(ProgramTask::RaceTrack, ProgramState::FollowSafetyCar);
//                }
                break;

            case ProgramState::FollowSafetyCar:
                controlData.speed = safetyCarFollowSpeed(distances.front, isFastSection);

                if (distances.front < meter_t(1.5f)) {
                    lastDistWithActiveSafetyCar = globals::car.distance;
                }

                // when the safety car leaves the track (after a curve, before the fast signs),
//                if (isBtw(globals::car.distance - lastDistWithActiveSafetyCar, centimeter_t(50), centimeter_t(150)) &&
//                    isFastSection &&
//                    globals::car.distance - sectionStartDist < centimeter_t(5)) {
//
//                    globals::programState.set(ProgramTask::RaceTrack, ProgramState::Race);
//                }

                break;

            case ProgramState::OvertakeSafetyCar:
                if (overtakeSafetyCar(detectedLines, controlData)) {
                    globals::programState = ProgramState::Race;
                }
                break;

            case ProgramState::Race:
//                if (isFastSection || (sectionStartDist != startDist && globals::car.distance - sectionStartDist < globals::slowSectionStartOffset)) {
//                    if (!isFastSection && detectedLines.pattern.type == LinePattern::SINGLE_LINE) {
//
//                        controlData.speed = globals::speed_SLOW;
//                    } else if (abs(controlData.baseline.pos) < centimeter_t(2)) {
//                        controlData.speed = globals::speed_FAST;
//                    } else {
//                        controlData.speed = globals::speed_SLOW;
//                    }
//                    //controlData.speed = isSafe(mainLine) ? globals::speed_FAST : globals::speed_FAST_UNSAFE;
//                } else { // slow section
//                    controlData.speed = globals::speed_SLOW;
//                    //controlData.speed = isSafe(mainLine) ? globals::speed_SLOW : globals::speed_SLOW_UNSAFE;
//                }
                controlData.speed = isFastSection ? globals::speed_FAST1 : globals::speed_SLOW1;
                break;

            default:
                LOG_ERROR("Invalid program state counter: [%u]", globals::programState);
                break;
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
