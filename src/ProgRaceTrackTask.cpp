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

Timer speedSendTimer;

constexpr m_per_sec_t maxSpeed_SAFETY_CAR = m_per_sec_t(1.6f);

constexpr uint8_t NUM_LAPS = 6;

struct TrackSegment {
    std::function<bool(const LinePattern&, const Line&)> hasBecomeActive;
    std::function<ControlData(const LinePattern&, const Line&, uint8_t, const CarProps&)> getControl;
};

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

bool hasBecomeActive_SlowStart(const LinePattern& pattern, const Line&) {
    static constexpr m_per_sec2_t MAX_DECELERATION = m_per_sec2_t(4);

    static bool signDetected = false;
    static meter_t signStartDist;

    const meter_t brakeDist = meter_t(0.5f * globals::car.speed.get() * globals::car.speed.get() / MAX_DECELERATION.get());

    bool active = false;
    if (LinePattern::BRAKE == pattern.type && !signDetected) {
        signDetected = true;
        signStartDist = globals::car.distance;
    }
    if (signDetected && globals::car.distance - signStartDist > meter_t(3) - centimeter_t(100) - brakeDist) {
        signDetected = false;
        active = true;
    }
    return active;
}

bool hasBecomeActive_SlowRoundLeft(const LinePattern& pattern, const Line& mainLine) {
    return LinePattern::SINGLE_LINE == pattern.type && mainLine.pos < centimeter_t(-5);
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
    case 7: speeds = { m_per_sec_t(0),       m_per_sec_t(7 )      }; break;
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

ControlData getControl_SlowStart(const LinePattern& pattern, const Line& mainLine, uint8_t lap, const CarProps&) {

    static bool fastSignDetected = false;
    static meter_t startDist = globals::car.distance;

    if (LinePattern::BRAKE == pattern.type) {
        startDist = globals::car.distance;
    }

    if (LinePattern::ACCELERATE == pattern.type) {
        fastSignDetected = true;
    } else if (globals::car.distance - startDist < centimeter_t(20)) {
        fastSignDetected = false;
    }

    const bool brakeNeeded = globals::car.distance - startDist < centimeter_t(150) ||
        globals::car.distance - startDist > centimeter_t(500);

    ControlData controlData;
    controlData.speed = brakeNeeded ? m_per_sec_t(1.5f) : getSpeeds(lap).first;
    controlData.rampTime = millisecond_t(200);
    controlData.baseline = mainLine;
    controlData.offset = millimeter_t(0);
    controlData.angle = radian_t(0);
    controlData.rearServoEnabled = !fastSignDetected;
    return controlData;
}

ControlData getControl_SlowRoundLeft(const LinePattern&, const Line& mainLine, uint8_t lap, const CarProps& segStartCarProps) {
    const radian_t angleDiff = abs(normalizePM180(globals::car.pose.angle - segStartCarProps.pose.angle));
    ControlData controlData;
    controlData.speed = getSpeeds(lap).first;
    controlData.rampTime = millisecond_t(0);
    controlData.baseline = mainLine;
    controlData.offset = angleDiff < PI_2 ? angleDiff / PI_2 * centimeter_t(10) : (PI - angleDiff) / PI_2 * centimeter_t(10);
    controlData.angle = radian_t(0);
    controlData.rearServoEnabled = false;
    return controlData;
}

constexpr meter_t PREV_CAR_PROPS_RESOLUTION = centimeter_t(5);
infinite_buffer<CarProps, static_cast<uint32_t>(meter_t(5) / PREV_CAR_PROPS_RESOLUTION)> prevCarProps;

typedef vec<TrackSegment, 10> TrackSegments;

const TrackSegments trackSegments = {
    { hasBecomeActive_Fast,          getControl_Fast },
    { hasBecomeActive_SlowStart,     getControl_SlowStart },
    { hasBecomeActive_Fast,          getControl_Fast },
    { hasBecomeActive_SlowStart,     getControl_SlowStart },
    //{ hasBecomeActive_SlowRoundLeft, getControl_SlowRoundLeft },
    { hasBecomeActive_Fast,          getControl_Fast },
    { hasBecomeActive_SlowStart,     getControl_SlowStart },
    //{ hasBecomeActive_SlowRoundLeft, getControl_SlowRoundLeft },
    { hasBecomeActive_Fast,          getControl_Fast },
    { hasBecomeActive_SlowStart,     getControl_SlowStart }
};

struct {
    const TrackSegments::const_iterator segment = trackSegments.begin() + 5;
    radian_t orientation  = radian_t(0);
    Trajectory trajectory = Trajectory(cfg::CAR_OPTO_CENTER_DIST);
    uint8_t cntr = 0;
} overtake;

TrackSegments::const_iterator nextSegment(const TrackSegments::const_iterator currentSeg) {
    return trackSegments.back() == currentSeg ? trackSegments.begin() : currentSeg + 1;
}

m_per_sec_t safetyCarFollowSpeed(meter_t frontDist) {
    return map(frontDist.get(), meter_t(0.3f).get(), meter_t(0.8f).get(), m_per_sec_t(0), maxSpeed_SAFETY_CAR);
}

bool overtakeSafetyCar(const DetectedLines& detectedLines, ControlData& controlData) {

    static constexpr meter_t OVERTAKE_SECTION_LENGTH = centimeter_t(900);

    static constexpr meter_t SIDE_DISTANCE         = centimeter_t(45);
    static constexpr meter_t BEGIN_SINE_ARC_LENGTH = centimeter_t(150);
    static constexpr meter_t ACCELERATION_LENGTH   = centimeter_t(100);
    static constexpr meter_t BRAKE_LENGTH          = centimeter_t(100);
    static constexpr meter_t END_SINE_ARC_LENGTH   = centimeter_t(150);
    static constexpr meter_t FAST_SECTION_LENGTH   = OVERTAKE_SECTION_LENGTH - meter_t(1.5f) - BEGIN_SINE_ARC_LENGTH - ACCELERATION_LENGTH - BRAKE_LENGTH - END_SINE_ARC_LENGTH;

    bool finished = false;

    if (overtake.trajectory.length() == meter_t(0)) {

        static constexpr meter_t ORIENTATION_FILTER_DIST = centimeter_t(70);
        if (globals::car.orientedDistance > ORIENTATION_FILTER_DIST) {

            const point2m posDiff = globals::car.pose.pos - prevCarProps.peek_back(static_cast<uint32_t>(ORIENTATION_FILTER_DIST / PREV_CAR_PROPS_RESOLUTION)).pose.pos;
            overtake.orientation = posDiff.getAngle();

            overtake.trajectory.setStartConfig(Trajectory::config_t{
                globals::car.pose.pos + vec2m(cfg::CAR_OPTO_CENTER_DIST, centimeter_t(0)).rotate(overtake.orientation),
                globals::car.speed
            }, globals::car.distance);

            overtake.trajectory.appendSineArc(Trajectory::config_t{
                overtake.trajectory.lastConfig().pos + vec2m(BEGIN_SINE_ARC_LENGTH, SIDE_DISTANCE).rotate(overtake.orientation),
                m_per_sec_t(1.0f)
            }, globals::car.pose.angle, 50);

            overtake.trajectory.appendLine(Trajectory::config_t{
                overtake.trajectory.lastConfig().pos + vec2m(ACCELERATION_LENGTH / 2, centimeter_t(0)).rotate(overtake.orientation),
                m_per_sec_t(1.0f)
            });

            overtake.trajectory.appendLine(Trajectory::config_t{
                overtake.trajectory.lastConfig().pos + vec2m(ACCELERATION_LENGTH / 2, centimeter_t(0)).rotate(overtake.orientation),
                m_per_sec_t(2.0f)
            });

            for (uint8_t i = 0; i < 100; ++i) {
                overtake.trajectory.appendLine(Trajectory::config_t{
                    overtake.trajectory.lastConfig().pos + vec2m(FAST_SECTION_LENGTH / 100, centimeter_t(0)).rotate(overtake.orientation),
                    m_per_sec_t(2.0f)
                });
            }

            overtake.trajectory.appendLine(Trajectory::config_t{
                overtake.trajectory.lastConfig().pos + vec2m(BRAKE_LENGTH, centimeter_t(0)).rotate(overtake.orientation),
                m_per_sec_t(1.5f)
            });

            overtake.trajectory.appendSineArc(Trajectory::config_t{
                overtake.trajectory.lastConfig().pos + vec2m(END_SINE_ARC_LENGTH, -SIDE_DISTANCE - centimeter_t(5)).rotate(overtake.orientation),
                m_per_sec_t(1.0f)
            }, globals::car.pose.angle, 50);
        }
    }

    if (overtake.trajectory.length() > meter_t(0)) {
        controlData = overtake.trajectory.update(globals::car);
        controlData.rearServoEnabled = false;

        finished = overtake.trajectory.length() - overtake.trajectory.coveredDistance() < centimeter_t(40) && LinePattern::NONE != detectedLines.pattern.type;
        if (finished) {
            overtake.trajectory.clear();
        }
    }

    return finished;
}

bool overtakeSafetyCar2(const DetectedLines& detectedLines, ControlData& controlData) {

    static constexpr meter_t OVERTAKE_SECTION_LENGTH = centimeter_t(900);

    static constexpr meter_t SIDE_DISTANCE         = centimeter_t(45);
    static constexpr meter_t BEGIN_SINE_ARC_LENGTH = centimeter_t(150);
    static constexpr meter_t ACCELERATION_LENGTH   = centimeter_t(100);
    static constexpr meter_t BRAKE_LENGTH          = centimeter_t(100);
    static constexpr meter_t END_SINE_ARC_LENGTH   = centimeter_t(150);
    static constexpr meter_t FAST_SECTION_LENGTH   = OVERTAKE_SECTION_LENGTH - meter_t(1.5f) - BEGIN_SINE_ARC_LENGTH - ACCELERATION_LENGTH - BRAKE_LENGTH - END_SINE_ARC_LENGTH;

    bool finished = false;

    if (overtake.trajectory.length() == meter_t(0)) {

        static constexpr meter_t ORIENTATION_FILTER_DIST = centimeter_t(70);
        if (globals::car.orientedDistance > ORIENTATION_FILTER_DIST) {

            const point2m posDiff = globals::car.pose.pos - prevCarProps.peek_back(static_cast<uint32_t>(ORIENTATION_FILTER_DIST / PREV_CAR_PROPS_RESOLUTION)).pose.pos;
            overtake.orientation = posDiff.getAngle();

            overtake.trajectory.setStartConfig(Trajectory::config_t{
                globals::car.pose.pos + vec2m(cfg::CAR_OPTO_CENTER_DIST, centimeter_t(0)).rotate(overtake.orientation),
                globals::car.speed
            }, globals::car.distance);

            overtake.trajectory.appendSineArc(Trajectory::config_t{
                overtake.trajectory.lastConfig().pos + vec2m(BEGIN_SINE_ARC_LENGTH, SIDE_DISTANCE).rotate(overtake.orientation),
                m_per_sec_t(1.0f)
            }, overtake.orientation, 50);

            overtake.trajectory.appendLine(Trajectory::config_t{
                overtake.trajectory.lastConfig().pos + vec2m(ACCELERATION_LENGTH / 2, centimeter_t(0)).rotate(overtake.orientation),
                m_per_sec_t(1.0f)
            });

            overtake.trajectory.appendLine(Trajectory::config_t{
                overtake.trajectory.lastConfig().pos + vec2m(ACCELERATION_LENGTH / 2, centimeter_t(0)).rotate(overtake.orientation),
                m_per_sec_t(2.0f)
            });

            for (uint8_t i = 0; i < 100; ++i) {
                overtake.trajectory.appendLine(Trajectory::config_t{
                    overtake.trajectory.lastConfig().pos + vec2m(FAST_SECTION_LENGTH / 100, centimeter_t(0)).rotate(overtake.orientation),
                    m_per_sec_t(2.0f)
                });
            }

            overtake.trajectory.appendLine(Trajectory::config_t{
                overtake.trajectory.lastConfig().pos + vec2m(BRAKE_LENGTH, centimeter_t(0)).rotate(overtake.orientation),
                m_per_sec_t(1.5f)
            });

            overtake.trajectory.appendSineArc(Trajectory::config_t{
                overtake.trajectory.lastConfig().pos + vec2m(END_SINE_ARC_LENGTH, -SIDE_DISTANCE - centimeter_t(5)).rotate(overtake.orientation),
                m_per_sec_t(1.0f)
            }, overtake.orientation, 50);
        }
    }

    if (overtake.trajectory.length() > meter_t(0)) {
        controlData = overtake.trajectory.update(globals::car);
        controlData.rearServoEnabled = false;

        finished = overtake.trajectory.length() - overtake.trajectory.coveredDistance() < centimeter_t(40) && LinePattern::NONE != detectedLines.pattern.type;
        if (finished) {
            overtake.trajectory.clear();
        }
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

    speedSendTimer.start(second_t(1));

    while (true) {
        switch(getActiveTask(globals::programState)) {
        case ProgramTask::RaceTrack:
        {
            static const bool runOnce = [&lap, &currentSeg, &currentSegStartCarProps]() {
                // TODO set according to the number of clicks
                lap = 1;
                currentSeg = trackSegments.begin();
                currentSegStartCarProps = globals::car;
                prevCarProps.push_back(currentSegStartCarProps);
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

            TrackSegments::const_iterator nextSeg = nextSegment(currentSeg);
            if (nextSeg->hasBecomeActive(detectedLines.pattern, controlData.baseline)) {
                currentSeg = nextSeg;
                currentSegStartCarProps = globals::car;
                if (trackSegments.begin() == currentSeg) {
                    //++lap; TODO
                }
            }

            switch (globals::programState) {
            case ProgramState::ReachSafetyCar:
                controlData.speed = globals::speed_REACH_SAFETY_CAR;
                controlData.rampTime = millisecond_t(0);
                if (distances.front > meter_t(0) && safetyCarFollowSpeed(distances.front) < controlData.speed) {
                    globals::programState = ProgramState::FollowSafetyCar;
                }
                break;

            case ProgramState::FollowSafetyCar:
                controlData.speed = safetyCarFollowSpeed(distances.front);
                controlData.rampTime = millisecond_t(0);
                if (overtake.segment == currentSeg) {
                    globals::programState = ProgramState::OvertakeSafetyCar;
                }
                break;

            case ProgramState::OvertakeSafetyCar:
                controlData.speed = safetyCarFollowSpeed(distances.front);
                if (overtakeSafetyCar2(detectedLines, controlData)) {
                    ++overtake.cntr;
                    globals::programState = ProgramState::Race;
                }
                break;

            case ProgramState::Race:

                controlData = currentSeg->getControl(detectedLines.pattern, controlData.baseline, lap, currentSegStartCarProps);
                if (lap > NUM_LAPS) {
                    globals::programState = ProgramState::Finish;
                } else if (overtake.cntr < 2 && distances.front < centimeter_t(80)) {
                    //globals::programState = ProgramState::FollowSafetyCar;
                }
                break;
            case ProgramState::Finish:
            {
                static const meter_t startDist = globals::car.distance;
                if (globals::car.distance - startDist < meter_t(3.5f)) {
                    controlData = currentSeg->getControl(detectedLines.pattern, controlData.baseline, lap, currentSegStartCarProps);
                } else {
                    controlData.speed = m_per_sec_t(0);
                    controlData.rampTime = millisecond_t(1500);
                }
                break;
            }

            default:
                LOG_ERROR("Invalid program state counter: [%u]", globals::programState);
                break;
            }

            xQueueOverwrite(controlQueue, &controlData);
            prevDetectedLines = detectedLines;

            if (speedSendTimer.checkTimeout()) {
                LOG_DEBUG("speed: %fm/s", globals::car.speed.get());
            }

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
