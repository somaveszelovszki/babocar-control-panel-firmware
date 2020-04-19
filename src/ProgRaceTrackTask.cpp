#include <micro/container/infinite_buffer.hpp>
#include <micro/hw/SteeringServo.hpp>
#include <micro/sensor/Filter.hpp>
#include <micro/utils/ControlData.hpp>
#include <micro/utils/Line.hpp>
#include <micro/utils/timer.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/trajectory.hpp>
#include <micro/utils/updatable.hpp>

#include <cfg_board.h>
#include <cfg_car.hpp>
#include <DetectedLines.hpp>
#include <DistancesData.hpp>
#include <globals.hpp>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

using namespace micro;

extern QueueHandle_t detectedLinesQueue;
extern QueueHandle_t controlQueue;
extern QueueHandle_t distancesQueue;

namespace {

constexpr m_per_sec_t MAX_SPEED_SAFETY_CAR_SLOW = m_per_sec_t(1.3f);
constexpr m_per_sec_t MAX_SPEED_SAFETY_CAR_FAST = m_per_sec_t(1.8f);
constexpr meter_t PREV_CAR_PROPS_RESOLUTION     = centimeter_t(5);

struct TrackSegment {
    bool isFast;
    std::function<bool(const LinePattern&, const MainLine&)> hasBecomeActive;
    std::function<ControlData(const LinePattern&, const MainLine&)> getControl;
};

typedef vec<TrackSegment, 20> TrackSegments;

struct {
    TrackSegments::const_iterator segment;
    meter_t startDist;
    radian_t orientation;
    Trajectory trajectory;
    uint8_t cntr = 0;
} overtake;

struct {
    uint8_t lap;
    TrackSegments::const_iterator trackSeg;
    CarProps segStartCarProps;
} currentSeg;

infinite_buffer<CarProps, static_cast<uint32_t>(meter_t(5) / PREV_CAR_PROPS_RESOLUTION)> prevCarProps;

bool forceInstantBrake = true;
bool forceSlowSpeed = false;

bool hasBecomeActive_Fast(const LinePattern& pattern, const MainLine& mainLine) {
    static bool signDetected = false;
    static meter_t lastSignDist = meter_t(0);

    bool active = false;
    if (LinePattern::ACCELERATE == pattern.type) {
        signDetected = true;
        lastSignDist = globals::car.distance;
    } else if (globals::car.distance - lastSignDist > meter_t(5)) {
        signDetected = false;
    }

    if (signDetected && globals::car.orientedDistance > centimeter_t(25)) {
        signDetected = false;
        active = true;
    }
    return active;
}

bool hasBecomeActive_Slow1_prepare(const LinePattern& pattern, const MainLine&) {
    return LinePattern::BRAKE == pattern.type;
}

bool hasBecomeActive_Slow1(const LinePattern& pattern, const MainLine&) {
    return globals::car.distance - currentSeg.segStartCarProps.distance > meter_t(3);
}

bool hasBecomeActive_Slow2_prepare(const LinePattern& pattern, const MainLine&) {
    return LinePattern::BRAKE == pattern.type;
}

bool hasBecomeActive_Slow2_begin(const LinePattern& pattern, const MainLine&) {
    return globals::car.distance - currentSeg.segStartCarProps.distance > meter_t(3);
}

bool hasBecomeActive_Slow2_round(const LinePattern& pattern, const MainLine& mainLine) {
    return globals::car.distance - currentSeg.segStartCarProps.distance > centimeter_t(120);
}

bool hasBecomeActive_Slow3_prepare(const LinePattern& pattern, const MainLine&) {
    return LinePattern::BRAKE == pattern.type;
}

bool hasBecomeActive_Slow3_round(const LinePattern& pattern, const MainLine&) {
    return LinePattern::BRAKE == pattern.type;
}

bool hasBecomeActive_Slow3_end(const LinePattern& pattern, const MainLine&) {
    return globals::car.distance - currentSeg.segStartCarProps.distance > meter_t(1) * PI.get();
}

bool hasBecomeActive_Slow4_prepare(const LinePattern& pattern, const MainLine&) {
    return LinePattern::BRAKE == pattern.type;
}

bool hasBecomeActive_Slow4(const LinePattern& pattern, const MainLine&) {
    return globals::car.distance - currentSeg.segStartCarProps.distance > meter_t(3);
}

const TrackSpeeds& getSpeeds(uint8_t lap) {
    return globals::trackSpeeds[lap - 1];
}

ControlData getControl_CommonFast(const MainLine& mainLine) {
    static bool fastSpeedEnabled = true;

    if (fastSpeedEnabled && abs(mainLine.centerLine.pos) > centimeter_t(8)) {
        fastSpeedEnabled = false;
    } else if (!fastSpeedEnabled && globals::car.orientedDistance > centimeter_t(50)) {
        fastSpeedEnabled = true;
    }

    ControlData controlData;
    controlData.speed                = fastSpeedEnabled && !forceSlowSpeed ? getSpeeds(currentSeg.lap).fast : m_per_sec_t(2.0f);
    controlData.rampTime             = millisecond_t(500);
    controlData.controlType          = ControlData::controlType_t::Line;
    controlData.lineControl.baseline = mainLine.centerLine;
    controlData.lineControl.offset   = millimeter_t(0);
    controlData.lineControl.angle    = radian_t(0);
    return controlData;
}

ControlData getControl_CommonSlow(const LinePattern& pattern, const MainLine& mainLine) {
    ControlData controlData;
    controlData.rampTime             = millisecond_t(500);
    controlData.controlType          = ControlData::controlType_t::Line;
    controlData.lineControl.baseline = mainLine.centerLine;
    controlData.lineControl.offset   = millimeter_t(0);
    controlData.lineControl.angle    = radian_t(0);
    return controlData;
}

ControlData getControl_Fast1(const LinePattern&, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonFast(mainLine);

    return controlData;
}

ControlData getControl_Slow1_prepare(const LinePattern& pattern, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(pattern, mainLine);

    controlData.speed = getSpeeds(currentSeg.lap).slow1_prepare;

    return controlData;
}

ControlData getControl_Slow1_round(const LinePattern& pattern, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(pattern, mainLine);

    controlData.speed = getSpeeds(currentSeg.lap).slow1_round;

    return controlData;
}

ControlData getControl_Fast2(const LinePattern&, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonFast(mainLine);

    return controlData;
}

ControlData getControl_Slow2_prepare(const LinePattern& pattern, const MainLine& mainLine) {
    ControlData controlData   = getControl_CommonSlow(pattern, mainLine);

    controlData.speed = getSpeeds(currentSeg.lap).slow2_begin;

    return controlData;
}


ControlData getControl_Slow2_begin(const LinePattern& pattern, const MainLine& mainLine) {
    ControlData controlData   = getControl_CommonSlow(pattern, mainLine);

    controlData.speed = getSpeeds(currentSeg.lap).slow2_begin;

    return controlData;
}

ControlData getControl_Slow2_round(const LinePattern& pattern, const MainLine& mainLine) {
    ControlData controlData   = getControl_CommonSlow(pattern, mainLine);

    controlData.speed = getSpeeds(currentSeg.lap).slow2_round;

    return controlData;
}

ControlData getControl_Fast3(const LinePattern&, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonFast(mainLine);

    return controlData;
}

ControlData getControl_Slow3_prepare(const LinePattern& pattern, const MainLine& mainLine) {
    ControlData controlData   = getControl_CommonSlow(pattern, mainLine);

    controlData.speed = getSpeeds(currentSeg.lap).slow3_round;

    return controlData;
}

ControlData getControl_Slow3_round(const LinePattern& pattern, const MainLine& mainLine) {
    ControlData controlData   = getControl_CommonSlow(pattern, mainLine);

    controlData.speed = getSpeeds(currentSeg.lap).slow3_round;

    return controlData;
}

ControlData getControl_Slow3_end(const LinePattern& pattern, const MainLine& mainLine) {
    ControlData controlData   = getControl_CommonSlow(pattern, mainLine);

    controlData.speed = getSpeeds(currentSeg.lap).slow3_end;

    return controlData;
}

ControlData getControl_Fast4(const LinePattern&, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonFast(mainLine);

    return controlData;
}

ControlData getControl_Slow4_prepare(const LinePattern& pattern, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(pattern, mainLine);

    controlData.speed = getSpeeds(currentSeg.lap).slow4_prepare;

    return controlData;
}

ControlData getControl_Slow4_round(const LinePattern& pattern, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(pattern, mainLine);

    controlData.speed = getSpeeds(currentSeg.lap).slow4_round;

    return controlData;
}

const TrackSegments trackSegments = {
    { true,  hasBecomeActive_Fast,          getControl_Fast1         },
    { false, hasBecomeActive_Slow1_prepare, getControl_Slow1_prepare },
    { false, hasBecomeActive_Slow1,         getControl_Slow1_round   },
    { true,  hasBecomeActive_Fast,          getControl_Fast2         },
    { false, hasBecomeActive_Slow2_prepare, getControl_Slow2_prepare },
    { false, hasBecomeActive_Slow2_begin,   getControl_Slow2_begin   },
    { false, hasBecomeActive_Slow2_round,   getControl_Slow2_round   },
    { true,  hasBecomeActive_Fast,          getControl_Fast3         },
    { false, hasBecomeActive_Slow3_prepare, getControl_Slow3_prepare },
    { false, hasBecomeActive_Slow3_round,   getControl_Slow3_round   },
    { false, hasBecomeActive_Slow3_end,     getControl_Slow3_end     },
    { true,  hasBecomeActive_Fast,          getControl_Fast4         },
    { false, hasBecomeActive_Slow4_prepare, getControl_Slow4_prepare },
    { false, hasBecomeActive_Slow4,         getControl_Slow4_round   }
};

TrackSegments::const_iterator nextSegment(const TrackSegments::const_iterator currentSeg) {
    return trackSegments.back() == currentSeg ? trackSegments.begin() : currentSeg + 1;
}

m_per_sec_t safetyCarFollowSpeed(meter_t frontDist, bool isFast) {
    return map(frontDist.get(), meter_t(0.3f).get(), meter_t(0.8f).get(), m_per_sec_t(0), isFast ? MAX_SPEED_SAFETY_CAR_FAST : MAX_SPEED_SAFETY_CAR_SLOW);
}

bool overtakeSafetyCar(const DetectedLines& detectedLines, ControlData& controlData) {

    static constexpr meter_t OVERTAKE_SECTION_LENGTH = centimeter_t(900) - centimeter_t(250);

    static constexpr meter_t ORIENTATION_FILTER_DIST = centimeter_t(30);
    static constexpr meter_t BEGIN_SINE_ARC_LENGTH   = centimeter_t(150);
    static constexpr meter_t ACCELERATION_LENGTH     = centimeter_t(50);
    static constexpr meter_t BRAKE_LENGTH            = centimeter_t(30);
    static constexpr meter_t END_SINE_ARC_LENGTH     = centimeter_t(150);
    static constexpr meter_t FAST_SECTION_LENGTH     = OVERTAKE_SECTION_LENGTH - ORIENTATION_FILTER_DIST - BEGIN_SINE_ARC_LENGTH - ACCELERATION_LENGTH - BRAKE_LENGTH - END_SINE_ARC_LENGTH;

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
                globals::car.pose,
                clamp(globals::car.speed, m_per_sec_t(1.0f), globals::speed_OVERTAKE_BEGIN)
            }, globals::car.distance);

            overtake.trajectory.appendSineArc(Trajectory::config_t{
                Pose{
                    overtake.trajectory.lastConfig().pose.pos + vec2m{ BEGIN_SINE_ARC_LENGTH, globals::dist_OVERTAKE_SIDE }.rotate(overtake.orientation),
                    overtake.orientation
                },
                globals::speed_OVERTAKE_BEGIN
            }, globals::car.pose.angle, Trajectory::orientationUpdate_t::FIX_ORIENTATION, 50, radian_t(0), PI);

            overtake.trajectory.appendLine(Trajectory::config_t{
                Pose{
                    overtake.trajectory.lastConfig().pose.pos + vec2m{ ACCELERATION_LENGTH, centimeter_t(0) }.rotate(overtake.orientation),
                    overtake.orientation
                },
                globals::speed_OVERTAKE_STRAIGHT
            });

            for (uint8_t i = 0; i < 10; ++i) {
                overtake.trajectory.appendLine(Trajectory::config_t{
                    Pose{
                        overtake.trajectory.lastConfig().pose.pos + vec2m{ FAST_SECTION_LENGTH / 10, centimeter_t(0) }.rotate(overtake.orientation),
                        overtake.orientation
                    },
                    globals::speed_OVERTAKE_STRAIGHT
                });
            }

            overtake.trajectory.appendLine(Trajectory::config_t{
                Pose{
                    overtake.trajectory.lastConfig().pose.pos + vec2m{ BRAKE_LENGTH, centimeter_t(0) }.rotate(overtake.orientation),
                    overtake.orientation
                },
                globals::speed_OVERTAKE_END
            });

            overtake.trajectory.appendSineArc(Trajectory::config_t{
                Pose{
                    overtake.trajectory.lastConfig().pose.pos + vec2m{ END_SINE_ARC_LENGTH, -globals::dist_OVERTAKE_SIDE }.rotate(overtake.orientation),
                    overtake.orientation
                },
                globals::speed_OVERTAKE_BEGIN
            }, globals::car.pose.angle, Trajectory::orientationUpdate_t::FIX_ORIENTATION, 50, radian_t(0), PI_2);
        }
    }

    if (overtake.trajectory.length() > meter_t(0)) {
        controlData = overtake.trajectory.update(globals::car);

        finished = overtake.trajectory.length() - overtake.trajectory.coveredDistance() < centimeter_t(30) && LinePattern::NONE != detectedLines.front.pattern.type;
        if (finished) {
            overtake.trajectory.clear();
            overtake.startDist = meter_t(0);
        }
    }

    return finished;
}

} // namespace

extern "C" void runProgRaceTrackTask(void) {

    micro::waitReady(detectedLinesQueue);
    micro::waitReady(controlQueue);
    micro::waitReady(distancesQueue);

    DetectedLines prevDetectedLines, detectedLines;
    ControlData controlData;
    DistancesData distances;

    MainLine mainLine(cfg::CAR_FRONT_REAR_SENSOR_ROW_DIST);

    overtake.segment = trackSegments.begin() + 4;

    meter_t lastDistWithValidLine;
    millisecond_t lapStartTime;

    while (true) {
        switch(getActiveTask(globals::programState)) {
        case ProgramTask::RaceTrack:
        {
            static const bool runOnce = [&lastDistWithValidLine, &lapStartTime]() {

                if (ProgramState::Race          == globals::programState ||
                    ProgramState::Race_segFast2 == globals::programState ||
                    ProgramState::Race_segFast3 == globals::programState ||
                    ProgramState::Race_segFast4 == globals::programState) {

                    currentSeg.lap = 3;
                    currentSeg.trackSeg = ProgramState::Race == globals::programState ? trackSegments.begin() :
                                 ProgramState::Race_segFast2 == globals::programState ? trackSegments.begin() + 2 :
                                 ProgramState::Race_segFast3 == globals::programState ? trackSegments.begin() + 4 :
                                 trackSegments.begin() + 6;

                    globals::programState = ProgramState::Race;
                    forceSlowSpeed = true;
                } else {
                    currentSeg.lap = 1;
                    currentSeg.trackSeg = trackSegments.begin();
                    forceSlowSpeed = false;
                }

                lastDistWithValidLine = globals::car.distance;
                lapStartTime = getTime();

                currentSeg.segStartCarProps = globals::car;
                prevCarProps.push_back(currentSeg.segStartCarProps);
                forceInstantBrake = true;
                return true;
            }();
            UNUSED(runOnce);

            xQueuePeek(detectedLinesQueue, &detectedLines, 0);
            xQueuePeek(distancesQueue, &distances, 0);

            micro::updateMainLine(detectedLines.front.lines, detectedLines.rear.lines, mainLine, globals::car.speed >= m_per_sec_t(0));

            if (globals::car.distance - prevCarProps.peek_back(0).distance >= PREV_CAR_PROPS_RESOLUTION) {
                prevCarProps.push_back(globals::car);
            }

            if (LinePattern::NONE != detectedLines.front.pattern.type) {
                lastDistWithValidLine = globals::car.distance;
            }

            TrackSegments::const_iterator nextSeg = nextSegment(currentSeg.trackSeg);
            if (nextSeg->hasBecomeActive(detectedLines.front.pattern, mainLine)) {
                forceSlowSpeed = false;
                currentSeg.trackSeg = nextSeg;
                currentSeg.segStartCarProps = globals::car;
                if (trackSegments.begin() == currentSeg.trackSeg) {
                    LOG_INFO("Lap %d finished (time: %f seconds)", static_cast<int32_t>(currentSeg.lap), static_cast<second_t>(getTime() - lapStartTime).get());
                    ++currentSeg.lap;
                    lapStartTime = getTime();
                }
                LOG_INFO("Segment %d became active (lap: %d)", static_cast<int32_t>(currentSeg.trackSeg - trackSegments.begin()), static_cast<int32_t>(currentSeg.lap));
            }

            switch (globals::programState) {
            case ProgramState::ReachSafetyCar:
                controlData.speed = globals::speed_REACH_SAFETY_CAR;
                controlData.rampTime = millisecond_t(0);
                forceInstantBrake = true;
                if (distances.front > meter_t(0) && safetyCarFollowSpeed(distances.front, true) < controlData.speed) {
                    globals::programState = ProgramState::FollowSafetyCar;
                    LOG_DEBUG("Reached safety car, starts following");
                }
                break;

            case ProgramState::FollowSafetyCar:
                static meter_t lastDistWithSafetyCar;
                controlData.speed = safetyCarFollowSpeed(distances.front, currentSeg.trackSeg->isFast);
                controlData.rampTime = millisecond_t(0);
                forceInstantBrake = true;
                globals::distServoEnabled = true;

                if (distances.front < centimeter_t(100)) {
                    lastDistWithSafetyCar = globals::car.distance;
                }

                if (overtake.segment == currentSeg.trackSeg && ((0 == overtake.cntr && 1 == currentSeg.lap) || (1 == overtake.cntr && 3 == currentSeg.lap))) {
                    globals::programState = ProgramState::OvertakeSafetyCar;
                    LOG_DEBUG("Starts overtake");
                } else if ((trackSegments.begin() == currentSeg.trackSeg && currentSeg.lap > 3) || globals::car.distance - lastDistWithSafetyCar > centimeter_t(80)) {
                    globals::programState = ProgramState::Race;
                    LOG_DEBUG("Safety car left the track, starts race");
                }
                break;

            case ProgramState::OvertakeSafetyCar:
                controlData.speed = safetyCarFollowSpeed(distances.front, currentSeg.trackSeg->isFast);
                forceInstantBrake = true;
                if (overtakeSafetyCar(detectedLines, controlData)) {
                    ++overtake.cntr;
                    globals::programState = ProgramState::Race;
                    LOG_DEBUG("Overtake finished, starts race");
                }
                break;

            case ProgramState::Race:
                forceInstantBrake = currentSeg.lap < 4 && 2 != currentSeg.lap;
                if (overtake.segment == currentSeg.trackSeg && (1 == currentSeg.lap || 3 == currentSeg.lap)) {
                    forceSlowSpeed = true;
                }

                controlData = currentSeg.trackSeg->getControl(detectedLines.front.pattern, mainLine);
                if (currentSeg.lap > NUM_LAPS) {
                    globals::programState = ProgramState::Finish;
                    LOG_DEBUG("Race finished");
                } else if (currentSeg.lap <= 3 && overtake.cntr < 2 && distances.front < (currentSeg.trackSeg->isFast ? centimeter_t(120) : centimeter_t(60))) {
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
                if (globals::car.distance - startDist < meter_t(2.0f)) {
                    controlData = currentSeg.trackSeg->getControl(detectedLines.front.pattern, mainLine);
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
