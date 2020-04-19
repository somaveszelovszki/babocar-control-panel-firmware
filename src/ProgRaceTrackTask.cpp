#include <micro/container/infinite_buffer.hpp>
#include <micro/hw/SteeringServo.hpp>
#include <micro/sensor/Filter.hpp>
#include <micro/utils/ControlData.hpp>
#include <micro/utils/LinePattern.hpp>
#include <micro/utils/timer.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/trajectory.hpp>
#include <micro/utils/updatable.hpp>

#include <cfg_board.h>
#include <cfg_car.hpp>
#include <cfg_track.hpp>
#include <DetectedLines.hpp>
#include <DistancesData.hpp>
#include <globals.hpp>
#include <track.hpp>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <track.hpp>

using namespace micro;

extern QueueHandle_t detectedLinesQueue;
extern QueueHandle_t controlQueue;
extern QueueHandle_t distancesQueue;

namespace {

constexpr m_per_sec_t MAX_SPEED_SAFETY_CAR_SLOW = m_per_sec_t(1.3f);
constexpr m_per_sec_t MAX_SPEED_SAFETY_CAR_FAST = m_per_sec_t(1.8f);
constexpr meter_t PREV_CAR_PROPS_RESOLUTION     = centimeter_t(5);

struct {
    TrackSegments::const_iterator segment;
    meter_t startDist;
    radian_t orientation;
    Trajectory trajectory;
    uint8_t cntr = 0;
} overtake;

infinite_buffer<CarProps, static_cast<uint32_t>(meter_t(5) / PREV_CAR_PROPS_RESOLUTION)> prevCarProps;

bool forceSlowSpeed    = false;
Sign speedSign         = Sign::POSITIVE;

TrackSegments::const_iterator nextSegment(const TrackSegments::const_iterator currentSeg) {
    return trackSegments.back() == currentSeg ? trackSegments.begin() : currentSeg + 1;
}

m_per_sec_t safetyCarFollowSpeed(meter_t frontDist, bool isFast) {
    return speedSign * map(frontDist.get(), meter_t(0.3f).get(), meter_t(0.8f).get(), m_per_sec_t(0), isFast ? MAX_SPEED_SAFETY_CAR_FAST : MAX_SPEED_SAFETY_CAR_SLOW);
}

bool overtakeSafetyCar(const DetectedLines& detectedLines, ControlData& controlData) {

    static constexpr meter_t OVERTAKE_SECTION_LENGTH = centimeter_t(700);
    static constexpr meter_t ORIENTATION_FILTER_DIST = centimeter_t(50);
    static constexpr meter_t BEGIN_SINE_ARC_LENGTH   = centimeter_t(180);
    static constexpr meter_t END_SINE_ARC_LENGTH     = centimeter_t(150);
    static constexpr meter_t FAST_SECTION_MAX_LENGTH = OVERTAKE_SECTION_LENGTH - ORIENTATION_FILTER_DIST - BEGIN_SINE_ARC_LENGTH - END_SINE_ARC_LENGTH;

    bool finished = false;

    if (overtake.trajectory.length() == meter_t(0)) {

        if (overtake.startDist == meter_t(0)) {
            overtake.startDist = globals::car.distance;
        }

        const meter_t overtakeDist = globals::car.distance - overtake.startDist;

        if (overtakeDist > ORIENTATION_FILTER_DIST) {

            const point2m posDiff           = globals::car.pose.pos - prevCarProps.peek_back(static_cast<uint32_t>(overtakeDist / PREV_CAR_PROPS_RESOLUTION)).pose.pos;
            const meter_t fastSectionLength = FAST_SECTION_MAX_LENGTH - overtakeDist;
            overtake.orientation            = posDiff.getAngle();

            overtake.trajectory.setStartConfig(Trajectory::config_t{
                Pose{ globals::car.pose.pos, overtake.orientation },
                clamp(globals::car.speed, m_per_sec_t(1.0f), globals::speed_OVERTAKE_BEGIN)
            }, globals::car.distance);

            overtake.trajectory.appendSineArc(Trajectory::config_t{
                Pose{
                    overtake.trajectory.lastConfig().pose.pos + vec2m{ BEGIN_SINE_ARC_LENGTH, globals::dist_OVERTAKE_SIDE }.rotate(overtake.orientation),
                    overtake.orientation
                },
                globals::speed_OVERTAKE_BEGIN
            }, globals::car.pose.angle, Trajectory::orientationUpdate_t::FIX_ORIENTATION, 50, radian_t(0), PI);

            // TODO this should work without the for cycle
            for (uint8_t i = 0; i < 10; ++i) {
                overtake.trajectory.appendLine(Trajectory::config_t{
                    Pose{
                        overtake.trajectory.lastConfig().pose.pos + vec2m{ fastSectionLength / 10, centimeter_t(0) }.rotate(overtake.orientation),
                        overtake.orientation
                    },
                    globals::speed_OVERTAKE_STRAIGHT
                });
            }

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

    TrackInfo trackInfo;

    overtake.segment = trackSegments.begin() + 4;

    meter_t lastDistWithValidLine;
    millisecond_t lapStartTime;

    while (true) {
        switch(getActiveTask(globals::programState)) {
        case ProgramTask::RaceTrack:
        {
            static const bool runOnce = [&trackInfo, &lastDistWithValidLine, &lapStartTime, &mainLine]() {

                if (ProgramState::Race          == globals::programState ||
                    ProgramState::Race_segFast2 == globals::programState ||
                    ProgramState::Race_segFast3 == globals::programState ||
                    ProgramState::Race_segFast4 == globals::programState) {

                    trackInfo.lap = 3;
                    trackInfo.seg = ProgramState::Race == globals::programState ? trackSegments.begin() :
                           ProgramState::Race_segFast2 == globals::programState ? trackSegments.begin() + 2 :
                           ProgramState::Race_segFast3 == globals::programState ? trackSegments.begin() + 4 :
                                                                                  trackSegments.begin() + 6;

                    globals::programState = ProgramState::Race;
                    forceSlowSpeed = true;
                } else {
                    trackInfo.lap = 1;
                    trackInfo.seg = trackSegments.begin();
                    forceSlowSpeed = false;
                }

                lastDistWithValidLine = globals::car.distance;
                lapStartTime = getTime();

                trackInfo.segStartCarProps = globals::car;
                trackInfo.segStartLine = mainLine.centerLine;
                prevCarProps.push_back(trackInfo.segStartCarProps);
                return true;
            }();
            UNUSED(runOnce);

            xQueuePeek(detectedLinesQueue, &detectedLines, 0);
            xQueuePeek(distancesQueue, &distances, 0);

            micro::updateMainLine(detectedLines.front.lines, detectedLines.rear.lines, mainLine, globals::car.speed >= m_per_sec_t(0));

            // sets default lateral control
            controlData.controlType          = ControlData::controlType_t::Line;
            controlData.lineControl.baseline = mainLine.centerLine;
            controlData.lineControl.offset   = millimeter_t(0);
            controlData.lineControl.angle    = radian_t(0);

            if (globals::car.distance - prevCarProps.peek_back(0).distance >= PREV_CAR_PROPS_RESOLUTION) {
                prevCarProps.push_back(globals::car);
            }

            if (LinePattern::NONE != detectedLines.front.pattern.type) {
                lastDistWithValidLine = globals::car.distance;
            }

            TrackSegments::const_iterator nextSeg = nextSegment(trackInfo.seg);
            if (nextSeg->hasBecomeActive(trackInfo, detectedLines.front.pattern)) {
                forceSlowSpeed = false;
                trackInfo.seg = nextSeg;
                trackInfo.segStartCarProps = globals::car;
                trackInfo.segStartLine = mainLine.centerLine;

                if (trackSegments.begin() == trackInfo.seg) {
                    LOG_INFO("Lap %d finished (time: %f seconds)", static_cast<int32_t>(trackInfo.lap), static_cast<second_t>(getTime() - lapStartTime).get());
                    ++trackInfo.lap;
                    lapStartTime = getTime();
                }
                LOG_INFO("Segment %d became active (lap: %d)", static_cast<int32_t>(trackInfo.seg - trackSegments.begin()), static_cast<int32_t>(trackInfo.lap));
            }

            switch (globals::programState) {
            case ProgramState::ReachSafetyCar:
                controlData.speed = globals::speed_REACH_SAFETY_CAR;
                controlData.rampTime = millisecond_t(0);
                if (distances.front > meter_t(0) && safetyCarFollowSpeed(distances.front, true) < controlData.speed) {
                    globals::programState = ProgramState::FollowSafetyCar;
                    LOG_DEBUG("Reached safety car, starts following");
                }
                break;

            case ProgramState::FollowSafetyCar:
                static meter_t lastDistWithSafetyCar;
                controlData.speed = safetyCarFollowSpeed(distances.front, trackInfo.seg->isFast);
                controlData.rampTime = millisecond_t(0);
                globals::distServoEnabled = true;

                if (distances.front < centimeter_t(100)) {
                    lastDistWithSafetyCar = globals::car.distance;
                }

                if (overtake.segment == trackInfo.seg && ((0 == overtake.cntr && 1 == trackInfo.lap) || (1 == overtake.cntr && 3 == trackInfo.lap))) {
                    globals::programState = ProgramState::OvertakeSafetyCar;
                    LOG_DEBUG("Starts overtake");
                } else if ((trackSegments.begin() == trackInfo.seg && trackInfo.lap > 3) || globals::car.distance - lastDistWithSafetyCar > centimeter_t(80)) {
                    globals::programState = ProgramState::Race;
                    LOG_DEBUG("Safety car left the track, starts race");
                }
                break;

            case ProgramState::OvertakeSafetyCar:
                controlData.speed = safetyCarFollowSpeed(distances.front, trackInfo.seg->isFast);
                if (overtakeSafetyCar(detectedLines, controlData)) {
                    ++overtake.cntr;
                    globals::programState = ProgramState::Race;
                    LOG_DEBUG("Overtake finished, starts race");
                }
                break;

            case ProgramState::Race:
                if (overtake.segment == trackInfo.seg && (1 == trackInfo.lap || 3 == trackInfo.lap)) {
                    forceSlowSpeed = true;
                }

                controlData = trackInfo.seg->getControl(trackInfo, mainLine);
                if (trackInfo.lap > cfg::NUM_RACE_LAPS) {
                    globals::programState = ProgramState::Finish;
                    LOG_DEBUG("Race finished");
                } else if (trackInfo.lap <= 3 && overtake.cntr < 2 && distances.front < (trackInfo.seg->isFast ? centimeter_t(120) : centimeter_t(60))) {
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
                    controlData = trackInfo.seg->getControl(trackInfo, mainLine);
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
                controlData.speed = min(controlData.speed, m_per_sec_t(1.8f));
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
