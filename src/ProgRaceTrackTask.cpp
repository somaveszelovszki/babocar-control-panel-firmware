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

#define DISTANCES_QUEUE_LENGTH 1
QueueHandle_t distancesQueue;
static uint8_t distancesQueueStorageBuffer[DISTANCES_QUEUE_LENGTH * sizeof(DistancesData)];
static StaticQueue_t distancesQueueBuffer;

namespace {

constexpr m_per_sec_t maxSpeed_SAFETY_CAR_FAST = m_per_sec_t(1.6f);
constexpr m_per_sec_t maxSpeed_SAFETY_CAR_SLOW = m_per_sec_t(1.2f);

struct TrackSegment {
    enum Type {
        Slow,
        Fast
    };

    Type type;
    Direction dir;
    meter_t startDist;
    meter_t endDist;
};

TrackSegment trackSegments[] = {
    { TrackSegment::Fast, Direction::CENTER, meter_t(0), meter_t(0) },
    { TrackSegment::Slow, Direction::LEFT,   meter_t(0), meter_t(0) },
    { TrackSegment::Fast, Direction::CENTER, meter_t(0), meter_t(0) },
    { TrackSegment::Slow, Direction::RIGHT,  meter_t(0), meter_t(0) },
    { TrackSegment::Slow, Direction::LEFT,   meter_t(0), meter_t(0) },
    { TrackSegment::Fast, Direction::CENTER, meter_t(0), meter_t(0) },
    { TrackSegment::Slow, Direction::RIGHT,  meter_t(0), meter_t(0) },
    { TrackSegment::Slow, Direction::LEFT,   meter_t(0), meter_t(0) },
    { TrackSegment::Slow, Direction::CENTER, meter_t(0), meter_t(0) },
    { TrackSegment::Slow, Direction::LEFT,   meter_t(0), meter_t(0) }
};

struct {
    radian_t sectionOrientation = radian_t(0);
    Trajectory trajectory       = Trajectory(cfg::CAR_OPTO_CENTER_DIST);
} overtake;

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
        });

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
    distancesQueue = xQueueCreateStatic(DISTANCES_QUEUE_LENGTH, sizeof(DistancesData), distancesQueueStorageBuffer, &distancesQueueBuffer);

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
            xQueuePeek(detectedLinesQueue, &detectedLines, 0);
            xQueuePeek(distancesQueue, &distances, 0);

            controlData.directControl = false;
            LineCalculator::updateMainLine(detectedLines.lines, controlData.baseline);
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

                if (safetyCarFollowSpeed(distances.front, false) < controlData.speed) {
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
                controlData.speed = isFastSection ? globals::speed_FAST : globals::speed_SLOW;
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
            vTaskDelay(100);
            break;
        }
    }

    vTaskDelete(nullptr);
}
