#include <cfg_board.h>
#include <micro/task/common.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/updatable.hpp>
#include <micro/hw/SteeringServo.hpp>
#include <micro/sensor/Filter.hpp>
#include <micro/panel/LineDetectPanel.hpp>
#include <micro/panel/MotorPanel.hpp>
#include <micro/utils/Line.hpp>
#include <micro/utils/timer.hpp>

#include <DetectedLines.hpp>
#include <ControlData.hpp>
#include <DistancesData.hpp>
#include <cfg_car.hpp>

#include <globals.hpp>

using namespace micro;

extern QueueHandle_t detectedLinesQueue;
extern QueueHandle_t controlQueue;

#define DISTANCES_QUEUE_LENGTH 1
QueueHandle_t distancesQueue;
static uint8_t distancesQueueStorageBuffer[DISTANCES_QUEUE_LENGTH * sizeof(DistancesData)];
static StaticQueue_t distancesQueueBuffer;

namespace {

enum {
    ProgSubCntr_ReachSafetyCar  = 0,
    ProgSubCntr_FollowSafetyCar = 1,
    ProgSubCntr_Overtake        = 2,
    ProgSubCntr_Race            = 3
};

constexpr m_per_sec_t maxSpeed_SAFETY_CAR_FAST = m_per_sec_t(1.8f);
constexpr m_per_sec_t maxSpeed_SAFETY_CAR_SLOW = m_per_sec_t(1.4f);

struct Overtake {
    Pose startPose;

};

bool isSafe(const Line& line) {

    static millisecond_t unsafeSectionStartTime = millisecond_t(0);

    if (abs(line.angular_velocity) > deg_per_sec_t(30)) {
        unsafeSectionStartTime = getTime();
    }

    return getTime() - unsafeSectionStartTime > millisecond_t(1000);
}

m_per_sec_t calcSafetyCarSpeed(meter_t frontDist, bool isFastSection) {
    return map(frontDist.get(), meter_t(0.3f).get(), meter_t(0.8f).get(), m_per_sec_t(0),
        isFastSection ? maxSpeed_SAFETY_CAR_FAST : maxSpeed_SAFETY_CAR_SLOW);
}

} // namespace

extern "C" void runProgRaceTrackTask(const void *argument) {
    distancesQueue = xQueueCreateStatic(DISTANCES_QUEUE_LENGTH, sizeof(DistancesData), distancesQueueStorageBuffer, &distancesQueueBuffer);

    vTaskDelay(500); // gives time to other tasks to wake up

    DetectedLines prevDetectedLines, detectedLines;
    Line mainLine;
    ControlData controlData;
    DistancesData distances;

    bool isFastSection = false;

    meter_t startDist = globals::car.distance;
    meter_t sectionStartDist = startDist;
    meter_t lastDistWithActiveSafetyCar = startDist;

    while (true) {
        switch(globals::programState.activeModule()) {
        case ProgramState::ActiveModule::RaceTrack:
        {
            xQueuePeek(detectedLinesQueue, &detectedLines, 0);
            xQueuePeek(distancesQueue, &distances, 0);

            LineCalculator::updateMainLine(detectedLines.lines, mainLine);

            controlData.baseline = mainLine;
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

            switch (globals::programState.subCntr()) {
            case ProgSubCntr_ReachSafetyCar:
                controlData.speed = m_per_sec_t(0.75f);

                if (calcSafetyCarSpeed(distances.front, false) < controlData.speed) {
                    globals::programState.set(ProgramState::ActiveModule::RaceTrack, ProgSubCntr_FollowSafetyCar);
                }

//                if (distances.front < centimeter_t(60)) {
//                    globals::programState.set(ProgramState::ActiveModule::RaceTrack, ProgSubCntr_FollowSafetyCar);
//                }
                break;

            case ProgSubCntr_FollowSafetyCar:
                controlData.speed = calcSafetyCarSpeed(distances.front, isFastSection);

//                if (distances.front < meter_t(1.5f)) {
//                    lastDistWithActiveSafetyCar = globals::car.distance;
//                }
//
//                // when the safety car leaves the track (after a curve, before the fast signs),
//                if (isBtw(globals::car.distance - lastDistWithActiveSafetyCar, centimeter_t(50), centimeter_t(150)) &&
//                    isFastSection &&
//                    globals::car.distance - sectionStartDist < centimeter_t(5)) {
//
//                    globals::programState.set(ProgramState::ActiveModule::RaceTrack, ProgSubCntr_Race);
//                }

                break;

            case ProgSubCntr_Overtake:
                break;

            case ProgSubCntr_Race:
                if (isFastSection || (sectionStartDist != startDist && globals::car.distance - sectionStartDist < globals::slowSectionStartOffset)) {
                    if (!isFastSection && detectedLines.pattern.type == LinePattern::SINGLE_LINE) {

                        controlData.speed = globals::speed_SLOW;
                    } else if (abs(controlData.baseline.pos) < centimeter_t(2)) {
                        controlData.speed = globals::speed_FAST;
                    } else {
                        controlData.speed = globals::speed_SLOW;
                    }
                    //controlData.speed = isSafe(mainLine) ? globals::speed_FAST : globals::speed_FAST_UNSAFE;
                } else { // slow section
                    controlData.speed = globals::speed_SLOW;
                    //controlData.speed = isSafe(mainLine) ? globals::speed_SLOW : globals::speed_SLOW_UNSAFE;
                }
                break;

            default:
                LOG_ERROR("Invalid program state counter: [%u]", globals::programState.subCntr());
                globals::programState.set(ProgramState::ActiveModule::INVALID, 0);
                break;
            }

            xQueueOverwrite(controlQueue, &controlData);
            prevDetectedLines = detectedLines;

            vTaskDelay(1);
            break;
        }

        default:
            vTaskDelay(100);
            break;
        }
    }

    vTaskDelete(nullptr);
}
