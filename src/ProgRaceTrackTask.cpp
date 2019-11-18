#include <cfg_board.h>
#include <micro/task/common.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/updatable.hpp>
#include <micro/hw/SteeringServo.hpp>
#include <micro/sensor/Filter.hpp>
#include <micro/control/LineController.hpp>
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
    ProgSubCntr_FollowSafetyCar = 0,
    ProgSubCntr_Overtake        = 1,
    ProgSubCntr_ReachSafetyCar  = 2,
    ProgSubCntr_Race            = 3
};

constexpr m_per_sec_t speed_FAST = m_per_sec_t(1.5f);
constexpr m_per_sec_t speed_SLOW = m_per_sec_t(1.0f);

constexpr m_per_sec_t maxSpeed_SAFETY_CAR_FAST = m_per_sec_t(1.6f);
constexpr m_per_sec_t maxSpeed_SAFETY_CAR_SLOW = m_per_sec_t(1.2f);

struct Overtake {
    Pose startPose;

};

} // namespace

extern "C" void runProgRaceTrackTask(const void *argument) {
    distancesQueue = xQueueCreateStatic(DISTANCES_QUEUE_LENGTH, sizeof(DistancesData), distancesQueueStorageBuffer, &distancesQueueBuffer);

    vTaskDelay(10); // gives time to other tasks to wake up

    DetectedLines prevDetectedLines, detectedLines;
    ControlData controlData;
    DistancesData distances;

    bool isFastSection = false;

    while (true) {
        switch(globals::programState.activeModule()) {
        case ProgramState::ActiveModule::RaceTrack:
        {
            controlData.baseline = detectedLines.mainLine;
            xQueuePeek(detectedLinesQueue, &detectedLines, 0);
            xQueuePeek(distancesQueue, &distances, 0);

            if (detectedLines.pattern.type != prevDetectedLines.pattern.type && LinePattern::SINGLE_LINE == prevDetectedLines.pattern.type) {
                if (LinePattern::ACCELERATE == detectedLines.pattern.type) {
                    isFastSection = true;
                } else if (LinePattern::BRAKE == detectedLines.pattern.type) {
                    isFastSection = false;
                }
            }

            switch (globals::programState.subCntr()) {
            case ProgSubCntr_FollowSafetyCar:
                controlData.speed = map(distances.front.get(), meter_t(0.2f).get(), meter_t(0.7f).get(), m_per_sec_t(0),
                    isFastSection ? maxSpeed_SAFETY_CAR_FAST : maxSpeed_SAFETY_CAR_SLOW);
                break;
            case ProgSubCntr_Overtake:
                break;
            case ProgSubCntr_ReachSafetyCar:
                break;
            case ProgSubCntr_Race:
                controlData.speed = isFastSection ? speed_FAST : speed_SLOW;
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
