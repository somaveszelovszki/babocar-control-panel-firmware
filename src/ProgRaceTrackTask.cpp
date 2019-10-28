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
#include <cfg_car.hpp>

#include <globals.hpp>

using namespace micro;

extern QueueHandle_t detectedLinesQueue;
extern QueueHandle_t controlQueue;

namespace {

constexpr m_per_sec_t speed_FAST = m_per_sec_t(1.5f);
constexpr m_per_sec_t speed_SLOW = m_per_sec_t(1.0f);

} // namespace

extern "C" void runProgRaceTrackTask(const void *argument) {

    vTaskDelay(5); // gives time to other tasks to initialize their queues

    DetectedLines prevDetectedLines, detectedLines;
    ControlData controlData;

    while(true) {
        switch(globals::programState.activeModule()) {
        case ProgramState::ActiveModule::RaceTrack:
        {
            if (xQueuePeek(detectedLinesQueue, &detectedLines, 0)) {

                controlData.baseline = detectedLines.mainLine;
                controlData.baselineOffset = millimeter_t(0);
                controlData.baselineAngle = radian_t(0);

                if (detectedLines.pattern.type != prevDetectedLines.pattern.type) {
                    if (LinePattern::SINGLE_LINE == prevDetectedLines.pattern.type) {
                        if (LinePattern::ACCELERATE == detectedLines.pattern.type) {
                            controlData.speed = speed_FAST;
                        } else if (LinePattern::BRAKE == detectedLines.pattern.type) {
                            controlData.speed = speed_SLOW;
                        }
                    }
                }

                xQueueOverwrite(controlQueue, &controlData);

                prevDetectedLines = detectedLines;
            }

            switch(globals::programState.subCntr()) {
            case 0: break; // TODO
            default:
                LOG_ERROR("Invalid program state counter: [%u]", globals::programState.subCntr());
                globals::programState.set(ProgramState::ActiveModule::INVALID, 0);
                break;
            }

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
