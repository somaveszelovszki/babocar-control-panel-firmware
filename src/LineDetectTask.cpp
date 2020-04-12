#include <micro/panel/vehicleCanTypes.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/Line.hpp>
#include <micro/utils/timer.hpp>

#include <cfg_board.h>
#include <cfg_car.hpp>
#include <DetectedLines.hpp>
#include <globals.hpp>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

using namespace micro;

#define DETECTED_LINES_QUEUE_LENGTH 1
QueueHandle_t detectedLinesQueue;
static uint8_t detectedLinesQueueStorageBuffer[DETECTED_LINES_QUEUE_LENGTH * sizeof(DetectedLines)];
static StaticQueue_t detectedLinesQueueBuffer;

namespace {

DetectedLines detectedLines;

void parseVehicleCanData(const uint32_t id, const uint8_t * const data) {

    switch (id) {
    case can::FrontLines::id():
        reinterpret_cast<const can::FrontLines*>(data)->acquire(detectedLines.front.lines);
        break;

    case can::RearLines::id():
        reinterpret_cast<const can::RearLines*>(data)->acquire(detectedLines.rear.lines);
        break;

    case can::FrontLinePattern::id():
        reinterpret_cast<const can::FrontLinePattern*>(data)->acquire(detectedLines.front.pattern);
        break;

    case can::RearLinePattern::id():
        reinterpret_cast<const can::RearLinePattern*>(data)->acquire(detectedLines.rear.pattern);
        break;
    }
}

} // namespace

extern "C" void runLineDetectTask(void) {

    detectedLinesQueue = xQueueCreateStatic(DETECTED_LINES_QUEUE_LENGTH, sizeof(DetectedLines), detectedLinesQueueStorageBuffer, &detectedLinesQueueBuffer);

    vTaskDelay(10); // gives time to other tasks to wake up

    CAN_RxHeaderTypeDef rxHeader;
    alignas(8) uint8_t rxData[8];
    uint32_t txMailbox = 0;

    Timer lineDetectControlTimer(can::LineDetectControl::period());
    WatchdogTimer vehicleCanWatchdog(millisecond_t(15));

    while (true) {

        globals::isLineDetectTaskOk = !vehicleCanWatchdog.hasTimedOut();

        if (HAL_CAN_GetRxFifoFillLevel(can_Vehicle, canRxFifo_Vehicle)) {
            if (HAL_OK == HAL_CAN_GetRxMessage(can_Vehicle, canRxFifo_Vehicle, &rxHeader, rxData)) {
                parseVehicleCanData(rxHeader.StdId, rxData);
                vehicleCanWatchdog.reset();

                if (can::FrontLines::id() == rxHeader.StdId) {
                    xQueueOverwrite(detectedLinesQueue, &detectedLines);
                }
            }
        }

        if (lineDetectControlTimer.checkTimeout()) {
            const bool isFwd                     = globals::car.speed >= m_per_sec_t(0);
            const bool isRace                    = ProgramTask::RaceTrack == getActiveTask(globals::programState);
            const linePatternDomain_t domain     = isRace ? linePatternDomain_t::Race : linePatternDomain_t::Labyrinth;
            const bool isReducedScanRangeEnabled = isRace && ((isFwd && detectedLines.front.lines.size()) || (!isFwd && detectedLines.rear.lines.size()));
            const uint8_t scanRangeRadius        = isReducedScanRangeEnabled ? globals::reducedLineDetectScanRangeRadius : 0;

            CAN_TxHeaderTypeDef txHeader = micro::can::buildHeader<can::LineDetectControl>();
            can::LineDetectControl lineDetectControl(globals::indicatorLedsEnabled, scanRangeRadius, domain);
            HAL_CAN_AddTxMessage(can_Vehicle, &txHeader, reinterpret_cast<uint8_t*>(&lineDetectControl), &txMailbox);
        }

        vTaskDelay(1);
    }

    vTaskDelete(nullptr);
}
