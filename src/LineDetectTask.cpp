#include <micro/panel/CanManager.hpp>
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

} // namespace

extern "C" void runLineDetectTask(void) {

    detectedLinesQueue = xQueueCreateStatic(DETECTED_LINES_QUEUE_LENGTH, sizeof(DetectedLines), detectedLinesQueueStorageBuffer, &detectedLinesQueueBuffer);

    DetectedLines detectedLines;

    CanManager canManager(can_Vehicle, canRxFifo_Vehicle, millisecond_t(15));

    canManager.registerHandler(can::FrontLines::id(), [&detectedLines] (const uint8_t * const data) {
        reinterpret_cast<const can::FrontLines*>(data)->acquire(detectedLines.front.lines);
        xQueueOverwrite(detectedLinesQueue, &detectedLines);
    });

    canManager.registerHandler(can::RearLines::id(), [&detectedLines] (const uint8_t * const data) {
        reinterpret_cast<const can::RearLines*>(data)->acquire(detectedLines.rear.lines);
    });

    canManager.registerHandler(can::FrontLinePattern::id(), [&detectedLines] (const uint8_t * const data) {
        reinterpret_cast<const can::FrontLinePattern*>(data)->acquire(detectedLines.front.pattern);
    });

    canManager.registerHandler(can::RearLinePattern::id(), [&detectedLines] (const uint8_t * const data) {
        reinterpret_cast<const can::RearLinePattern*>(data)->acquire(detectedLines.rear.pattern);
    });

    Timer lineDetectControlTimer(can::LineDetectControl::period());

    while (true) {

        globals::isLineDetectTaskOk = !canManager.hasRxTimedOut();

        canManager.handleIncomingFrames();

        if (lineDetectControlTimer.checkTimeout()) {
            const bool isFwd                     = globals::car.speed >= m_per_sec_t(0);
            const bool isRace                    = ProgramTask::RaceTrack == getActiveTask(globals::programState);
            const linePatternDomain_t domain     = isRace ? linePatternDomain_t::Race : linePatternDomain_t::Labyrinth;
            const bool isReducedScanRangeEnabled = isRace && ((isFwd && detectedLines.front.lines.size()) || (!isFwd && detectedLines.rear.lines.size()));
            const uint8_t scanRangeRadius        = isReducedScanRangeEnabled ? globals::reducedLineDetectScanRangeRadius : 0;

            canManager.send(can::LineDetectControl(globals::indicatorLedsEnabled, scanRangeRadius, domain));
        }

        vTaskDelay(1);
    }

    vTaskDelete(nullptr);
}
