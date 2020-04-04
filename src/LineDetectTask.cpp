#include <micro/panel/LineDetectPanelLinkData.hpp>
#include <micro/sensor/Filter.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/Line.hpp>

#include <cfg_board.h>
#include <cfg_car.hpp>
#include <DetectedLines.hpp>
#include <DistancesData.hpp>
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

PanelLink<LineDetectOutPanelLinkData, LineDetectInPanelLinkData> frontLineDetectPanelLink(panelLinkRole_t::Master, uart_FrontLineDetectPanel);
PanelLink<LineDetectOutPanelLinkData, LineDetectInPanelLinkData> rearLineDetectPanelLink(panelLinkRole_t::Master, uart_RearLineDetectPanel);

void fillLineDetectPanelData(LineDetectInPanelLinkData& txData, const bool indicatorLedsEnabled, const uint8_t scanRangeRadius, const linePatternDomain_t domain) {
    txData.indicatorLedsEnabled = indicatorLedsEnabled;
    txData.scanRangeRadius      = scanRangeRadius;
    txData.domain               = static_cast<uint8_t>(domain);
    txData.distance_mm          = static_cast<uint32_t>(static_cast<millimeter_t>(globals::car.distance).get());
}

void parseLineDetectPanelData(LineDetectOutPanelLinkData& rxData, Lines& lines, LinePattern& pattern, const bool mirror = false) {

    lines.clear();
    for (uint8_t i = 0; i < ARRAY_SIZE(rxData.lines); ++i) {
        if (rxData.lines[i].id != 0) {
            lines.insert(Line{ millimeter_t(rxData.lines[i].pos_mm_per16 / 16.0f) * (mirror ? -1 : 1), rxData.lines[i].id });
        } else {
            break;
        }
    }

    pattern.type      = static_cast<LinePattern::type_t>(rxData.pattern.type);
    pattern.dir       = static_cast<Sign>(rxData.pattern.dir);
    pattern.side      = static_cast<Direction>(rxData.pattern.side);
    pattern.startDist = millimeter_t(rxData.pattern.startDist_mm);
}

} // namespace

extern "C" void runLineDetectTask(void) {

    detectedLinesQueue = xQueueCreateStatic(DETECTED_LINES_QUEUE_LENGTH, sizeof(DetectedLines), detectedLinesQueueStorageBuffer, &detectedLinesQueueBuffer);

    vTaskDelay(10); // gives time to other tasks to wake up

    LineDetectOutPanelLinkData rxDataFront, rxDataRear;
    LineDetectInPanelLinkData txDataFront, txDataRear;
    DetectedLines detectedLines;
    Line mainLine;

    while (true) {
        frontLineDetectPanelLink.update();
        rearLineDetectPanelLink.update();

        globals::isLineDetectTaskOk = frontLineDetectPanelLink.isConnected() && rearLineDetectPanelLink.isConnected();

        const bool isFwd = globals::car.speed >= m_per_sec_t(0);
        const ProgramTask task = getActiveTask(globals::programState);
        const linePatternDomain_t domain = ProgramTask::RaceTrack == task ? linePatternDomain_t::Race : linePatternDomain_t::Labyrinth;
        const bool isReducedScanRangeEnabled = ProgramTask::RaceTrack == task && ((isFwd && detectedLines.front.lines.size()) || (!isFwd && detectedLines.rear.lines.size()));
        const uint8_t scanRangeRadius = isReducedScanRangeEnabled ? 10 : 0;

        if (frontLineDetectPanelLink.readAvailable(rxDataFront) && rearLineDetectPanelLink.readAvailable(rxDataRear)) {
            parseLineDetectPanelData(rxDataFront, detectedLines.front.lines, detectedLines.front.pattern, !isFwd);
            parseLineDetectPanelData(rxDataRear, detectedLines.rear.lines, detectedLines.rear.pattern, isFwd);
            xQueueOverwrite(detectedLinesQueue, &detectedLines);
        }

        if (frontLineDetectPanelLink.shouldSend()) {
            fillLineDetectPanelData(txDataFront, globals::frontIndicatorLedsEnabled, scanRangeRadius, domain);
            frontLineDetectPanelLink.send(txDataFront);
        }

        if (rearLineDetectPanelLink.shouldSend()) {
            fillLineDetectPanelData(txDataRear, globals::rearIndicatorLedsEnabled, scanRangeRadius, domain);
            rearLineDetectPanelLink.send(txDataRear);
        }

        vTaskDelay(1);
    }

    vTaskDelete(nullptr);
}

/* @brief Callback for front line detect panel UART RxCplt - called when receive finishes.
 */
void micro_FrontLineDetectPanel_Uart_RxCpltCallback() {
    frontLineDetectPanelLink.onNewRxData();
}

/* @brief Callback for rear line detect panel UART RxCplt - called when receive finishes.
 */
void micro_RearLineDetectPanel_Uart_RxCpltCallback() {
    rearLineDetectPanelLink.onNewRxData();
}
