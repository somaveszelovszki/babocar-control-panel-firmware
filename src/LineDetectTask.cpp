#include <micro/utils/log.hpp>
#include <micro/utils/Line.hpp>
#include <micro/utils/timer.hpp>
#include <micro/sensor/Filter.hpp>
#include <micro/panel/LineDetectPanelLink.hpp>
#include <micro/task/common.hpp>

#include <cfg_board.h>
#include <cfg_car.hpp>
#include <globals.hpp>
#include <ControlData.hpp>
#include <DistancesData.hpp>
#include <DetectedLines.hpp>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include <string.h>

using namespace micro;

#define DETECTED_LINES_QUEUE_LENGTH 1
QueueHandle_t detectedLinesQueue;
static uint8_t detectedLinesQueueStorageBuffer[DETECTED_LINES_QUEUE_LENGTH * sizeof(DetectedLines)];
static StaticQueue_t detectedLinesQueueBuffer;

static LineDetectPanelLink frontLineDetectPanelLink(
    uart_FrontLineDetectPanel,
    millisecond_t(LINE_DETECT_PANEL_LINK_OUT_TIMEOUT_MS),
    millisecond_t(LINE_DETECT_PANEL_LINK_IN_PERIOD_MS));

static LineCalculator lineCalc;
static LinePatternCalculator linePatternCalc;

static Line mainLine;

static void fillLineDetectPanelData(lineDetectPanelDataIn_t& txData) {
    txData.flags = 0x00;
    if (globals::indicatorLedsEnabled) txData.flags |= LINE_DETECT_PANEL_FLAG_INDICATOR_LEDS_ENABLED;
}

static void parseLineDetectPanelData(lineDetectPanelDataOut_t& rxData, Lines& lines, bool mirror = false) {
    lines.clear();

    for (uint8_t i = 0; i < MAX_NUM_LINES; ++i) {
        const trackedLine_t * const l = &rxData.values[i];
        if (l->id != INVALID_LINE_IDX) {
            lines.push_back(Line{ millimeter_t(l->pos_mm) * (mirror ? -1 : 1), l->id });
        } else {
            break;
        }
    }

    lines.removeDuplicates();
}

extern "C" void runLineDetectTask(const void *argument) {

    detectedLinesQueue = xQueueCreateStatic(DETECTED_LINES_QUEUE_LENGTH, sizeof(DetectedLines), detectedLinesQueueStorageBuffer, &detectedLinesQueueBuffer);

    vTaskDelay(10); // gives time to other tasks to wake up

    lineDetectPanelDataOut_t rxData;
    lineDetectPanelDataIn_t txData;
    Lines lines;

    while (true) {
        frontLineDetectPanelLink.update();
        globals::isLineDetectTaskOk = frontLineDetectPanelLink.isConnected();

        if (frontLineDetectPanelLink.readAvailable(rxData)) {

            if (globals::lineDetectionEnabled) {
                parseLineDetectPanelData(rxData, lines);
            } else {
                lines.clear();
            }

            lineCalc.update(lines);
            linePatternCalc.update(getActiveTask(globals::programState), lines, globals::car.distance);
            const DetectedLines detectedLines = { lineCalc.lines(), linePatternCalc.pattern() };
            xQueueOverwrite(detectedLinesQueue, &detectedLines);
        }

        if (frontLineDetectPanelLink.shouldSend()) {
            fillLineDetectPanelData(txData);
            frontLineDetectPanelLink.send(txData);
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

