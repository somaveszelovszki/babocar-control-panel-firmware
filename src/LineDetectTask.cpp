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

static LineDetectPanelLink rearLineDetectPanelLink(
    uart_RearLineDetectPanel,
    millisecond_t(LINE_DETECT_PANEL_LINK_OUT_TIMEOUT_MS),
    millisecond_t(LINE_DETECT_PANEL_LINK_IN_PERIOD_MS));

static LinePatternCalculator linePatternCalc;

static Line mainLine;

static void fillLineDetectPanelData(lineDetectPanelDataIn_t& txData, bool indicatorLedsEnabled) {
    txData.flags = 0x00;
    if (indicatorLedsEnabled) txData.flags |= LINE_DETECT_PANEL_FLAG_INDICATOR_LEDS_ENABLED;
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

    lineDetectPanelDataOut_t rxDataFront, rxDataRear;
    lineDetectPanelDataIn_t txDataFront, txDataRear;
    DetectedLines detectedLines;

    while (true) {
        frontLineDetectPanelLink.update();
        rearLineDetectPanelLink.update();

        globals::isLineDetectTaskOk = frontLineDetectPanelLink.isConnected() && rearLineDetectPanelLink.isConnected();

        if (frontLineDetectPanelLink.readAvailable(rxDataFront)) {
            if (globals::lineDetectionEnabled) {
                parseLineDetectPanelData(rxDataFront, detectedLines.lines.front);
            } else {
                detectedLines.lines.front.clear();
            }

            linePatternCalc.update(getActiveTask(globals::programState), detectedLines.lines.front, globals::car.distance);
            detectedLines.pattern = linePatternCalc.pattern();
            detectedLines.isPending = linePatternCalc.isPending();
            xQueueOverwrite(detectedLinesQueue, &detectedLines);
        }

        if (rearLineDetectPanelLink.readAvailable(rxDataRear)) {
            if (globals::lineDetectionEnabled) {
                parseLineDetectPanelData(rxDataRear, detectedLines.lines.rear, true);
            } else {
                detectedLines.lines.rear.clear();
            }
        }

        if (frontLineDetectPanelLink.shouldSend()) {
            fillLineDetectPanelData(txDataFront, globals::frontIndicatorLedsEnabled);
            frontLineDetectPanelLink.send(txDataFront);
        }

        if (rearLineDetectPanelLink.shouldSend()) {
            fillLineDetectPanelData(txDataRear, globals::rearIndicatorLedsEnabled);
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
