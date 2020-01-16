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

    for (uint8_t i = 0; i < rxData.lines.numLines; ++i) {
        const line_t * const l = &rxData.lines.values[i];
        lines.push_back(Line{ millimeter_t(l->pos_mm) * (mirror ? -1 : 1), l->id });
    }

    lines.removeDuplicates();
}

volatile bool newData = false;

void startPanel() {
    millisecond_t prevSendTime = millisecond_t(0);
    char startChar = 'S';
    do {
        if (getTime() - prevSendTime > millisecond_t(50)) {
            HAL_UART_Transmit_DMA(uart_FrontLineDetectPanel, (uint8_t*)&startChar, 1);
            prevSendTime = getTime();
        }
        vTaskDelay(5);
    } while (!newData);
    newData = false;
}

extern "C" void runLineDetectTask(const void *argument) {

    detectedLinesQueue = xQueueCreateStatic(DETECTED_LINES_QUEUE_LENGTH, sizeof(DetectedLines), detectedLinesQueueStorageBuffer, &detectedLinesQueueBuffer);

    Lines lines;

    lineDetectPanelDataOut_t rxData;
    lineDetectPanelDataIn_t txData;

    HAL_UART_Receive_DMA(uart_FrontLineDetectPanel, (uint8_t*)&rxData, sizeof(lineDetectPanelDataOut_t));

    vTaskDelay(10); // gives time to other tasks to wake up

    millisecond_t prevReadTime;

    LinePattern linePattern;
    linePattern.type = LinePattern::SINGLE_LINE;
    lines.push_back(Line{ millimeter_t(0), 1 });

    startPanel();

    millisecond_t prevSendTime = millisecond_t(0);
    globals::isLineDetectTaskOk = true;

    while (true) {
//        frontLineDetectPanelLink.update();
//        globals::isLineDetectTaskOk = frontLineDetectPanelLink.isConnected();
//
//        if (frontLineDetectPanelLink.readAvailable(rxData)) {
//            prevReadTime = getTime();
//            parseLineDetectPanelData(rxData, lines);
//            lineCalc.update(lines);
//            linePatternCalc.update(getActiveTask(globals::programState), lines, globals::car.distance);
//            const DetectedLines detectedLines = { lineCalc.lines(), linePatternCalc.pattern() };
//            xQueueOverwrite(detectedLinesQueue, &detectedLines);
//        }
//
//        if (frontLineDetectPanelLink.shouldSend()) {
//            fillLineDetectPanelData(txData);
//            frontLineDetectPanelLink.send(txData);
//        }

        if (newData) {
            newData = false;
            parseLineDetectPanelData(rxData, lines);
            lineCalc.update(lines);
            linePatternCalc.update(getActiveTask(globals::programState), lines, globals::car.distance);
            const DetectedLines detectedLines = { lineCalc.lines(), linePatternCalc.pattern() };
            xQueueOverwrite(detectedLinesQueue, &detectedLines);
        }

        if (getTime() - prevSendTime > millisecond_t(20)) {
            fillLineDetectPanelData(txData);
            HAL_UART_Transmit_DMA(uart_FrontLineDetectPanel, (uint8_t*)&txData, sizeof(lineDetectPanelDataIn_t));
            prevSendTime = getTime();
        }

        vTaskDelay(1);
    }

    vTaskDelete(nullptr);
}

/* @brief Callback for front line detect panel UART RxCplt - called when receive finishes.
 */
void micro_FrontLineDetectPanel_Uart_RxCpltCallback(const uint32_t leftBytes) {
    newData = true;
    //frontLineDetectPanelLink.onNewRxData(sizeof(lineDetectPanelDataOut_t) - leftBytes);
}

void micro_FrontLineDetectPanel_Uart_ErrorCallback() {
    //frontLineDetectPanelLink.onRxError();
}

