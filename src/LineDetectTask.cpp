#include <cfg_board.h>
#include <micro/utils/log.hpp>
#include <micro/utils/Line.hpp>
#include <micro/utils/timer.hpp>
#include <micro/panel/LineDetectPanel.hpp>
#include <micro/panel/LineDetectPanelData.h>
#include <micro/task/common.hpp>
#include <micro/sensor/Filter.hpp>

#include <ControlData.hpp>
#include <DistancesData.hpp>
#include <DetectedLines.hpp>
#include <cfg_car.hpp>
#include <globals.hpp>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

using namespace micro;

#define DETECTED_LINES_QUEUE_LENGTH 1
QueueHandle_t detectedLinesQueue;
static uint8_t detectedLinesQueueStorageBuffer[DETECTED_LINES_QUEUE_LENGTH * sizeof(DetectedLines)];
static StaticQueue_t detectedLinesQueueBuffer;

static LineDetectPanel frontLineDetectPanel(uart_FrontLineDetectPanel);
static lineDetectPanelDataIn_t frontLineDetectPanelData;

static LineCalculator lineCalc;
static LinePatternCalculator linePatternCalc;

static Timer lineDetectPanelSendTimer;

static Line mainLine;

static void fillLineDetectPanelData(lineDetectPanelDataIn_t& panelData) {
    panelData.flags = 0x00;
    if (globals::indicatorLedsEnabled) panelData.flags |= LINE_DETECT_PANEL_FLAG_INDICATOR_LEDS_ENABLED;
}

static void getLinesFromPanel(LineDetectPanel& panel, Lines& lines, bool mirror = false) {
    lineDetectPanelDataOut_t dataIn = panel.acquireLastValue();
    lines.clear();

    for (uint8_t i = 0; i < dataIn.lines.numLines; ++i) {
        const line_t * const l = &dataIn.lines.values[i];
        lines.push_back(Line{ millimeter_t(l->pos_mm) * (mirror ? -1 : 1), l->id });
    }

    lines.removeDuplicates();
}

extern "C" void runLineDetectTask(const void *argument) {

    detectedLinesQueue = xQueueCreateStatic(DETECTED_LINES_QUEUE_LENGTH, sizeof(DetectedLines), detectedLinesQueueStorageBuffer, &detectedLinesQueueBuffer);

    vTaskDelay(300); // gives time to other tasks to wake up

    frontLineDetectPanel.start();
    frontLineDetectPanel.waitResponse();

    lineDetectPanelSendTimer.start(millisecond_t(400));

    globals::isLineDetectInitialized = true;
    LOG_DEBUG("Line detect task initialized");

    Lines lines;

    while (true) {

        if (frontLineDetectPanel.hasNewValue()) {
            getLinesFromPanel(frontLineDetectPanel, lines);
            lineCalc.update(lines);
            linePatternCalc.update(globals::programState, lines, globals::car.distance);
            const DetectedLines detectedLines = { lineCalc.lines(), linePatternCalc.pattern() };
            xQueueOverwrite(detectedLinesQueue, &detectedLines);
        }

        if (lineDetectPanelSendTimer.checkTimeout()) {
            fillLineDetectPanelData(frontLineDetectPanelData);
            frontLineDetectPanel.send(frontLineDetectPanelData);
        }

        vTaskDelay(1);
    }

    vTaskDelete(nullptr);
}

/* @brief Callback for front line detect panel UART RxCplt - called when receive finishes.
 */
void micro_FrontLineDetectPanel_Uart_RxCpltCallback() {
    frontLineDetectPanel.onDataReceived();
}
