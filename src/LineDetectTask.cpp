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
        lines.append(Line{ millimeter_t(l->pos_mm) * (mirror ? -1 : 1), l->id });
    }

    lines.removeDuplicates();
}

void testLinePattern() {
    static const m_per_sec_t speed = m_per_sec_t(1);
    static const millisecond_t step = millisecond_t(10);

    static const Lines positions[] = {
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 }, { centimeter_t(3.8), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(3.8), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(3.8), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(3.8), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(3.8), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(3.8), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(3.8), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(3.8), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(3.8), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(3.8), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(3.8), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(3.8), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(3.8), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(3.8), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(3.8), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(3.8), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(3.8), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(3.8), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(3.8), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(3.8), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(3.8), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(3.8), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(3.8), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(3.8), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(3.9), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(4.0), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(4.1), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(4.2), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(4.3), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(4.4), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(4.5), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(4.6), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(4.7), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(4.8), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(4.9), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(5.0), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(5.1), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(5.2), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(5.3), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(5.4), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(5.5), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(5.7), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(5.9), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(6.1), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(6.3), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(6.5), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(6.8), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(7.2), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(7.6), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(8.0), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(8.5), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(9.0), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(9.5), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(10.1), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(10.8), 3 } },
        { { centimeter_t(0), 1 }, { centimeter_t(11.5), 3 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } },
        { { centimeter_t(0), 1 } }
    };

    meter_t distance = meter_t(0);

    LinePattern prevPattern;

    for (uint32_t i = 0; i < ARRAY_SIZE(positions); ++i) {
        linePatternCalc.update(globals::programState, positions[i], distance);
        distance += step * speed;

        const LinePattern& currentPattern = linePatternCalc.pattern();
        if (currentPattern != prevPattern) {
            prevPattern = currentPattern;
        }
    }
}

extern "C" void runLineDetectTask(const void *argument) {

    detectedLinesQueue = xQueueCreateStatic(DETECTED_LINES_QUEUE_LENGTH, sizeof(DetectedLines), detectedLinesQueueStorageBuffer, &detectedLinesQueueBuffer);

    vTaskDelay(300); // gives time to other tasks to wake up

    testLinePattern();

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
