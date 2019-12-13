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

static LineDetectPanel rearLineDetectPanel(uart_RearLineDetectPanel);
static lineDetectPanelDataIn_t rearLineDetectPanelData;

static LineCalculator lineCalc;
static LinePatternCalculator linePatternCalc;

static Timer lineDetectPanelsSendTimer;

static Line mainLine;

static void fillLineDetectPanelData(lineDetectPanelDataIn_t& panelData) {
    panelData.flags = 0x00;
    if (globals::indicatorLedsEnabled) panelData.flags |= LINE_DETECT_PANEL_FLAG_INDICATOR_LEDS_ENABLED;
}

static void getLinesFromPanel(LineDetectPanel& panel, LinePositions& positions, bool mirror = false) {
    lineDetectPanelDataOut_t dataIn = panel.acquireLastValue();
    positions.clear();
    for (uint8_t i = 0; i < dataIn.lines.numLines; ++i) {
        positions.append(millimeter_t(dataIn.lines.values[i].pos_mm) * (mirror ? -1 : 1));
    }

//    const char *PANEL = mirror ? "r" : "f";
//
//    if (positions.size() == 0) {
//        LOG_DEBUG("%s: none", PANEL);
//    } else if (positions.size() == 1) {
//        LOG_DEBUG("%s: %f", PANEL, positions[0].get());
//    } else if (positions.size() == 2) {
//        LOG_DEBUG("%s: %f, %f", PANEL, positions[0].get(), positions[1].get());
//    } else if (positions.size() == 3) {
//        LOG_DEBUG("%s: %f, %f, %f", PANEL, positions[0].get(), positions[1].get(), positions[2].get());
//    }

    positions.removeDuplicates();
}

extern "C" void runLineDetectTask(const void *argument) {

    detectedLinesQueue = xQueueCreateStatic(DETECTED_LINES_QUEUE_LENGTH, sizeof(DetectedLines), detectedLinesQueueStorageBuffer, &detectedLinesQueueBuffer);

    vTaskDelay(300); // gives time to other tasks to wake up

    frontLineDetectPanel.start();
    //rearLineDetectPanel.start();

    frontLineDetectPanel.waitResponse();
    //rearLineDetectPanel.waitResponse();

    lineDetectPanelsSendTimer.start(millisecond_t(400));

    globals::isLineDetectInitialized = true;
    LOG_DEBUG("Line detect task initialized");

    while (true) {

        if (lineDetectPanelsSendTimer.checkTimeout()) {
            fillLineDetectPanelData(frontLineDetectPanelData);
            frontLineDetectPanel.send(frontLineDetectPanelData);

            fillLineDetectPanelData(rearLineDetectPanelData);
            rearLineDetectPanel.send(rearLineDetectPanelData);
        }

        if (frontLineDetectPanel.hasNewValue()) {// && rearLineDetectPanel.hasNewValue()) {
            LinePositions frontLinePositions, rearLinePositions;

            getLinesFromPanel(frontLineDetectPanel, frontLinePositions);
            //getLinesFromPanel(rearLineDetectPanel, rearLinePositions, true);

            //lineCalc.update(frontLinePositions, rearLinePositions);
            lineCalc.update(frontLinePositions);
//            LineCalculator::updateMainLine(lineCalc.lines(), mainLine);
//
//            LOG_DEBUG("(%f, %f) %f deg\t\t%f deg/s",
//                    mainLine.pos_front.get(),
//                    mainLine.pos_rear.get(),
//                    static_cast<degree_t>(mainLine.angle).get(),
//                    static_cast<deg_per_sec_t>(mainLine.angular_velocity).get());

            linePatternCalc.update(frontLinePositions, rearLinePositions, globals::car.distance);
            const DetectedLines detectedLines = { lineCalc.lines(), linePatternCalc.pattern() };
            xQueueOverwrite(detectedLinesQueue, &detectedLines);
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

/* @brief Callback for rear line detect panel UART RxCplt - called when receive finishes.
 */
void micro_RearLineDetectPanel_Uart_RxCpltCallback() {
    rearLineDetectPanel.onDataReceived();
}
