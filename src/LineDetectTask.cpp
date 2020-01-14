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
    millisecond_t(LINE_DETECT_PANEL_LINK_RX_TIMEOUT_MS),
    millisecond_t(LINE_DETECT_PANEL_LINK_TX_PERIOD_MS));

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

static lineDetectPanelDataOut_t rxDataBuffer;
static volatile bool available = false;

extern "C" void runLineDetectTask(const void *argument) {

    detectedLinesQueue = xQueueCreateStatic(DETECTED_LINES_QUEUE_LENGTH, sizeof(DetectedLines), detectedLinesQueueStorageBuffer, &detectedLinesQueueBuffer);

    Lines lines;

    lineDetectPanelDataIn_t txData;

    HAL_UART_Receive_DMA(uart_FrontLineDetectPanel, (uint8_t*)&rxDataBuffer, sizeof(lineDetectPanelDataOut_t));

    vTaskDelay(300); // gives time to other tasks to wake up

    millisecond_t prevReadTime;

    LinePattern linePattern;
    linePattern.type = LinePattern::SINGLE_LINE;
    lines.push_back(Line{ millimeter_t(0), 1 });

    while (true) {
        if (available) {
            available = false;

            lineDetectPanelDataOut_t rxData;
            memcpy(&rxData, &rxDataBuffer, sizeof(lineDetectPanelDataOut_t));

            HAL_GPIO_WritePin(gpio_Led, gpioPin_Led, GPIO_PIN_SET);
            prevReadTime = getTime();
            parseLineDetectPanelData(rxData, lines);
            lineCalc.update(lines);
            linePatternCalc.update(globals::programState, lines, globals::car.distance);
            const DetectedLines detectedLines = { lineCalc.lines(), linePatternCalc.pattern() };
            xQueueOverwrite(detectedLinesQueue, &detectedLines);
        } else {
            HAL_GPIO_WritePin(gpio_Led, gpioPin_Led, GPIO_PIN_RESET);
        }
        //frontLineDetectPanelLink.update();
        //globals::isLineDetectTaskOk = frontLineDetectPanelLink.isConnected();
//        HAL_GPIO_WritePin(gpio_Led, gpioPin_Led, globals::isLineDetectTaskOk ? GPIO_PIN_SET : GPIO_PIN_RESET);
//
//        if (frontLineDetectPanelLink.readAvailable(rxData)) {
//            HAL_GPIO_WritePin(gpio_Led, gpioPin_Led, GPIO_PIN_SET);
//            prevReadTime = getTime();
//            parseLineDetectPanelData(rxData, lines);
//            lineCalc.update(lines);
//            linePatternCalc.update(globals::programState, lines, globals::car.distance);
//            const DetectedLines detectedLines = { lineCalc.lines(), linePatternCalc.pattern() };
//            xQueueOverwrite(detectedLinesQueue, &detectedLines);
//        }
//
//        if (frontLineDetectPanelLink.shouldSend()) {
//            fillLineDetectPanelData(txData);
//            frontLineDetectPanelLink.send(txData);
//        }

        vTaskDelay(1);
    }

    vTaskDelete(nullptr);
}

/* @brief Callback for front line detect panel UART RxCplt - called when receive finishes.
 */
void micro_FrontLineDetectPanel_Uart_RxCpltCallback(const uint32_t leftBytes) {
    //frontLineDetectPanelLink.onNewRxData(sizeof(lineDetectPanelDataOut_t) - leftBytes);
    if (0 == leftBytes) {
        available = true;
    }
    HAL_UART_Receive_DMA(uart_FrontLineDetectPanel, (uint8_t*)&rxDataBuffer, sizeof(lineDetectPanelDataOut_t));
}

void micro_FrontLineDetectPanel_Uart_ErrorCallback() {
    //frontLineDetectPanelLink.onRxError();
    HAL_UART_Receive_DMA(uart_FrontLineDetectPanel, (uint8_t*)&rxDataBuffer, sizeof(lineDetectPanelDataOut_t));
}

