#include <cfg_board.h>
#include <micro/task/common.hpp>
#include <micro/container/vec.hpp>
#include <micro/container/ring_buffer.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/convert.hpp>
#include <micro/utils/arrays.hpp>
#include <micro/utils/timer.hpp>
#include <micro/debug/params.hpp>

#include <globals.hpp>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include <cstring>

using namespace micro;

#define MAX_RX_BUFFER_SIZE          1024
#define DEBUG_PARAMS_STR_MAX_SIZE   1024

#define LOG_QUEUE_LENGTH 16
QueueHandle_t logQueue;
static uint8_t logQueueStorageBuffer[LOG_QUEUE_LENGTH * LOG_MSG_MAX_SIZE];
static StaticQueue_t logQueueBuffer;

static ring_buffer<uint8_t[MAX_RX_BUFFER_SIZE], 3> rxBuffer;
static char txLog[LOG_MSG_MAX_SIZE];
static char debugParamsStr[DEBUG_PARAMS_STR_MAX_SIZE];

static Params debugParams;
static Timer debugParamsSendTimer;

static Timer ledBlinkTimer;

static bool areAllTasksInitialized(void) {
    return globals::isControlTaskInitialized &&
           globals::isDebugTaskInitialized &&
           globals::isDistSensorTaskInitialized &&
           globals::isGyroTaskInitialized &&
           globals::isLineDetectInitialized;
}

extern "C" void runDebugTask(const void *argument) {
    logQueue = xQueueCreateStatic(LOG_QUEUE_LENGTH, LOG_MSG_MAX_SIZE, logQueueStorageBuffer, &logQueueBuffer);
    globals::initializeGlobalParams(debugParams);

    vTaskDelay(10); // gives time to other tasks to wake up

    HAL_UART_Receive_DMA(uart_Command, *rxBuffer.getWritableBuffer(), MAX_RX_BUFFER_SIZE);

    debugParamsSendTimer.start(millisecond_t(250));
    ledBlinkTimer.start(millisecond_t(250));

    globals::isDebugTaskInitialized = true;
    LOG_DEBUG("Debug task initialized");

    while (true) {
        if (rxBuffer.size() > 0) {
            const uint8_t (*inCmd)[MAX_RX_BUFFER_SIZE] = rxBuffer.getReadableBuffer();
            debugParams.deserializeAll(reinterpret_cast<const char*>(*inCmd + 3));
            rxBuffer.updateTail(1);
        }

        if (debugParamsSendTimer.checkTimeout()) {
            strncpy(debugParamsStr, "[P]", 3);
            uint32_t len = 3 + debugParams.serializeAll(debugParamsStr + 3, DEBUG_PARAMS_STR_MAX_SIZE - 6);
            debugParamsStr[len++] = '\r';
            debugParamsStr[len++] = '\n';
            debugParamsStr[len++] = '\0';

            //while (HAL_OK != HAL_UART_Transmit_DMA(uart_Command, reinterpret_cast<uint8_t*>(debugParamsStr), len)) {}
        }

        // receives all available messages coming from the tasks and adds them to the buffer vector
        if(xQueueReceive(logQueue, txLog, 1)) {
            while (HAL_OK != HAL_UART_Transmit_DMA(uart_Command, reinterpret_cast<uint8_t*>(txLog), strlen(txLog))) {}
        }

        ledBlinkTimer.setPeriod(millisecond_t(areAllTasksInitialized() ? 500 : 250));
        if (ledBlinkTimer.checkTimeout()) {
            HAL_GPIO_TogglePin(gpio_Led, gpioPin_Led);
        }

        vTaskDelay(1);
    }

    vTaskDelete(nullptr);
}

/* @brief Callback for Serial UART RxCplt - called when receive finishes.
 */
void micro_Command_Uart_RxCpltCallback() {
    rxBuffer.updateHead(1);
    HAL_UART_Receive_DMA(uart_Command, *rxBuffer.getWritableBuffer(), MAX_RX_BUFFER_SIZE);
}

/* @brief Callback for Serial UART TxCplt - called when transmit finishes.
 */
void micro_Command_Uart_TxCpltCallback() {}
