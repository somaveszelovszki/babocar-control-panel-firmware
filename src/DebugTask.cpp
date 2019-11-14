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

#include <cstring>

#define MAX_TX_BUFFER_SIZE  1024    // size of the log TX buffer
#define MAX_RX_BUFFER_SIZE  1024    // size of the log RX buffer

using namespace micro;

#define LOG_QUEUE_LENGTH 16
QueueHandle_t logQueue;
static uint8_t logQueueStorageBuffer[LOG_QUEUE_LENGTH * LOG_MSG_MAX_SIZE];
static StaticQueue_t logQueueBuffer;

static ring_buffer<uint8_t[MAX_RX_BUFFER_SIZE], 3> rxBuffer;
static vec<uint8_t, MAX_TX_BUFFER_SIZE> txBuffer;

static Params debugParams;
static Timer debugParamsSendTimer;

static Timer ledBlinkTimer;

volatile bool uartOccupied = false;

static bool areAllTasksInitialized(void) {
    return globals::isControlTaskInitialized &&
           globals::isDebugTaskInitialized &&
           globals::isSensorTaskInitialized;
}

extern "C" void runDebugTask(const void *argument) {
    logQueue = xQueueCreateStatic(LOG_QUEUE_LENGTH, LOG_MSG_MAX_SIZE, logQueueStorageBuffer, &logQueueBuffer);
    globals::initializeGlobalParams(debugParams);

    vTaskDelay(5); // gives time to other tasks to initialize their queues

    char txLog[LOG_MSG_MAX_SIZE];
    char debugParamsStr[LOG_MSG_MAX_SIZE];

    HAL_UART_Receive_DMA(uart_Command, *rxBuffer.getWritableBuffer(), MAX_RX_BUFFER_SIZE);

    debugParamsSendTimer.start(millisecond_t(100));
    ledBlinkTimer.start(millisecond_t(250));
    globals::isDebugTaskInitialized = true;

    while (true) {
        // handle incoming control messages from the monitoring app
//        if (isOk(getRxMsg(rxMsg))) {
//            handleRxMsg(rxMsg);
//        }

        if (rxBuffer.size() > 0) {
            const char *inCmd = rxBuffer.getReadableBuffer();
            debugParams.deserializeAll(in);
            rxBuffer.updateTail(1);
        }

        if (debugParamsSendTimer.checkTimeout()) {
            strncpy(debugParamsStr, "[P]", 3);
            uint32_t len = 3 + debugParams.serializeAll(debugParamsStr + 3, LOG_MSG_MAX_SIZE - 3);
            if (len < LOG_MSG_MAX_SIZE - 2) {
                debugParamsStr[len++] = '\r';
                debugParamsStr[len++] = '\n';
                debugParamsStr[len++] = '\0';
            }

            xQueueSend(logQueue, debugParamsStr, 1);
        }

        // receives all available messages coming from the tasks and adds them to the buffer vector
        if(!uartOccupied && xQueueReceive(logQueue, txLog, 0)) {
            HAL_UART_Transmit_IT(uart_Command, reinterpret_cast<uint8_t*>(txLog), strlen(txLog));
            uartOccupied = true;
//            while (HAL_OK != HAL_UART_Transmit_IT(uart_Command, reinterpret_cast<uint8_t*>(txLog), strlen(txLog))) {  // sends messages once UART is free
//                vTaskDelay(1);
//            }
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
}

/* @brief Callback for Serial UART TxCplt - called when transmit finishes.
 */
void micro_Command_Uart_TxCpltCallback() {
    uartOccupied = false;
}
