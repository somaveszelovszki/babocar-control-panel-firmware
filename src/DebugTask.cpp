#include <cfg_board.h>
#include <micro/task/common.hpp>
#include <micro/container/vec.hpp>
#include <micro/container/ring_buffer.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/convert.hpp>
#include <micro/utils/arrays.hpp>
#include <micro/utils/timer.hpp>
#include <micro/debug/params.hpp>

#include <cfg_car.hpp>
#include <globals.hpp>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include <cstring>

using namespace micro;

#define MAX_RX_BUFFER_SIZE  1024

#define LOG_QUEUE_LENGTH    16
QueueHandle_t logQueue;
static uint8_t logQueueStorageBuffer[LOG_QUEUE_LENGTH * LOG_MSG_MAX_SIZE];
static StaticQueue_t logQueueBuffer;

static ring_buffer<uint8_t[MAX_RX_BUFFER_SIZE], 6> rxBuffer;
static char txLog[LOG_MSG_MAX_SIZE];
static microsecond_t txEndTime;

#if SERIAL_DEBUG_ENABLED
static char inCmd[MAX_RX_BUFFER_SIZE];
static char debugParamsStr[MAX_RX_BUFFER_SIZE];
static Params debugParams;
static Timer debugParamsSendTimer;
#endif // SERIAL_DEBUG_ENABLED

static Timer ledBlinkTimer;

static void transmit(const char * const data, const uint32_t length) {
    HAL_UART_Transmit_DMA(uart_Command, reinterpret_cast<uint8_t*>(const_cast<char*>(data)), length);
    txEndTime = getExactTime() + length * second_t(1) / (uart_Command->Init.BaudRate / 12);
}

extern "C" void runDebugTask(const void *argument) {
    logQueue = xQueueCreateStatic(LOG_QUEUE_LENGTH, LOG_MSG_MAX_SIZE, logQueueStorageBuffer, &logQueueBuffer);

    vTaskDelay(10); // gives time to other tasks to wake up

    HAL_UART_Receive_DMA(uart_Command, *rxBuffer.getWritableBuffer(), MAX_RX_BUFFER_SIZE);

#if SERIAL_DEBUG_ENABLED
    globals::registerGlobalParams(debugParams);
    debugParamsSendTimer.start(millisecond_t(500));
#endif // SERIAL_DEBUG_ENABLED

    ledBlinkTimer.start(millisecond_t(250));

    globals::isDebugTaskOk = true;
    LOG_DEBUG("Debug task initialized");

    while (true) {
#if SERIAL_DEBUG_ENABLED
        if (rxBuffer.size() > 0) {
            const char * const rxData = reinterpret_cast<const char*>(*rxBuffer.getReadableBuffer());

            uint32_t len = 0;
            for (; len < MAX_RX_BUFFER_SIZE; ++len) {
                const char c = rxData[len];
                if ('$' == c) break;
                inCmd[len] = c;
            }

            rxBuffer.updateTail(1);

            if (!strncmp(inCmd, "[P]", 3)) {
                debugParams.deserializeAll(&inCmd[3], len - 3);
                LOG_INFO("Params updated from server");
            }
        }

        if (getExactTime() > txEndTime && debugParamsSendTimer.checkTimeout()) {
            strncpy(debugParamsStr, "[P]", 3);
            uint32_t len = 3 + debugParams.serializeAll(debugParamsStr + 3, MAX_RX_BUFFER_SIZE - (3 + 4));
            debugParamsStr[len++] = '$';
            debugParamsStr[len++] = '\r';
            debugParamsStr[len++] = '\n';
            debugParamsStr[len++] = '\0';

            transmit(debugParamsStr, len);
        }
#endif // SERIAL_DEBUG_ENABLED

        // receives all available messages coming from the tasks and adds them to the buffer vector
        if (getExactTime() > txEndTime && xQueueReceive(logQueue, txLog, 1)) {
            transmit(txLog, strlen(txLog));
        }

        ledBlinkTimer.setPeriod(millisecond_t(globals::areAllTasksOk() ? 500 : 250));
        if (ledBlinkTimer.checkTimeout()) {
            HAL_GPIO_TogglePin(gpio_Led, gpioPin_Led);
//            if (!globals::isControlTaskOk)    LOG_DEBUG("ControlTask not OK");
//            if (!globals::isDebugTaskOk)      LOG_DEBUG("DebugTask not OK");
//            if (!globals::isDistSensorTaskOk) LOG_DEBUG("DistSensorTask not OK");
//            if (!globals::isLineDetectTaskOk) LOG_DEBUG("LineDetectTask not OK");
//            if (!globals::isGyroTaskOk)       LOG_DEBUG("GyroTask not OK");
        }
    }

    vTaskDelete(nullptr);
}

/* @brief Callback for Serial UART RxCplt - called when receive finishes.
 */
void micro_Command_Uart_RxCpltCallback(const uint32_t leftBytes) {
    if (MAX_RX_BUFFER_SIZE > leftBytes) {
        rxBuffer.updateHead(1);
    }

    HAL_UART_Receive_DMA(uart_Command, *rxBuffer.getWritableBuffer(), MAX_RX_BUFFER_SIZE);
}

/* @brief Callback for Serial UART TxCplt - called when transmit finishes.
 */
void micro_Command_Uart_TxCpltCallback() {}
