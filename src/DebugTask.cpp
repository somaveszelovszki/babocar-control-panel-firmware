#include <micro/task/common.hpp>
#include <micro/container/vec.hpp>
#include <micro/bsp/tim.hpp>
#include <micro/container/ring_buffer.hpp>
#include <micro/container/span.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/convert.hpp>
#include <micro/utils/arrays.hpp>
#include <micro/debug/params.hpp>

#include <cfg_board.hpp>
#include <globals.hpp>

#include <FreeRTOS.h>
#include <queue.h>

#include <cstring>

#define MAX_TX_BUFFER_SIZE  512u    // size of the log TX buffer
#define MAX_RX_BUFFER_SIZE  512u    // size of the log RX buffer

using namespace micro;

QueueHandle_t logQueue;

static ring_buffer<uint8_t[MAX_RX_BUFFER_SIZE], 3> rxBuffer;
static vec<uint8_t, MAX_TX_BUFFER_SIZE> txBuffer;

volatile bool uartOccupied = false;

extern "C" void runDebugTask(const void *argument) {
    char txLog[LOG_MSG_MAX_SIZE];

    HAL_UART_Receive_DMA(uart_Command, *rxBuffer.getWritableBuffer(), MAX_RX_BUFFER_SIZE);

    while (true) {
        // handle incoming control messages from the monitoring app
//        if (isOk(getRxMsg(rxMsg))) {
//            handleRxMsg(rxMsg);
//        }

        // receives all available messages coming from the tasks and adds them to the buffer vector
        if(!uartOccupied && xQueueReceive(logQueue, txLog, 0)) {
            HAL_UART_Transmit_IT(uart_Command, reinterpret_cast<uint8_t*>(txLog), strlen(txLog));
            uartOccupied = true;
//            while (HAL_OK != HAL_UART_Transmit_IT(uart_Command, reinterpret_cast<uint8_t*>(txLog), strlen(txLog))) {  // sends messages once UART is free
//                vTaskDelay(1);
//            }
        }
    }

    vTaskDelete(nullptr);
}

/* @brief Callback for Serial UART RxCplt - called when receive finishes.
 */
void micro_Command_Uart_RxCpltCallback() {
    // does not handle Serial messages
}

/* @brief Callback for Serial UART TxCplt - called when transmit finishes.
 */
void micro_Command_Uart_TxCpltCallback() {
    uartOccupied = false;
}
