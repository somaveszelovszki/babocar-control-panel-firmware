#include <micro/task/common.hpp>
#include <micro/container/vec.hpp>
#include <micro/bsp/tim.hpp>
#include <micro/bsp/uart.hpp>
#include <micro/bsp/queue.hpp>
#include <micro/bsp/task.hpp>
#include <micro/container/ring_buffer.hpp>
#include <micro/container/span.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/convert.hpp>
#include <micro/utils/arrays.hpp>
#include <micro/debug/params.hpp>

#include <cfg_board.hpp>
#include <cfg_os.hpp>

#include <globals.hpp>

#include <cstring>

#define MAX_TX_BUFFER_SIZE  512u    // size of the log TX buffer
#define MAX_RX_BUFFER_SIZE  512u    // size of the log RX buffer

using namespace micro;

namespace {

enum class DebugCode {
    Log
};

ring_buffer<uint8_t[MAX_RX_BUFFER_SIZE], 3> rxBuffer;
vec<uint8_t, MAX_TX_BUFFER_SIZE> txBuffer;

} // namespace

extern "C" void runDebugTask(const void *argument) {
    char txLog[LOG_MSG_MAX_SIZE];

    UART_Receive_DMA(cfg::uart_Command, *rxBuffer.getWritableBuffer(), MAX_RX_BUFFER_SIZE);

    while (!task::hasErrorHappened()) {
        // handle incoming control messages from the monitoring app
//        if (isOk(getRxMsg(rxMsg))) {
//            handleRxMsg(rxMsg);
//        }

        // receives all available messages coming from the tasks and adds them to the buffer vector
        if(micro::isOk(micro::queueReceive(cfg::queue_Log, &txLog))) {
            while (!isOk(UART_Transmit_IT(cfg::uart_Command, reinterpret_cast<const uint8_t*>(txLog), strlen(txLog)))) {  // sends messages once UART is free
                micro::nonBlockingDelay(micro::millisecond_t(1));
            }
        }

        micro::nonBlockingDelay(micro::millisecond_t(1));
    }

    taskDeleteCurrent();
}

/* @brief Callback for Serial UART RxCplt - called when receive finishes.
 */
void micro_Command_Uart_RxCpltCallback() {
    // does not handle Serial messages
}
