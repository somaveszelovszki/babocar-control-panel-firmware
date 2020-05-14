#include <micro/container/ring_buffer.hpp>
#include <micro/debug/params.hpp>
#include <micro/port/task.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/timer.hpp>

#include <cfg_board.h>
#include <globals.hpp>

using namespace micro;

extern Params params;

queue_t<char[LOG_MSG_MAX_SIZE], LOG_QUEUE_MAX_SIZE> logQueue;

namespace {

#define SERIAL_DEBUG_ENABLED true

constexpr uint32_t MAX_BUFFER_SIZE = 1024;

ring_buffer<uint8_t[MAX_BUFFER_SIZE], 6> rxBuffer;
char txLog[LOG_MSG_MAX_SIZE];
semaphore_t txSemaphore;

void transmit(const char * const data) {
    HAL_UART_Transmit_DMA(uart_Command, reinterpret_cast<uint8_t*>(const_cast<char*>(data)), strlen(data));
    txSemaphore.take(micro::numeric_limits<millisecond_t>::infinity());
}

} // namespace

extern "C" void runDebugTask(void) {
    HAL_UART_Receive_DMA(uart_Command, *rxBuffer.getWritableBuffer(), MAX_BUFFER_SIZE);

#if SERIAL_DEBUG_ENABLED
    char paramsStr[MAX_BUFFER_SIZE];
    Timer debugParamsSendTimer;
    debugParamsSendTimer.start(millisecond_t(500));
#endif // SERIAL_DEBUG_ENABLED

    Timer ledBlinkTimer;
    ledBlinkTimer.start(millisecond_t(250));

    globals::isDebugTaskOk = true;
    LOG_DEBUG("Debug task initialized");

    while (true) {
#if SERIAL_DEBUG_ENABLED
        if (rxBuffer.size() > 0) {
            const char * const inCmd = reinterpret_cast<const char*>(*rxBuffer.getReadableBuffer());
            rxBuffer.updateTail(1);
            params.deserializeAll(inCmd, MAX_BUFFER_SIZE);
        }

        if (debugParamsSendTimer.checkTimeout()) {
            params.serializeAll(paramsStr, MAX_BUFFER_SIZE);
            transmit(paramsStr);
        }
#endif // SERIAL_DEBUG_ENABLED

        // receives all available messages coming from the tasks and adds them to the buffer vector
        if (logQueue.receive(txLog, millisecond_t(1))) {
            transmit(txLog);
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
}

void micro_Command_Uart_RxCpltCallback() {
    if (MAX_BUFFER_SIZE > uart_Command->hdmarx->Instance->NDTR) {
        rxBuffer.updateHead(1);
    }
    HAL_UART_Receive_DMA(uart_Command, *rxBuffer.getWritableBuffer(), MAX_BUFFER_SIZE);
}

void micro_Command_Uart_TxCpltCallback() {
    txSemaphore.give();
}
