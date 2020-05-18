#include <micro/container/ring_buffer.hpp>
#include <micro/debug/DebugLed.hpp>
#include <micro/debug/params.hpp>
#include <micro/debug/SystemManager.hpp>
#include <micro/port/task.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/str_utils.hpp>
#include <micro/utils/timer.hpp>

#include <cfg_board.h>

using namespace micro;

namespace {

constexpr uint32_t MAX_PARAMS_BUFFER_SIZE = 1024;

ring_buffer<uint8_t[MAX_PARAMS_BUFFER_SIZE], 3> rxBuffer;
Log::message_t txLog;
char paramsStr[MAX_PARAMS_BUFFER_SIZE];
semaphore_t txSemaphore;

void transmit(const char * const data) {
    HAL_UART_Transmit_DMA(uart_Debug, reinterpret_cast<uint8_t*>(const_cast<char*>(data)), strlen(data));
    txSemaphore.take(micro::numeric_limits<millisecond_t>::infinity());
}

bool monitorTasks() {
    static Timer failureLogTimer(millisecond_t(100));

    const SystemManager::taskStates_t failingTasks = SystemManager::instance().failingTasks();

    if (failingTasks.size() && failureLogTimer.checkTimeout()) {
        char msg[LOG_MSG_MAX_SIZE];
        uint32_t idx = 0;
        for (SystemManager::taskStates_t::const_iterator it = failingTasks.begin(); it != failingTasks.end(); ++it) {
            idx += strncpy_until(&msg[idx], it->details.pcTaskName, min(static_cast<uint32_t>(configMAX_TASK_NAME_LEN), LOG_MSG_MAX_SIZE - idx));
            if (it != failingTasks.back()) {
                idx += strncpy_until(&msg[idx], ", ", sizeof(", "), LOG_MSG_MAX_SIZE - idx);
            }
        }
        msg[idx] = '\0';
        LOG_ERROR("Failing tasks: %s", msg);
    }

    return failingTasks.size() == 0;
}

} // namespace

extern "C" void runDebugTask(void) {

    SystemManager::instance().registerTask();

    HAL_UART_Receive_DMA(uart_Debug, *rxBuffer.getWritableBuffer(), MAX_PARAMS_BUFFER_SIZE);

    DebugLed debugLed(gpio_Led, gpioPin_Led);
    Timer debugParamsSendTimer(millisecond_t(500));

    while (true) {
        if (rxBuffer.size() > 0) {
            const char * const inCmd = reinterpret_cast<const char*>(*rxBuffer.getReadableBuffer());
            rxBuffer.updateTail(1);
            Params::instance().deserializeAll(inCmd, MAX_PARAMS_BUFFER_SIZE);
        }

        if (debugParamsSendTimer.checkTimeout()) {
            Params::instance().serializeAll(paramsStr, MAX_PARAMS_BUFFER_SIZE);
            transmit(paramsStr);
        }

        if (Log::instance().receive(txLog)) {
            transmit(txLog);
        }

        debugLed.update(monitorTasks());
        SystemManager::instance().notify(true);
        os_delay(1);
    }
}

void micro_Command_Uart_RxCpltCallback() {
    if (MAX_PARAMS_BUFFER_SIZE > uart_Debug->hdmarx->Instance->NDTR) {
        rxBuffer.updateHead(1);
    }
    HAL_UART_Receive_DMA(uart_Debug, *rxBuffer.getWritableBuffer(), MAX_PARAMS_BUFFER_SIZE);
}

void micro_Command_Uart_TxCpltCallback() {
    txSemaphore.give();
}
