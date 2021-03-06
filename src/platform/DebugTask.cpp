#include <cfg_board.hpp>
#include <micro/container/ring_buffer.hpp>
#include <micro/debug/DebugLed.hpp>
#include <micro/debug/params.hpp>
#include <micro/debug/SystemManager.hpp>
#include <micro/port/semaphore.hpp>
#include <micro/port/task.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/str_utils.hpp>
#include <micro/utils/timer.hpp>


using namespace micro;

namespace {

#define FAILING_TASKS_LOG_ENABLED false

constexpr uint32_t MAX_PARAMS_BUFFER_SIZE = 1024;

typedef uint8_t rxParams_t[MAX_PARAMS_BUFFER_SIZE];
ring_buffer<rxParams_t, 3> rxBuffer;
Log::message_t txLog;
char paramsStr[MAX_PARAMS_BUFFER_SIZE];
semaphore_t txSemaphore;

void transmit(const char * const data) {
    uart_transmit(uart_Debug, reinterpret_cast<uint8_t*>(const_cast<char*>(data)), strlen(data));
    txSemaphore.take();
}

bool monitorTasks() {
    static Timer failureLogTimer(millisecond_t(100));

    const SystemManager::TaskStates failingTasks = SystemManager::instance().failingTasks();

#if FAILING_TASKS_LOG_ENABLED

    if (failingTasks.size() && failureLogTimer.checkTimeout()) {
        char msg[LOG_MSG_MAX_SIZE];
        uint32_t idx = 0;
        for (SystemManager::TaskStates::const_iterator it = failingTasks.begin(); it != failingTasks.end(); ++it) {
            idx += strncpy_until(&msg[idx], it->details.pcTaskName, min(static_cast<uint32_t>(configMAX_TASK_NAME_LEN), LOG_MSG_MAX_SIZE - idx));
            if (it != failingTasks.back()) {
                idx += strncpy_until(&msg[idx], ", ", sizeof(", "), LOG_MSG_MAX_SIZE - idx);
            }
        }
        msg[idx] = '\0';
        LOG_ERROR("Failing tasks: %s", msg);
    }

#endif // FAILING_TASKS_LOG_ENABLED

    return failingTasks.size() == 0;
}

} // namespace

extern "C" void runDebugTask(void) {

    SystemManager::instance().registerTask();

    uart_receive(uart_Debug, *rxBuffer.startWrite(), MAX_PARAMS_BUFFER_SIZE);

    DebugLed debugLed(gpio_Led);
    Timer debugParamsSendTimer(millisecond_t(200));

    while (true) {
        const rxParams_t *inCmd = rxBuffer.startRead();
        if (inCmd) {
            Params::instance().deserializeAll(reinterpret_cast<const char*>(*inCmd), MAX_PARAMS_BUFFER_SIZE);
            rxBuffer.finishRead();
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
        os_sleep(millisecond_t(1));
    }
}

void micro_Command_Uart_RxCpltCallback() {
    if (MAX_PARAMS_BUFFER_SIZE > uart_Debug.handle->hdmarx->Instance->NDTR) {
        rxBuffer.finishWrite();
    }
    uart_receive(uart_Debug, *rxBuffer.startWrite(), MAX_PARAMS_BUFFER_SIZE);
}

void micro_Command_Uart_TxCpltCallback() {
    txSemaphore.give();
}
