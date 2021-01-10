#include <micro/debug/SystemManager.hpp>
#include <micro/port/gpio.hpp>
#include <micro/port/task.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/state.hpp>
#include <micro/utils/timer.hpp>

#include <cfg_board.hpp>
#include <cfg_track.hpp>

using namespace micro;

queue_t<state_t<char>, 1> radioRecvQueue;

namespace {
uint8_t radioRecvBuffer[1] = { 0 };
volatile state_t<char> radioRecvValue('\0', millisecond_t(0));
} // namespace

extern "C" void runRadioRecvTask(void) {

    SystemManager::instance().registerTask();

    millisecond_t lastQueueSendTime;
    uart_receive(uart_RadioModule, radioRecvBuffer, 1);

    while (true) {
        criticalSection_t criticalSection;
        criticalSection.lock();
        const state_t<char> radioRecv = const_cast<const state_t<char>&>(radioRecvValue);
        criticalSection.unlock();

        if (lastQueueSendTime != radioRecv.timestamp()) {
            radioRecvQueue.overwrite(radioRecv);
            LOG_INFO("Received character: %c", radioRecv.value());
            lastQueueSendTime = radioRecv.timestamp();
        }

        SystemManager::instance().notify(true);
        os_sleep(millisecond_t(20));
    }
}

/* @brief Callback for RadioModule UART RxCplt - called when receive finishes.
 */
void micro_RadioModule_Uart_RxCpltCallback() {
    const_cast<state_t<char>&>(radioRecvValue).set(static_cast<char>(radioRecvBuffer[0]));
}
