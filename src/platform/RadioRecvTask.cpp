#include <micro/debug/SystemManager.hpp>
#include <micro/port/gpio.hpp>
#include <micro/port/task.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/timer.hpp>

#include <cfg_board.hpp>
#include <cfg_track.hpp>

using namespace micro;

queue_t<char, 1> radioRecvQueue;

namespace {
volatile millisecond_t lastRxTime;
} // namespace

extern "C" void runRadioRecvTask(void) {

    SystemManager::instance().registerTask();

    millisecond_t lastQueueSendTime;
    uint8_t lastRadioRecvValue = 0, radioRecvValue = 0;
    uart_receive(uart_RadioModule, &radioRecvValue, 1);

    while (true) {
        if (const_cast<const millisecond_t&>(lastRxTime) != lastQueueSendTime) {
            radioRecvQueue.overwrite(static_cast<char>(radioRecvValue));
            lastQueueSendTime = const_cast<const millisecond_t&>(lastRxTime);
        }

        if (radioRecvValue != lastRadioRecvValue) {
            LOG_INFO("Received character: %c", static_cast<char>(radioRecvValue));
            lastRadioRecvValue = radioRecvValue;
        }

        SystemManager::instance().notify(true);
        os_sleep(millisecond_t(20));
    }
}

/* @brief Callback for RadioModule UART RxCplt - called when receive finishes.
 */
void micro_RadioModule_Uart_RxCpltCallback() {
    const_cast<millisecond_t&>(lastRxTime) = getTime();
}
