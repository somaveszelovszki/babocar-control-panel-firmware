#include <micro/debug/SystemManager.hpp>
#include <micro/port/gpio.hpp>
#include <micro/port/task.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/timer.hpp>

#include <cfg_board.hpp>
#include <cfg_track.hpp>
#include <globals.hpp>

using namespace micro;

namespace {

uint8_t radioRecvBuffer[1] = { 0 };
volatile char radioRecvValue = '\0';

} // namespace

extern "C" void runRadioRecvTask(void) {

    SystemManager::instance().registerTask();

    char prevRadioRecv = radioRecvValue;
    uart_receive(uart_RadioModule, radioRecvBuffer, 1);

    while (true) {
        const char radioRecv = radioRecvValue;
        if (radioRecv != prevRadioRecv) {
            LOG_INFO("Received character: %c", radioRecv);
            prevRadioRecv = radioRecv;
        }

        radioRecvQueue.overwrite(radioRecv);
        SystemManager::instance().notify(true);
        os_sleep(millisecond_t(20));
    }
}

/* @brief Callback for RadioModule UART RxCplt - called when receive finishes.
 */
void micro_RadioModule_Uart_RxCpltCallback() {
    radioRecvValue = static_cast<char>(radioRecvBuffer[0]);
}
