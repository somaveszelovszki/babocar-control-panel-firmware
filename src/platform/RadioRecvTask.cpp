#include <micro/debug/TaskMonitor.hpp>
#include <micro/port/gpio.hpp>
#include <micro/port/task.hpp>
#include <micro/log/log.hpp>
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
    char prevRadioRecv = radioRecvValue;
    uart_receive(uart_RadioModule, radioRecvBuffer, 1);

    taskMonitor.registerInitializedTask();

    while (true) {
        const char radioRecv = radioRecvValue;
        if (radioRecv != prevRadioRecv) {
            LOG_INFO("Received character: {}", radioRecv);
            prevRadioRecv = radioRecv;
        }

        radioRecvQueue.overwrite(radioRecv);
        taskMonitor.notify(true);
        os_sleep(millisecond_t(20));
    }
}

/* @brief Callback for RadioModule UART RxCplt - called when receive finishes.
 */
void micro_RadioModule_Uart_RxCpltCallback() {
    radioRecvValue = static_cast<char>(radioRecvBuffer[0]);
}
