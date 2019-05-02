#include <uns/task/common.hpp>
#include <config/cfg_board.hpp>
#include <config/cfg_car.hpp>
#include <uns/util/debug.hpp>

using namespace uns;

extern ProgramTask PROGRAM_TASK;

namespace {

char startCounterBuffer;
volatile char startCounter = '6';   // start counter will count back from 5 to 0
volatile char cntrs[100];

} // namespace

extern "C" void runRadioStartTask(const void *argument) {

    if (cfg::START_SIGNAL_ENABLED) {
        uns::UART_Receive_DMA(cfg::uart_RadioModule, reinterpret_cast<uint8_t*>(&startCounterBuffer), 1);
        while(startCounter != '0') {
            debug::printlog("Seconds until start: %c", startCounter);
            uns::nonBlockingDelay(millisecond_t(50));
        }
    }

    uns::deleteCurrentTask();
}

/* @brief Callback for RadioModule UART RxCplt - called when receive finishes.
 */
void uns_RadioModule_Uart_RxCpltCallback() {
    if (startCounterBuffer == startCounter - 1) {
        startCounter = startCounterBuffer;
    }
}
