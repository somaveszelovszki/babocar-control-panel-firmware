#include <config/cfg_board.hpp>
#include <config/cfg_car.hpp>
#include <config/cfg_os.hpp>
#include <uns/task/Config.hpp>
#include <uns/util/debug.hpp>
#include <uns/globals.hpp>
#include <uns/panel/LineDetectPanel.hpp>
#include <uns/panel/MotorPanel.hpp>

using namespace uns;

extern panel::LineDetectPanel frontLineDetectPanel;
extern panel::LineDetectPanel rearLineDetectPanel;
extern panel::MotorPanel motorPanel;

namespace {

uint8_t startCounterBuffer[1];
volatile char startCounter = '6';   // start counter will count back from 5 to 0

void waitStartSignal() {
    uns::UART_Receive_DMA(cfg::uart_RadioModule, startCounterBuffer, 1);

    while(globals::taskConfig.startSignalEnabled && startCounter != '0') {
        debug::printlog("Seconds until start: %c", startCounter);
        uns::nonBlockingDelay(millisecond_t(50));
    }

    uns::UART_Stop_DMA(cfg::uart_RadioModule);
}

} // namespace

extern "C" void runSetupTask(const void *argument) {
    uns::taskSuspend(cfg::task_Control);
    uns::blockingDelay(millisecond_t(200));     // gives time to auxiliary panels to wake up

    globals::setDefaultTaskConfig();

    waitStartSignal();

    debug::printlog("Starting panel initialization");

    Status status;

    if (!isOk(status = motorPanel.start(globals::taskConfig.useSafetyEnableSignal))) {
        debug::printerr(status, "motorPanel.start failed");
        task::setErrorFlag();
    }

    if (!isOk(status = frontLineDetectPanel.start())) {
        debug::printerr(status, "frontLineDetectPanel.start failed");
        task::setErrorFlag();
    }

    if (!isOk(status = rearLineDetectPanel.start())) {
        debug::printerr(status, "rearLineDetectPanel.start failed");
        task::setErrorFlag();
    }

    uns::taskDeleteCurrent();
}

/* @brief Callback for RadioModule UART RxCplt - called when receive finishes.
 */
void uns_RadioModule_Uart_RxCpltCallback() {
    const uint8_t cntr = static_cast<uint8_t>(startCounterBuffer[0]);
    if (cntr == startCounter - 1) {
        startCounter = cntr;
    }
}
